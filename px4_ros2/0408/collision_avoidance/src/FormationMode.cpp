#include <collision_avoidance/FormationMode.hpp>

using namespace px4_msgs::msg;


FormationMode::FormationMode(rclcpp::Node & node, int vehicle_id, int total_agent_num)
: ModeBase(node, Settings{"VTOL Formation"}, "/px4_" + std::to_string(vehicle_id) + "/")
, _node(node)
{
    m_vehicle_id      = vehicle_id;
    m_total_agent_num = total_agent_num;

    _fw_setpoint = std::make_shared<px4_ros2::FwLateralLongitudinalSetpointType>(*this);

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    /* 바람 구독 (airspeed 보상용) — 콜백은 main thread */
    _wind_sub = _node.create_subscription<Wind>(
        "/px4_" + std::to_string(m_vehicle_id) + "/fmu/out/wind", qos,
        [this](const Wind::UniquePtr msg) {
            m_wind_n_mt2rt.store(msg->windspeed_north, std::memory_order_relaxed);
            m_wind_e_mt2rt.store(msg->windspeed_east,  std::memory_order_relaxed);
        });

    /* 공통 좌표계 odometry 구독 (모든 기체) */
    _trans_odom_subs.resize(m_total_agent_num);
    for (int n = 0; n < m_total_agent_num; n++) {
        _trans_odom_subs[n] = _node.create_subscription<VehicleOdometry>(
            "/common/px4_" + std::to_string(n) + "/trans_vehicle_odometry",
            qos,
            [this, n](const VehicleOdometry::UniquePtr msg) {
                m_state_for_control_mt[n].check_vehicle_id  = n;
                m_state_for_control_mt[n].position          = msg->position;
                m_state_for_control_mt[n].velocity          = msg->velocity;
                m_state_for_control_mt[n].position_variance = msg->position_variance;
                m_state_for_control_mt[n].velocity_variance = msg->velocity_variance;
                m_state_for_control_mt[n].timestamp         = 0.0;

                m_agent_updated_mt[n] = true;

                bool all_updated = true;
                for (int i = 0; i < m_total_agent_num; i++) {
                    if (!m_agent_updated_mt[i]) { all_updated = false; break; }
                }

                if (all_updated) {
                    StateType::Total_state_for_Control_mt2rt snapshot{};
                    snapshot.num_agents = m_total_agent_num;
                    snapshot.agents     = m_state_for_control_mt;
                    m_input_queue_mt2rt.try_push(snapshot);
                    m_agent_updated_mt.fill(false);
                }
            });
    }

    /* Flocking 파라미터 yaml 로드 */
    FlockingGuidance::Parameters flocking_params;
    auto declare_or_get = [&](const std::string& name, float default_val) -> float {
        if (!_node.has_parameter(name)) {
            _node.declare_parameter<float>(name, default_val);
        }
        return static_cast<float>(_node.get_parameter(name).as_double());
    };
    flocking_params.lambda           = declare_or_get("flocking_lambda", 1.0f);
    flocking_params.beta             = declare_or_get("flocking_beta", 0.5f);
    flocking_params.k1               = declare_or_get("flocking_k1", 0.5f);
    flocking_params.k2               = declare_or_get("flocking_k2", 1.0f);
    flocking_params.desired_distance = declare_or_get("flocking_desired_distance", 30.0f);
    flocking_params.neighbor_count   = (m_total_agent_num > 0) ? (m_total_agent_num - 1) : 4;
    flocking_params.integration_dt   = declare_or_get("integration_dt", 0.0333f);

    m_flocking = std::make_unique<FlockingGuidance>(flocking_params);

    RCLCPP_INFO(_node.get_logger(),
        "[Formation] Flocking 파라미터: lambda=%.2f beta=%.2f k1=%.2f k2=%.4f d=%.2f neighbors=%d dt=%.4f",
        flocking_params.lambda, flocking_params.beta,
        flocking_params.k1, flocking_params.k2,
        flocking_params.desired_distance, flocking_params.neighbor_count,
        flocking_params.integration_dt);

    /* rt_thread 는 노드 생성 시점부터 항상 돌도록 시작. */
    m_rt_running_mt2rt.store(true);
    m_rt_thread_mt = std::thread(&FormationMode::rt_loop, this);
}

FormationMode::~FormationMode()
{
    m_rt_running_mt2rt.store(false);
    if (m_rt_thread_mt.joinable()) { m_rt_thread_mt.join(); }
}

void FormationMode::onActivate()
{
    RCLCPP_INFO(_node.get_logger(),
        "[Formation] 활성화 (cruise alt=%.1f m, 초기 코스=%.2f rad)",
        m_cruise_altitude_amsl, m_initial_course);

    /* main thread 는 V_desired 를 직접 건드리지 않음. rt_thread 에 재초기화 신호만 보냄. */
    m_reinit_vdesired_mt2rt.store(true);

    /* main thread 의 hold-last 상태 초기화 — 새 활성화 사이클에서는 처음부터 다시 */
    m_has_last_output_mt = false;
    m_last_output_mt = StateType::FwSetpointOutput_rt2mt{};
}

void FormationMode::onDeactivate()
{
    RCLCPP_INFO(_node.get_logger(), "[Formation] 비활성화");
    /* rt_thread 는 소멸자에서 정지. 비활성화 시에는 안 멈춤
       (재활성화 시점에 다시 시작하느라 워밍업 갭 생기는 것 방지) */
}

/* ──────────────────────────────────────────────────────────────
   updateSetpoint — output_queue 에서 pop 해서 PX4 에 인가만.
   산수는 전혀 없음 (전부 rt_thread 가 수행).
   ────────────────────────────────────────────────────────────── */
void FormationMode::updateSetpoint(float /*dt_s*/)
{
    /* (1) rt_thread 가 push 한 최신 setpoint pop */
    std::optional<StateType::FwSetpointOutput_rt2mt> maybe_out =
        m_output_queue_rt2mt.try_pop();

    if (maybe_out.has_value()) {
        m_last_output_mt     = maybe_out.value();
        m_has_last_output_mt = true;
    } else if (m_has_last_output_mt) {
        /* (2) queue 비었지만 이전에 한 번이라도 받아본 적 있음 → hold last (ZOH) */
        RCLCPP_WARN_THROTTLE(_node.get_logger(), *_node.get_clock(), 1000,
            "[Formation] output_queue 비어있음 → hold last (ZOH)");
    } else {
        /* (3) 활성화 직후 rt_thread 가 아직 첫 결과를 push 못함 → cruise fallback */
        if (std::isfinite(m_cruise_altitude_amsl)) {
            _fw_setpoint->updateWithAltitude(
                m_cruise_altitude_amsl, m_initial_course, m_initial_ground_speed);
        } else {
            _fw_setpoint->updateWithHeightRate(0.f, m_initial_course, m_initial_ground_speed);
        }
        return;
    }

    /* (4) m_last_output_mt 값 인가. is_fallback 이면 cruise. */
    if (m_last_output_mt.is_fallback) {
        if (std::isfinite(m_cruise_altitude_amsl)) {
            _fw_setpoint->updateWithAltitude(
                m_cruise_altitude_amsl, m_initial_course, m_initial_ground_speed);
        } else {
            _fw_setpoint->updateWithHeightRate(0.f, m_initial_course, m_initial_ground_speed);
        }
    } else {
        _fw_setpoint->updateWithHeightRate(
            m_last_output_mt.height_rate,
            m_last_output_mt.course,
            m_last_output_mt.airspeed);
    }
}

/* ──────────────────────────────────────────────────────────────
   rt_loop — snapshot 을 FlockingGuidance 에 넘기고 결과를 output_queue 로.
   모든 계산(가속도, 적분, saturation, 변환, fallback) 은 FlockingGuidance 안.
   ────────────────────────────────────────────────────────────── */
void FormationMode::rt_loop()
{
    bool first_push_done = false;

    while (m_rt_running_mt2rt.load()) {
        /* (0) 활성화 직후 재초기화 신호 처리 */
        if (m_reinit_vdesired_mt2rt.exchange(false)) {
            m_flocking->resetVdesired(m_initial_course, m_initial_ground_speed);
            first_push_done = false;
        }

        std::optional<StateType::Total_state_for_Control_mt2rt> input_state =
            m_input_queue_mt2rt.try_pop();

        if (!input_state.has_value() || !m_flocking) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        const auto & snapshot = input_state.value();
        const int self_idx    = m_vehicle_id;

        if (self_idx < 0 || self_idx >= snapshot.num_agents) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        /* (1) self 상태 추출 */
        StateType::AgentState_rt self;
        self.pos_n = snapshot.agents[self_idx].position[0];
        self.pos_e = snapshot.agents[self_idx].position[1];
        self.vel_n = snapshot.agents[self_idx].velocity[0];
        self.vel_e = snapshot.agents[self_idx].velocity[1];

        /* (2) others 를 정적 버퍼 m_others_buf_rt 에 채움 — heap 할당 0 */
        int num_others = 0;
        for (int i = 0; i < snapshot.num_agents; i++) {
            if (i == self_idx) continue;
            if (num_others >= static_cast<int>(kMaxAgents)) break;  /* 안전 가드 */
            StateType::AgentState_rt & s = m_others_buf_rt[num_others];
            s.pos_n = snapshot.agents[i].position[0];
            s.pos_e = snapshot.agents[i].position[1];
            s.vel_n = snapshot.agents[i].velocity[0];
            s.vel_e = snapshot.agents[i].velocity[1];
            num_others++;
        }

        /* (3) 가이던스 한 방에 — 가속도/적분/saturation/변환/fallback 전부 내부에서 */
        const float wind_n = m_wind_n_mt2rt.load(std::memory_order_relaxed);
        const float wind_e = m_wind_e_mt2rt.load(std::memory_order_relaxed);
        const StateType::FwSetpointOutput_rt2mt out =
            m_flocking->computeFwSetpoint(self, m_others_buf_rt, num_others,
                                          wind_n, wind_e);

        /* (4) output_queue 로 push */
        m_output_queue_rt2mt.try_push(out);

        if (!first_push_done) {
            first_push_done = true;
            RCLCPP_INFO(_node.get_logger(),
                "[Formation] 첫 결과 push: course=%.2f airspeed=%.2f hr=%.2f fallback=%d",
                out.course, out.airspeed, out.height_rate, out.is_fallback);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
