#include <collision_avoidance/modes/FormationMode.hpp>

#include <collision_avoidance/control/GroundToEasAdapter.hpp>
#include <collision_avoidance/communication/ManeuverBudgetTraceMessage.hpp>

#include <limits>
#include <chrono>

using namespace px4_msgs::msg;


FormationMode::FormationMode(rclcpp::Node & node, int vehicle_id, int total_agent_num)
: ModeBase(node, Settings{"VTOL Formation"}, "/px4_" + std::to_string(vehicle_id) + "/")
, _node(node)
{
    m_vehicle_id      = vehicle_id;
    m_total_agent_num = total_agent_num;
    if (_node.has_parameter("masd_diagnostics_enabled")
        && _node.get_parameter("masd_diagnostics_enabled").as_bool()) {
        m_budget_trace_publisher = _node.create_publisher<
            collision_avoidance::msg::ManeuverBudgetTrace>(
            "/common/px4_" + std::to_string(vehicle_id) + "/maneuver_budget_trace",
            rclcpp::QoS(128).best_effort());
    }

    _fw_setpoint = std::make_shared<px4_ros2::FwLateralLongitudinalSetpointType>(*this);
    _vtol_status = std::make_shared<px4_ros2::VtolStatus>(*this);

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
    _vehicle_air_data_sub = _node.create_subscription<VehicleAirData>(
        "/px4_" + std::to_string(m_vehicle_id) + "/fmu/out/vehicle_air_data", qos,
        [this](const VehicleAirData::UniquePtr msg) {
            if (std::isfinite(msg->rho) && msg->rho > 0.0F) {
                m_air_density_mt = msg->rho;
            }
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
                if (n == m_vehicle_id) {
                    m_latest_self_state_timestamp_us_mt = msg->timestamp;
                    m_latest_self_state_received_steady_us_mt =
                        std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::steady_clock::now().time_since_epoch()).count();
                }

                /* ── alt_hold P-제어 reference 캡처 ──
                   self 의 첫 유효 odometry 시 한 번만 캡처. onActivate 가 valid=false
                   로 리셋해두므로, 매 활성화마다 새로운 reference 가 잡힘.
                   캡처 NED z (보통 음수, takeoff 후). */
                if (n == m_vehicle_id && !m_ref_pos_d_valid_mt &&
                    std::isfinite(msg->position[2])) {
                    m_ref_pos_d_mt       = msg->position[2];
                    m_ref_pos_d_valid_mt = true;
                    RCLCPP_INFO(_node.get_logger(),
                        "[Formation] alt_hold reference 캡처: ref_z=%.2f m (NED)",
                        m_ref_pos_d_mt);
                }

                m_agent_updated_mt[n] = true;

                bool all_updated = true;
                for (int i = 0; i < m_total_agent_num; i++) {
                    if (!m_agent_updated_mt[i]) { all_updated = false; break; }
                }

                if (all_updated) {
                    collision_avoidance::types::ControlSnapshot snapshot{};
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
    /* ── Flocking 알고리즘 파라미터 (flocking_params.yaml) ── */
    flocking_params.lambda           = declare_or_get("flocking_lambda", 1.0f);
    flocking_params.beta             = declare_or_get("flocking_beta", 0.5f);
    flocking_params.k1               = declare_or_get("flocking_k1", 0.5f);
    flocking_params.k2               = declare_or_get("flocking_k2", 1.0f);
    flocking_params.desired_distance = declare_or_get("flocking_desired_distance", 30.0f);
    flocking_params.neighbor_count   = (m_total_agent_num > 0) ? (m_total_agent_num - 1) : 4;
    flocking_params.integration_dt   = declare_or_get("integration_dt", 0.0333f);

    /* ── Airframe spec / safety limit (airframe_spec.yaml) ── */
    flocking_params.airspeed_min               = declare_or_get("airspeed_min", 10.0f);
    flocking_params.airspeed_max               = declare_or_get("airspeed_max", 25.0f);
    flocking_params.height_rate_max_climb      = declare_or_get("height_rate_max_climb", 8.0f);
    flocking_params.height_rate_max_sink       = declare_or_get("height_rate_max_sink",  2.7f);
    flocking_params.max_roll_deg               = declare_or_get("max_roll_deg", 50.0f);

    /* ── alt_hold (수직 가속도) P 게인 (airframe_spec.yaml) ── */
    flocking_params.alt_hold_p_gain            = declare_or_get("alt_hold_p_gain", 0.1f);

    const float gravity_mps2 = declare_or_get("gravity", 9.80665f);

    m_minimum_level_eas = flocking_params.airspeed_min;
    m_gravity = gravity_mps2;

    m_flocking = std::make_unique<FlockingGuidance>(
        flocking_params, gravity_mps2);

    if (!_node.has_parameter("test_guidance_mode")) {
        _node.declare_parameter<std::string>("test_guidance_mode", "formation");
    }
    m_point_convergence_test_mode =
        _node.get_parameter("test_guidance_mode").as_string()
        == "point_convergence";
    collision_avoidance::guidance::PointConvergenceGuidance::Parameters
        point_params;
    point_params.target_north_m = declare_or_get("point_target_north_m", 300.0F);
    point_params.target_east_m = declare_or_get("point_target_east_m", 300.0F);
    point_params.ground_speed_command_mps = declare_or_get(
        "point_ground_speed_command_mps", 20.0F);
    point_params.course_error_gain_per_s = declare_or_get(
        "point_course_error_gain_per_s", 1.2F);
    point_params.maximum_roll_degrees = flocking_params.max_roll_deg;
    point_params.gravity_mps2 = gravity_mps2;
    m_point_convergence = std::make_unique<
        collision_avoidance::guidance::PointConvergenceGuidance>(point_params);

    /* ── 고도 P-제어 파라미터 (airframe_spec.yaml) ── */
    m_alt_hold_p_gain = declare_or_get("alt_hold_p_gain", 0.5f);
    m_alt_hold_hr_max = declare_or_get("alt_hold_hr_max", 2.7f);

    RCLCPP_INFO(_node.get_logger(),
        "[Formation] Flocking params: lambda=%.2f beta=%.2f k1=%.2f k2=%.4f d=%.2f n=%d dt=%.4f",
        flocking_params.lambda, flocking_params.beta,
        flocking_params.k1, flocking_params.k2,
        flocking_params.desired_distance, flocking_params.neighbor_count,
        flocking_params.integration_dt);
    RCLCPP_INFO(_node.get_logger(),
        "[Formation] Airframe spec: airspeed=[%.1f,%.1f] hr_climb=%.1f hr_sink=%.1f roll_max=%.1f",
        flocking_params.airspeed_min, flocking_params.airspeed_max,
        flocking_params.height_rate_max_climb,
        flocking_params.height_rate_max_sink,
        flocking_params.max_roll_deg);
    RCLCPP_INFO(_node.get_logger(),
        "[Formation] Alt P-control: p_gain=%.2f hr_max=%.1f",
        m_alt_hold_p_gain, m_alt_hold_hr_max);

    /* rt_thread 는 노드 생성 시점부터 항상 돌도록 시작. */
    m_rt_running_mt2rt.store(true);
    m_rt_thread_mt = std::thread(&FormationMode::rt_loop, this);



}

FormationMode::~FormationMode()
{  /*메인 스레드 종료 시 스레드 회수를 위한 소멸자*/
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
    m_last_output_mt = collision_avoidance::types::FwSetpoint{};
    m_has_maneuver_decision_mt = false;
    if (m_maneuver_activation_gate_callback_mt) {
        m_maneuver_activation_gate_callback_mt(true);
    }

    /* 고도 P-제어 기준점 초기화 — 첫 유효 odometry 수신 시 캡처 */
    m_ref_pos_d_valid_mt = false;
}

void FormationMode::onDeactivate()
{
    recordPublishedSetpoint(0.0F, 0.0F, false);
    RCLCPP_INFO(_node.get_logger(), "[Formation] 비활성화");
    if (m_maneuver_activation_gate_callback_mt) {
        m_maneuver_activation_gate_callback_mt(false);
    }
    /* rt_thread 는 소멸자에서 정지. 비활성화 시에는 안 멈춤
       (재활성화 시점에 다시 시작하느라 워밍업 갭 생기는 것 방지) */
}

/* ──────────────────────────────────────────────────────────────
   updateSetpoint — output_queue 에서 pop 하고 PX4 경계의 EAS 변환 후 인가.
   guidance 계산은 rt_thread 가 수행.
   ────────────────────────────────────────────────────────────── */
void FormationMode::updateSetpoint(float /*dt_s*/)
{
    const float wind_n = m_wind_n_mt2rt.load(std::memory_order_relaxed);
    const float wind_e = m_wind_e_mt2rt.load(std::memory_order_relaxed);
    const auto to_eas = [wind_n, wind_e, this](
                            float v_cmd, float course, float height_rate,
                            float lateral_acceleration) {
        const float raw_eas =
            collision_avoidance::control::computeRequiredEquivalentAirspeed(
            v_cmd,
            course,
            height_rate,
            wind_n,
            wind_e,
            0.0F,
            m_air_density_mt);
        return collision_avoidance::control::applyTurnMinimumEquivalentAirspeed(
            raw_eas, m_minimum_level_eas, lateral_acceleration, m_gravity);
    };

    /* (0) VTOL 고정익 모드 확인 — FW 가 아니면 TECS 미작동 가능 */
    if (!_vtol_status->isFwMode()) {
        RCLCPP_ERROR_THROTTLE(_node.get_logger(), *_node.get_clock(), 1000,
            "[Formation] VTOL이 고정익 모드가 아님! (state=%d) → TECS 미작동 위험",
            static_cast<int>(_vtol_status->last().vehicle_vtol_state));
    }

    /* 10초마다 현재 상태 출력 */
    RCLCPP_INFO_THROTTLE(_node.get_logger(), *_node.get_clock(), 10000,
        "[Formation] vehicle=%d | vtol_state=%d(%s) | has_output=%d | fallback=%d | "
        "ground_course=%.2f v_cmd_ground=%.1f alt_sp=%.1f",
        m_vehicle_id,
        static_cast<int>(_vtol_status->last().vehicle_vtol_state),
        _vtol_status->isFwMode() ? "FW" :
            (_vtol_status->isMcMode() ? "MC" :
            (_vtol_status->isTransitioningToFw() ? "TRANS_FW" : "OTHER")),
        m_has_last_output_mt ? 1 : 0,
        m_last_output_mt.is_fallback ? 1 : 0,
        m_last_output_mt.course,
        m_last_output_mt.airspeed,
        m_cruise_altitude_amsl);

    /* (1) rt_thread 가 push 한 최신 setpoint pop */
    std::optional<collision_avoidance::types::FwSetpoint> maybe_out =
        m_output_queue_rt2mt.try_pop();

    if (maybe_out.has_value()) {
        m_last_output_mt     = maybe_out.value();
        m_has_last_output_mt = true;
    } else if (m_has_last_output_mt) {
        /* (2) queue 비었지만 이전에 한 번이라도 받아본 적 있음 → hold last (ZOH) */
        RCLCPP_WARN_THROTTLE(_node.get_logger(), *_node.get_clock(), 1000,
            "[Formation] output_queue 비어있음 → hold last (ZOH)");
    }

    if (m_nominal_setpoint_callback_mt) {
        collision_avoidance::selection::
            ManeuverSelectionNominalSetpointSnapshot snapshot;
        snapshot.timestamp_us = m_latest_self_state_timestamp_us_mt;
        snapshot.valid = m_has_last_output_mt
            && !m_last_output_mt.is_fallback
            && snapshot.timestamp_us > 0;
        if (snapshot.valid) {
            snapshot.ground_speed_command_mps = m_last_output_mt.airspeed;
            snapshot.altitude_command_m =
                std::isfinite(m_last_output_mt.height_setpoint)
                ? -static_cast<double>(m_last_output_mt.height_setpoint)
                : std::numeric_limits<double>::quiet_NaN();
            snapshot.lateral_acceleration_px4_mps2 =
                m_last_output_mt.lateral_acceleration;
        }
        m_nominal_setpoint_callback_mt(snapshot);
    }

    if (avoidanceOverrideRequested()) {
        publishAvoidanceSetpoint();
        return;
    }

    if (!m_has_last_output_mt) {
        recordPublishedSetpoint(0.0F, 0.0F, false);
        /* (3) 활성화 직후 rt_thread 가 아직 첫 결과를 push 못함 → cruise fallback */
        const float v_cmd_eas = to_eas(
            m_initial_ground_speed, m_initial_course, 0.0F, 0.0F);
        if (std::isfinite(m_cruise_altitude_amsl)) {
            _fw_setpoint->updateWithAltitude(
                m_cruise_altitude_amsl, m_initial_course, v_cmd_eas);
        } else {
            _fw_setpoint->updateWithHeightRate(
                0.f, m_initial_course, v_cmd_eas);
        }
        return;
    }

    /* (4) m_last_output_mt 값 인가. is_fallback 이면 cruise. */
    if (m_last_output_mt.is_fallback) {
        recordPublishedSetpoint(0.0F, 0.0F, false);
        const float v_cmd_eas = to_eas(
            m_initial_ground_speed, m_initial_course, 0.0F, 0.0F);
        if (std::isfinite(m_cruise_altitude_amsl)) {
            _fw_setpoint->updateWithAltitude(
                m_cruise_altitude_amsl, m_initial_course, v_cmd_eas);
        } else {
            _fw_setpoint->updateWithHeightRate(
                0.f, m_initial_course, v_cmd_eas);
        }
    } else {
        /* 2D flocking → lateral_acceleration + ground-speed command.
           현재 ground course로 EAS feed-forward만 계산하며 ground-speed 추종을
           보장하는 별도 제어기는 아니다.
           course 를 설정하지 않으면(NAN) lateral_acceleration 이 직접 제어 입력이 됨.
           현재 이 경로는 수직 채널을 PX4에 보내지 않으므로 EAS 변환에도
           height_rate=0 을 사용한다. */
        const float horizontal_ground_speed = std::sqrt(std::fmax(
            0.0F,
            m_last_output_mt.airspeed * m_last_output_mt.airspeed
                - m_last_output_mt.height_rate * m_last_output_mt.height_rate));
        const float v_cmd_eas = to_eas(
            horizontal_ground_speed, m_last_output_mt.course, 0.0F,
            m_last_output_mt.lateral_acceleration);
        px4_ros2::FwLateralLongitudinalSetpoint sp;
        sp.withLateralAcceleration(m_last_output_mt.lateral_acceleration)
          .withEquivalentAirspeed(v_cmd_eas);
        collision_avoidance::selection::ManeuverBudgetTrace trace;
        if (m_budget_trace_publisher) collision_avoidance::selection::stampBudgetTrace(trace);
        _fw_setpoint->update(sp);
        recordPublishedSetpoint(horizontal_ground_speed,
                               m_last_output_mt.lateral_acceleration, true);
        traceSetpoint(trace.wall_ns, trace.steady_ns, false,
                      m_last_output_mt.lateral_acceleration, horizontal_ground_speed, v_cmd_eas);
    }
}

// Executor-thread command delivery shared by decision events and the
// existing periodic setpoint refresh. No guidance recomputation here.
bool FormationMode::avoidanceOverrideRequested() const
{
    return !m_collision_avoidance_shadow_only_mt
        && m_has_maneuver_decision_mt
        && m_maneuver_decision_mt.coordination_qualified
        && m_maneuver_decision_mt.command_execution_requested;
}

void FormationMode::setManeuverSelectionDecision(
    const collision_avoidance::selection::ManeuverSelectionDecision & decision)
{
    const bool previously_requested = avoidanceOverrideRequested();
    const auto previous_input = m_maneuver_decision_mt.ownship_input;
    m_maneuver_decision_mt = decision;
    m_has_maneuver_decision_mt = true;

    // Runs on the same executor as updateSetpoint. An executable new command
    // need not wait for the next periodic setpoint tick. Decision heartbeats
    // still use the existing periodic publisher, not extra immediate writes.
    if (isActive() && avoidanceOverrideRequested()
        && (!previously_requested
            || previous_input.a_lat_cmd != decision.ownship_input.a_lat_cmd
            || previous_input.V_cmd != decision.ownship_input.V_cmd)) {
        publishAvoidanceSetpoint();
    }
}

void FormationMode::publishAvoidanceSetpoint()
{
    float course = m_initial_course;
    if (m_vehicle_id >= 0
        && m_vehicle_id < static_cast<int>(m_state_for_control_mt.size())) {
        const auto & velocity = m_state_for_control_mt[m_vehicle_id].velocity;
        if (std::isfinite(velocity[0]) && std::isfinite(velocity[1])
            && std::hypot(velocity[0], velocity[1]) > 1.0e-3F) {
            course = std::atan2(velocity[1], velocity[0]);
        }
    }
    const float lateral_acceleration =
        static_cast<float>(m_maneuver_decision_mt.ownship_input.a_lat_cmd);
    const float v_cmd_ground =
        static_cast<float>(m_maneuver_decision_mt.ownship_input.V_cmd);
    const float raw_eas =
        collision_avoidance::control::computeRequiredEquivalentAirspeed(
            v_cmd_ground, course, 0.0F,
            m_wind_n_mt2rt.load(std::memory_order_relaxed),
            m_wind_e_mt2rt.load(std::memory_order_relaxed),
            0.0F, m_air_density_mt);
    const float v_cmd_eas =
        collision_avoidance::control::applyTurnMinimumEquivalentAirspeed(
            raw_eas, m_minimum_level_eas, lateral_acceleration, m_gravity);
    px4_ros2::FwLateralLongitudinalSetpoint sp;
    sp.withLateralAcceleration(lateral_acceleration)
      .withEquivalentAirspeed(v_cmd_eas);
    collision_avoidance::selection::ManeuverBudgetTrace trace;
    if (m_budget_trace_publisher) collision_avoidance::selection::stampBudgetTrace(trace);
    _fw_setpoint->update(sp);
    recordPublishedSetpoint(v_cmd_ground, lateral_acceleration, true);
    traceSetpoint(trace.wall_ns, trace.steady_ns, true, lateral_acceleration,
                  v_cmd_ground, v_cmd_eas);
    RCLCPP_WARN_THROTTLE(
        _node.get_logger(), *_node.get_clock(), 1000,
        "[Formation] collision-avoidance override: candidate=%u AD=%.2f "
        "a_lat=%.2f v_ground=%.2f",
        static_cast<unsigned>(m_maneuver_decision_mt.ownship_candidate_id),
        m_maneuver_decision_mt.ad_m, lateral_acceleration, v_cmd_ground);
}

void FormationMode::recordPublishedSetpoint(
    float ground_speed, float lateral_acceleration, bool valid)
{
    if (!m_published_setpoint_callback_mt
        || m_latest_self_state_timestamp_us_mt == 0) return;
    const auto now_us = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    collision_avoidance::selection::ManeuverSelectionPublishedSetpointSnapshot snapshot;
    // Stay in the ownship/common-state clock on ARM and x86. Local wall clocks
    // need not agree. This receive-time anchor is approximate: odometry
    // transport age and subsequent PX4 command receipt are not measured here.
    snapshot.timestamp_us = m_latest_self_state_timestamp_us_mt
        + (now_us - m_latest_self_state_received_steady_us_mt);
    // Receive jitter must not reverse the order of actual publications.
    snapshot.timestamp_us = std::max(snapshot.timestamp_us,
        m_last_published_setpoint_timestamp_us_mt + 1);
    m_last_published_setpoint_timestamp_us_mt = snapshot.timestamp_us;
    // These lateral paths publish EAS, not an altitude/height-rate command.
    // Retain the predictor's existing ground-speed and level-flight convention.
    snapshot.input = {ground_speed, std::numeric_limits<double>::quiet_NaN(),
                      0.0, lateral_acceleration};
    snapshot.valid = valid;
    m_published_setpoint_callback_mt(snapshot);
}

void FormationMode::traceSetpoint(
    std::uint64_t begin_wall_ns, std::uint64_t begin_steady_ns, bool avoidance,
    float lateral_acceleration, float ground_speed, float eas)
{
    if (!m_budget_trace_publisher) return;
    collision_avoidance::selection::ManeuverBudgetTrace trace;
    collision_avoidance::selection::stampBudgetTrace(trace);
    trace.publish_end_wall_ns = trace.wall_ns;
    trace.wall_ns = begin_wall_ns;
    trace.steady_ns = begin_steady_ns;
    trace.event = 4;
    trace.vehicle_id = m_vehicle_id;
    trace.state_timestamp_us = m_latest_self_state_timestamp_us_mt;
    trace.epoch = m_maneuver_decision_mt.local_selection_epoch;
    trace.candidate_id = m_maneuver_decision_mt.ownship_candidate_id;
    trace.input_revision = m_maneuver_decision_mt.selected_candidate_input_revisions[m_vehicle_id];
    trace.active = avoidance;
    trace.lateral_acceleration_mps2 = lateral_acceleration;
    trace.ground_speed_command_mps = ground_speed;
    trace.equivalent_airspeed_command_mps = eas;
    m_budget_trace_publisher->publish(
        collision_avoidance::communication::budgetTraceMessage(trace));
}

void FormationMode::rt_loop()
{
    bool first_push_done = false;

    while (m_rt_running_mt2rt.load()) {
        /* (0) 활성화 직후 재초기화 신호 처리 */
        if (m_reinit_vdesired_mt2rt.exchange(false)) {
            first_push_done = false;
        }

        std::optional<collision_avoidance::types::ControlSnapshot> input_state =
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
        collision_avoidance::types::AgentState self;
            self.pos_n = snapshot.agents[self_idx].position[0];
            self.pos_e = snapshot.agents[self_idx].position[1];
            self.pos_d = snapshot.agents[self_idx].position[2];
            self.vel_n = snapshot.agents[self_idx].velocity[0];
            self.vel_e = snapshot.agents[self_idx].velocity[1];
            self.vel_d = snapshot.agents[self_idx].velocity[2];
            self.speed = std::sqrt(  ( self.vel_n *self.vel_n) + (self.vel_e*self.vel_e)+ (self.vel_d*self.vel_d)  );
            self.psi   = std::atan2(self.vel_e, self.vel_n);   
            const float v_horizontal = std::sqrt(self.vel_n * self.vel_n                                                                                                                                             
                                        + self.vel_e * self.vel_e);
            self.gamma = std::atan2(-self.vel_d, v_horizontal);       

        /* (2) others 를 정적 버퍼 m_others_buf_rt 에 채움 — heap 할당 0 */
        int num_others = 0;
        for (int i = 0; i < snapshot.num_agents; i++) {
            if (i == self_idx) continue;
            if (num_others >= static_cast<int>(
                    collision_avoidance::types::kMaxAgents)) break;  /* 안전 가드 */
            collision_avoidance::types::AgentState & s = m_others_buf_rt[num_others];
                s.pos_n = snapshot.agents[i].position[0];   
                s.pos_e = snapshot.agents[i].position[1];
                s.pos_d = snapshot.agents[i].position[2];
                s.vel_n = snapshot.agents[i].velocity[0];                                                                                                                                                                
                s.vel_e = snapshot.agents[i].velocity[1];
                s.vel_d = snapshot.agents[i].velocity[2];                                                                                                                                                                
                s.speed = std::sqrt(s.vel_n * s.vel_n + s.vel_e * s.vel_e + s.vel_d * s.vel_d);                                                                                                                          
                s.psi   = std::atan2(s.vel_e, s.vel_n);     
                const float v_h = std::sqrt(s.vel_n * s.vel_n + s.vel_e * s.vel_e);                                                                                                                                      
                s.gamma = std::atan2(-s.vel_d, v_h);        

            num_others++;
        }

        /* (3) Guidance acceleration, integration and saturation. */
        /* (3b) height_setpoint — formation 시작 시점에 캡처된 reference NED z (m).
                 m_ref_pos_d_mt 가 아직 캡처 전이면 NaN 으로 전달 → FlockingGuidance 가 무시. */
        const float height_setpoint = m_ref_pos_d_valid_mt
                                          ? m_ref_pos_d_mt
                                          : std::nanf("");

        const collision_avoidance::types::FwSetpoint out =
            m_point_convergence_test_mode && m_point_convergence
            ? m_point_convergence->computeFwSetpoint(self, height_setpoint)
            : m_flocking->computeFwSetpoint(
                self, m_others_buf_rt, num_others, height_setpoint);





        /* (4) output_queue 로 push */
        m_output_queue_rt2mt.try_push(out);

        if (!first_push_done) {
            first_push_done = true;
            RCLCPP_INFO(_node.get_logger(),
                "[Formation] 첫 결과 push: ground_course=%.2f "
                "v_cmd_ground=%.2f hr=%.2f fallback=%d",
                out.course, out.airspeed, out.height_rate, out.is_fallback);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
