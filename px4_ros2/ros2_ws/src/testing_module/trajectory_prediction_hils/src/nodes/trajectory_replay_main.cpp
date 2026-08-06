/* ════════════════════════════════════════════════════════════════════
   main.cpp — trajectory_replay_node 진입점
   ────────────────────────────────────────────────────────────────────
   부팅 흐름:
     [1] rclcpp::Node 생성 ("trajectory_replay_node")
     [2] 파라미터 선언/조회
           - 노드 파라미터: vehicle_ID, sequence_file, csv_path_prefix, log_rate_hz
           - airframe_spec.yaml: airspeed_min/max, height_rate_max_climb/min_sink,
                                  max_roll_deg, gravity, tc_tas, tc_alt, tc_roll
           - replay_params.yaml: predict_horizon_endpoint_s, predict_rate_hz, predict_call_hz
     [3] 컴포넌트 인스턴스화:
           - SetpointSequencer  : yaml 시퀀스 적재
           - TrajectoryLogger   : CSV 파일 오픈 + 토픽 구독 시작
           - VtolPreflightMode  : 천이/안정화 모드
           - TrajectoryReplayMode : ★사용자 모드 (시퀀스 replay)★
     [3.5] TrajectoryPredict 생성 — kPredictHorizon 정합성 검증
     [4] TrajectoryReplayExecutor 생성 (Preflight ↔ Replay 인계)
     [4.5] Predict wall_timer (predict_call_hz) 등록
     [5] doRegister() — Executor + Replay 둘 다 PX4 에 등록
     [6] rclcpp::spin() — main thread 가 콜백 루프 진입

   ※ rt_thread 는 TrajectoryReplayMode 생성자가 자동 시작.
   ════════════════════════════════════════════════════════════════════ */

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>           /* ★ key_samples publisher 용 (Vec3 = float) */
#include <chrono>
#include <cmath>
#include <memory>

#include <trajectory_prediction_hils/modes/VtolPreflightMode.hpp>
#include <trajectory_prediction_hils/replay/TrajectoryReplayMode.hpp>
#include <trajectory_prediction_hils/modes/TrajectoryReplayExecutor.hpp>
#include <trajectory_prediction_hils/replay/SetpointSequencer.hpp>
#include <trajectory_prediction_hils/logging/TrajectoryLogger.hpp>
#include <trajectory_prediction_hils/estimation/BeliefConePublisher.hpp>
/* ★ 2026-05-13 마이그레이션: TrajectoryPredict 헤더는 collision_avoidance 의
   collision_estimation library 로 이동. 본 include 경로는 새 위치 가리킴. */
#include <collision_avoidance/estimation/trajectory_prediction/TrajectoryPredict.hpp>
#include <collision_avoidance/estimation/trajectory_prediction/UncertaintyTypes.hpp>
/* ★ 2026-05-13 테스트용: ReconstructTrajectory 의 debug 함수 사용
   (extractKeySamples 의 결과 검증 / 출력). */
#include <collision_avoidance/estimation/reconstruction/ReconstructTrajectory.hpp>
#include <iostream>     /* std::cout — debugPrintKeySamples 호출 */

using collision_avoidance::estimation::PredictState;
using collision_avoidance::estimation::PredictInput;
using collision_avoidance::estimation::PredictParams;
using collision_avoidance::estimation::TrajectoryPredict;
using collision_avoidance::estimation::TrajectorySample;        /* ★ extractKeySamples 반환 타입 */
using collision_avoidance::estimation::PredictionInputTrajectory;
using collision_avoidance::estimation::UncertaintyParams;


int main(int argc, char * argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    /* [1] Node 생성 */
    auto node = std::make_shared<rclcpp::Node>("trajectory_replay_node");

    /* [2] 파라미터 선언 — 노드 메타 */
    node->declare_parameter<int>("vehicle_ID", 0);
    const std::string package_share =
        ament_index_cpp::get_package_share_directory("trajectory_prediction_hils");
    node->declare_parameter<std::string>(
        "sequence_file", package_share + "/config/setpoint_sequence.yaml");
    node->declare_parameter<std::string>("csv_path_prefix", "/tmp/trajectory");
    node->declare_parameter<double>("log_rate_hz", 50.0);
    /* PX4 토픽 namespace prefix.
       single-vehicle SITL = "" (default) — `/fmu/out/...` 직접
       multi-vehicle SITL  = "/px4_<id>"  — `/px4_<id>/fmu/out/...` */
    node->declare_parameter<std::string>("topic_namespace_prefix", "");

    /* SafetyLimits + 새 명세용 시정수 (airframe_spec.yaml) */
    node->declare_parameter<double>("airspeed_min",            12.0);
    node->declare_parameter<double>("airspeed_max",            25.0);
    node->declare_parameter<double>("height_rate_max_climb",    8.0);
    node->declare_parameter<double>("height_rate_min_sink",     2.7);
    node->declare_parameter<double>("max_roll_deg",            50.0);
    node->declare_parameter<double>("gravity",                  9.80665);
    node->declare_parameter<double>("tc_tas",                   4.0);   /* FW_T_TAS_TC */
    node->declare_parameter<double>("tc_alt",                   2.0);   /* FW_T_ALT_TC */
    node->declare_parameter<double>("tc_roll",                  0.5);   /* FW_R_TC     */
    node->declare_parameter<double>("b_h",                      0.0);   /* ★ 본 작업: altitude term P 게인.
                                                                            0 = 기존 1차 지연 동작 (default). */
    /* ★ beta_phi 파라미터 제거 (2026-05-13 최종) — 위 코드 변경 참조. */
    node->declare_parameter<double>("alt_offset",               0.0);   /* ★ 본 작업: h_cmd = baseline_alt + alt_offset
                                                                            0 = Formation 식 baseline 유지 (default).
                                                                            >0 = baseline 보다 N m 위 target (P-term 자극). */

    /* Predict 메타 (replay_params.yaml).
       ★ 2026-05-13 컨벤션 변경: predict_horizon_s → predict_horizon_endpoint_s.
         의미: "마지막 점의 시각" (endpoint-inclusive). 점 개수 = endpoint × rate + 1.
         예: endpoint=4.5s, rate=10Hz → 46 점 (t=0, 0.1, ..., 4.5). */
    node->declare_parameter<double>("predict_horizon_endpoint_s", 4.5);
    node->declare_parameter<double>("predict_rate_hz",            10.0);
    node->declare_parameter<double>("predict_call_hz",            10.0);

    /* Continuous-time process-noise diagonal for the 7-state covariance model.
       These are initial engineering values; formal bag-based coverage calibration follows. */
    node->declare_parameter<double>("uncertainty.q_pn",   0.25);
    node->declare_parameter<double>("uncertainty.q_pe",   0.25);
    node->declare_parameter<double>("uncertainty.q_h",    0.25);
    node->declare_parameter<double>("uncertainty.q_v",    0.04);
    node->declare_parameter<double>("uncertainty.q_psi",  0.001);
    node->declare_parameter<double>("uncertainty.q_hdot", 0.04);
    node->declare_parameter<double>("uncertainty.q_phi",  0.001);

    const int    vehicle_ID     = node->get_parameter("vehicle_ID").as_int();
    const std::string seq_path  = node->get_parameter("sequence_file").as_string();
    const std::string csv_pref  = node->get_parameter("csv_path_prefix").as_string();
    const double log_rate_hz    = node->get_parameter("log_rate_hz").as_double();

    SetpointSequencer::SafetyLimits limits;
    limits.airspeed_min = static_cast<float>(node->get_parameter("airspeed_min").as_double());
    limits.airspeed_max = static_cast<float>(node->get_parameter("airspeed_max").as_double());
    limits.max_roll_rad =
        static_cast<float>(node->get_parameter("max_roll_deg").as_double()) * M_PI / 180.f;
    limits.gravity      = static_cast<float>(node->get_parameter("gravity").as_double());

    RCLCPP_INFO(node->get_logger(),
        "[main] vehicle_ID=%d sequence_file=%s csv_path=%s",
        vehicle_ID, seq_path.c_str(), csv_pref.c_str());
    RCLCPP_INFO(node->get_logger(),
        "[main] SafetyLimits: V=[%.1f, %.1f] roll_max=%.1f° g=%.3f",
        limits.airspeed_min, limits.airspeed_max,
        limits.max_roll_rad * 180.f / static_cast<float>(M_PI),
        limits.gravity);

    /* [3] 컴포넌트 생성 */
    auto sequencer = std::make_unique<SetpointSequencer>(
        seq_path, limits, node->get_logger());

    const std::string topic_ns =
        node->get_parameter("topic_namespace_prefix").as_string();
    auto logger = std::make_shared<TrajectoryLogger>(
        *node, vehicle_ID, csv_pref, log_rate_hz, topic_ns);

    /* [3.5] TrajectoryPredict 생성 — kPredictHorizon 와 yaml endpoint × rate + 1 정합성 검증.
       ★ 2026-05-13 endpoint-inclusive 컨벤션:
         kPredictHorizon = round(predict_horizon_endpoint_s × predict_rate_hz) + 1
         예: endpoint=4.5s, rate=10Hz → 46 점 (0, 0.1, ..., 4.5 의 양 끝점 포함). */
    const double predict_horizon_endpoint = node->get_parameter("predict_horizon_endpoint_s").as_double();
    const double predict_rate             = node->get_parameter("predict_rate_hz").as_double();
    const double predict_call             = node->get_parameter("predict_call_hz").as_double();

    const std::size_t expected_horizon =
        static_cast<std::size_t>(std::round(predict_horizon_endpoint * predict_rate)) + 1;  /* ★ +1: endpoint 포함 */
    if (expected_horizon != TrajectoryLogger::kPredictHorizon) {
        RCLCPP_FATAL(node->get_logger(),
            "[main] predict horizon 불일치: yaml(endpoint=%.2fs × rate=%.1fHz + 1) = %zu, "
            "TrajectoryLogger::kPredictHorizon (헤더 상수) = %zu. "
            "TrajectoryLogger.hpp 의 kPredictHorizon 또는 yaml 을 맞추세요.",
            predict_horizon_endpoint, predict_rate, expected_horizon,
            TrajectoryLogger::kPredictHorizon);
        throw std::runtime_error("predict horizon mismatch");
    }

    /* PredictParams — airframe_spec.yaml + 시정수 yaml 통합 sourcing */
    PredictParams pp;
    pp.tau_V       = node->get_parameter("tc_tas").as_double();
    pp.tau_hdot    = node->get_parameter("tc_alt").as_double();
    pp.tau_phi     = node->get_parameter("tc_roll").as_double();   /* ★ PATCH: tau_a → tau_phi */
    pp.b_h         = node->get_parameter("b_h").as_double();       /* ★ 본 작업: altitude P 게인 */
    /* ★ 본 작업: h_cmd 의 baseline 위 offset. predict 람다가 capture 해서 사용. */
    const double alt_offset = node->get_parameter("alt_offset").as_double();
    pp.V_min       = limits.airspeed_min;
    pp.V_max       = limits.airspeed_max;
    /* climb / sink 비대칭 → 보수적으로 min(climb, |sink|) 사용. plan TODO(question)#3. */
    {
        const double h_climb = node->get_parameter("height_rate_max_climb").as_double();
        const double h_sink  = std::abs(node->get_parameter("height_rate_min_sink").as_double());
        pp.h_dot_max = std::min(h_climb, h_sink);
    }
    pp.a_lat_max   = limits.gravity * std::tan(limits.max_roll_rad);
    pp.V_h_min     = 1.0;

    auto predictor = std::make_shared<TrajectoryPredict>(pp);

    UncertaintyParams uncertainty_params;
    uncertainty_params.process_noise_diagonal = {
        node->get_parameter("uncertainty.q_pn").as_double(),
        node->get_parameter("uncertainty.q_pe").as_double(),
        node->get_parameter("uncertainty.q_h").as_double(),
        node->get_parameter("uncertainty.q_v").as_double(),
        node->get_parameter("uncertainty.q_psi").as_double(),
        node->get_parameter("uncertainty.q_hdot").as_double(),
        node->get_parameter("uncertainty.q_phi").as_double()};
    auto cone_publisher = std::make_shared<BeliefConePublisher>(
        *node, topic_ns, predictor, uncertainty_params);

    /* 호출자 멤버 array — 매 predict 호출마다 in-place 덮어쓰기 (heap 0).
       shared_ptr 로 감싸 lambda 캡처 lifetime 보장 (노드 종료까지 살아있음). */
    auto predicted_traj =
        std::make_shared<TrajectoryLogger::PredictedTrajectory>();
    auto prediction_inputs = std::make_shared<PredictionInputTrajectory>();

    const double dt = 1.0 / predict_rate;
    RCLCPP_INFO(node->get_logger(),
        "[main] Predictor: endpoint=%.2fs rate=%.1fHz call=%.1fHz "
        "(N=%zu dt=%.3fs tau_V=%.2f tau_hdot=%.2f tau_phi=%.2f b_h=%.3f)",
        predict_horizon_endpoint, predict_rate, predict_call,
        TrajectoryLogger::kPredictHorizon, dt,
        pp.tau_V, pp.tau_hdot, pp.tau_phi, pp.b_h);
    if (pp.b_h > 0.0) {
        RCLCPP_INFO(node->get_logger(),
            "[main] 종 채널 PD 모드 활성 (b_h=%.3f, alt_offset=%+.2fm). "
            "h_cmd = baseline_alt + alt_offset 고정.", pp.b_h, alt_offset);
    } else {
        RCLCPP_INFO(node->get_logger(),
            "[main] 종 채널 1차 지연 모드 (b_h=0). h_cmd 무시 (NaN guard 가 자동 fallback). "
            "alt_offset=%+.2fm 은 PD 모드 켜질 때만 효과.", alt_offset);
    }

    auto preflight = std::make_shared<VtolPreflightMode>(*node, vehicle_ID);
    auto replay    = std::make_shared<TrajectoryReplayMode>(
        *node, vehicle_ID, std::move(sequencer), logger);

    /* [4] Executor 생성 (owned_mode = preflight, second_mode = replay) */
    auto executor = std::make_shared<TrajectoryReplayExecutor>(*preflight, *replay);

    /* [4.5] Predict wall_timer (predict_call_hz) 등록 — main thread executor 가 호출.
            매 호출마다 6-state 측정 + setpoint → 7-state PredictState + PredictInput
            변환 → predict() → logger 에 결과 push.
            CSV 의 다음 onTick (50Hz) 에 자연스럽게 같이 기록됨. */
    const auto predict_period = std::chrono::milliseconds(
        static_cast<int>(1000.0 / predict_call));
    /* ★ 2026-05-13: collision_estimation key_samples publisher 등록.
       extractKeySamples 의 결과 (4 시점 × {pos, vel} = 6 Vec3 = 18 float) 를
       std_msgs/Float32MultiArray 로 publish. 멀티 에이전트 통신의 1차 prototype.

       ★ float 사용 이유: Vec3 가 float (PX4 측정값과 일관, 대역폭 50% 절약).
         18 × 4 = 72 byte/cycle, 10 Hz × 5 agents × 5 neighbors = 5+ 대 swarm 시
         실용 대역폭.

       Layout (data 배열의 18 float):
         [0..2]   pos_t0  (NED north, east, down) [m]
         [3..5]   vel_t0  (NED vn, ve, vd)         [m/s]
         [6..8]   pos_t15 (NED)                    [m]
         [9..11]  pos_t30 (NED)                    [m]
         [12..14] pos_t45 (NED)                    [m]
         [15..17] vel_t45 (NED)                    [m/s]

       토픽 이름: "<topic_ns>/collision_estimation/key_samples"
         single-vehicle: "/collision_estimation/key_samples"
         multi-vehicle:  "/px4_<id>/collision_estimation/key_samples"

       QoS: SensorDataQoS (best-effort, depth 5) — latency 우선.
       향후: custom msg (PredictionSamples.msg) 로 정밀화 가능. */
    auto key_samples_pub = node->create_publisher<std_msgs::msg::Float32MultiArray>(
        topic_ns + "/collision_estimation/key_samples",
        rclcpp::SensorDataQoS());
    RCLCPP_INFO(node->get_logger(),
        "[main] key_samples publisher: '%s' (Float32MultiArray, 18 float, 10Hz)",
        (topic_ns + "/collision_estimation/key_samples").c_str());

    auto predict_timer = node->create_wall_timer(
        predict_period,
        [predictor, logger, predicted_traj, prediction_inputs,
         cone_publisher, dt, alt_offset, key_samples_pub]() {
            const auto m  = logger->getCurrentMeasurements();
            const auto sp = logger->getCurrentSetpointSnapshot();

            /* fallback 직전 (sp.is_fallback=1) ~10ms 동안 logger 의 sp 슬롯에
               sp.V/h_dot/a_lat = NaN 이 들어와 있음. 그대로 적분하면
               atan2(NaN, g)=NaN → stepRK4 첫 stage 부터 phi=NaN → trajectory 전체 발산
               (chunk 0 의 XY end 83m / psi 147° outlier 의 직접 원인).
               NaN 인 경우에만 안전 default 로 치환. CSV 는 logger 가 진실 그대로 기록. */
            const float a_lat_safe = std::isfinite(sp.a_lat) ? sp.a_lat : 0.0f;
            const float V_safe     = std::isfinite(sp.V)
                ? sp.V : static_cast<float>(m.true_airspeed);
            const float h_dot_safe = std::isfinite(sp.h_dot) ? sp.h_dot : 0.0f;

            /* 6-state 측정 → 7-state PredictState 변환.
               ★ A-1 (claude_code_task_spec.md): phi 의 초기값을 setpoint 등가 (atan2)
                  대신 vehicle_attitude → 실측 roll 사용. Beard-McLain 7-state 명세 정합.
                  (이전 atan2(a_lat_safe, k_g) 은 *명령* 이었지 *자세* 아음 → 부정합)
               ★ 2026-05-13 course-angle 패치: x0.psi = body yaw (m.yaw) → course angle
                  (atan2(m.ve, m.vn)). dynamics 식 ψ̇ = (g/V)·tan(φ) 는 coordinated turn
                  가정 (β̇≈0) 아래에서 body yaw rate = course angle rate 라 *동일 식*.
                  모델 유도 변경 없이 *interpretation* 만 변경. trajectory prediction 의 목적
                  ("4.5 s 후 *어디로 갈지*") 에 자연스러운 정의. body yaw 와 ground track 의
                  3.28° crab angle (R15P 측정치) artifact → t=0 velocity 벡터 오차 1.15 m/s
                  → 4.5s XY drift 5.17 m 의 주성분 제거 (xy_end 6.0 → ≈ 1 m 기대). */
            /* ★ 2026-05-13 ground-velocity 일관화 패치:
               x0 의 *속도 관련* 3 성분 (V, psi, h_dot) 모두 *같은 EKF snapshot*
               (vehicle_local_position 의 vn/ve/vd) 에서 derive.
                 V    = √(vn² + ve² + vd²)        ← speed magnitude
                 psi  = atan2(ve, vn)              ← course angle (수평 방향)
                 hdot = -vd                        ← climb rate
               이전 V = m.true_airspeed (airspeed_validated 토픽) 는 *다른 EKF 출력*
               이라 vn/ve/vd 와 시점·정밀도 inconsistency (0.03 m/s) 발생.
               7-state kinematic 모델은 외란 (풍) 을 *명시적으로 무시* 하므로
               V_air ≡ V_g 가정이 모델 일관성에 정합. trajectory_error = (모델 오차)
               + (외란) 의 *깨끗한 decomposition* 을 위한 *EKF principle* — *x0 = 추정값*. */
            const double V_mag = std::sqrt(
                static_cast<double>(m.vn) * m.vn +
                static_cast<double>(m.ve) * m.ve +
                static_cast<double>(m.vd) * m.vd);
            PredictState x0;
            x0.p_n   = m.x;
            x0.p_e   = m.y;
            x0.h     = -m.z;                                /* NED z(down) → altitude(up) */
            x0.V     = static_cast<float>(V_mag);           /* ★ ground-speed magnitude (EKF v 벡터 크기) */
            x0.psi   = std::atan2(m.ve, m.vn);              /* ★ course angle (NED ground velocity 방향) */
            x0.h_dot = -m.vd;                               /* NED vd(down) → climb-rate(up) */
            x0.phi   = m.roll;                              /* ★ A-1: 실측 roll (quaternion → roll) */

            /* Replay 활성 여부와 현재 적용 입력을 한 snapshot 으로 판정한다.
               미래 YAML schedule 은 predictor 가 읽지 않는다. 매 계산 프레임에서
               현재 선택된 후보 입력 하나를 4.5초 동안 ZOH 한 뒤 다음 프레임에서
               새 상태/입력으로 다시 계산한다. */
            const double t_now = logger->getReplayTime();

            /* baseline altitude — Replay 가 onActivate 시점 alt 캡처해서 logger 에 push.
               NaN 이면 stepRK4 의 h_cmd NaN guard 가 x.h 로 fallback. */
            const double baseline_alt = logger->getBaselineAlt();

            const bool replay_active = std::isfinite(t_now)
                && !sp.is_fallback
                && std::isfinite(sp.V)
                && std::isfinite(sp.h_dot)
                && std::isfinite(sp.a_lat);
            const double h_cmd = replay_active
                ? (std::isfinite(baseline_alt) ? baseline_alt + alt_offset : x0.h)
                : std::nan("");
            const PredictInput u_zoh{V_safe, h_cmd, h_dot_safe, a_lat_safe};
            prediction_inputs->fill(u_zoh);
            predictor->predict<TrajectoryLogger::kPredictHorizon>(
                x0, u_zoh, dt, *predicted_traj);
            /* ★ Option A (2026-05-13): traj + m 을 *한 호출* 로 logger 에 publish.
               m 은 위 (~200) 에서 이미 읽혀 x0 를 만드는 데 쓰였음 — 동일 변수를
               그대로 넘김 → "예측 시점의 측정 = m_at_pred = p_*_0 의 입력" 이라는
               물리적 invariant 가 publish 단계에서 보존. 향후 RT thread 이전 시
               이 호출부는 변경 없음 (mutex → SPSC 는 logger 내부만 바꾸면 됨). */
            logger->publishPredictBundle(*predicted_traj, m);
            if (replay_active) {
                cone_publisher->publish(*prediction_inputs, dt);
            }

            /* ★ 2026-05-13 key_samples 추출 + publish + 테스트 출력.
               좌표계: NED (extractKeySamples 가 입력 PX4 NED 와 정확히 일치하는
               좌표로 추출 — TrajectoryPredict.cpp 의 extractAt 좌표 변환 참조). */
            const TrajectorySample s =
                predictor->extractKeySamples(*predicted_traj);

            /* (a) ROS2 publish — 4 시점 × {pos, vel} = 18 float broadcast.
               멀티 에이전트 통신의 1차 prototype. */
            std_msgs::msg::Float32MultiArray ks_msg;
            ks_msg.data.resize(18);
            ks_msg.data[0]  = s.pos_t0.x;   ks_msg.data[1]  = s.pos_t0.y;   ks_msg.data[2]  = s.pos_t0.z;
            ks_msg.data[3]  = s.vel_t0.x;   ks_msg.data[4]  = s.vel_t0.y;   ks_msg.data[5]  = s.vel_t0.z;
            ks_msg.data[6]  = s.pos_t15.x;  ks_msg.data[7]  = s.pos_t15.y;  ks_msg.data[8]  = s.pos_t15.z;
            ks_msg.data[9]  = s.pos_t30.x;  ks_msg.data[10] = s.pos_t30.y;  ks_msg.data[11] = s.pos_t30.z;
            ks_msg.data[12] = s.pos_t45.x;  ks_msg.data[13] = s.pos_t45.y;  ks_msg.data[14] = s.pos_t45.z;
            ks_msg.data[15] = s.vel_t45.x;  ks_msg.data[16] = s.vel_t45.y;  ks_msg.data[17] = s.vel_t45.z;
            key_samples_pub->publish(ks_msg);

            /* (b) ★ 테스트용: ReconstructTrajectory 의 함수로 *수신측 처리 prototype*.
               - validateKeySamples: NaN 가드 (publish 직전 sanity check)
               - debugPrintKeySamples: 5 초 (50 cycle) 마다 콘솔 출력 — throttled */
            namespace ce = collision_avoidance::estimation;
            static std::size_t s_cycle_count = 0;
            if (!ce::validateKeySamples(s)) {
                /* NaN 또는 Inf 감지 — *조용히 skip* 후 다음 cycle (RCLCPP_WARN 안 함:
                   Replay 시작 직전엔 trajectory 가 NaN 으로 채워져 정상). */
            } else if (s_cycle_count % 50 == 0) {
                const double d_max = ce::computeMaxHorizontalDisplacement(s);
                std::cout << "─── [predict_timer cycle " << s_cycle_count
                          << "] horizontal look-ahead = " << d_max << " m ───\n";
                ce::debugPrintKeySamples(s, std::cout);
            }
            ++s_cycle_count;
        });

    /* [5] PX4 에 등록 — Executor + Replay 둘 다 doRegister() 필요 */
    RCLCPP_INFO(node->get_logger(),
        "[main] doRegister() 시작 (vehicle_ID=%d)", vehicle_ID);

    if (!executor->doRegister()) {
        RCLCPP_ERROR(node->get_logger(), "[main] Executor doRegister() 실패");
        throw std::runtime_error("Executor registration failed");
    }
    if (!replay->doRegister()) {
        RCLCPP_ERROR(node->get_logger(), "[main] TrajectoryReplayMode doRegister() 실패");
        throw std::runtime_error("ReplayMode registration failed");
    }

    RCLCPP_INFO(node->get_logger(), "[main] doRegister() 성공 → spin 시작");

    /* [6] 실행 */
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
