/* ════════════════════════════════════════════════════════════════════
   TrajectoryLogger.cpp — TrajectoryLogger.hpp 구현부
   ────────────────────────────────────────────────────────────────────
   하는 일:
     [생성자]
       - CSV 파일 오픈: <prefix>_YYYYMMDD_HHMMSS.csv
         · 라인 버퍼링 (_IOLBF) → crash 시에도 부분 회수 가능
         · 헤더 1회 fprintf  (고정 13 컬럼 + 7 × kPredictHorizon 컬럼)
       - 3개 PX4 토픽 구독 (sensor_data QoS, main thread 콜백):
           /px4_<id>/fmu/out/vehicle_local_position  → x,y,z,vn,ve,vd
           /px4_<id>/fmu/out/vehicle_attitude        → quaternion → yaw
           /px4_<id>/fmu/out/airspeed_validated      → true_airspeed
         콜백은 atomic 슬롯에 store 만 (lock-free)
       - 별도 wall_timer 50Hz 등록 → onTick() 호출

     [pushAppliedSetpoint()]
       Replay 가 매 publish 직후 호출. 보낸 setpoint 를 atomic 으로 저장.

     [publishPredictBundle()]
       main.cpp wall_timer (10Hz) 가 predict 직후 호출.
       (traj, m_at_pred) 쌍을 한 mutex 안에서 한꺼번에 갱신 — 두 데이터가 *같은
        predict() 호출에서 derive 됐다* 는 invariant 를 publish 단계에서 보장.

     [onTick()]
       50Hz Timer 콜백. atomic snapshot load → fprintf 로 한 줄 기록.
       publish 와 분리된 주기 → fwrite 지연이 setpoint jitter 로 안 번짐.

     [enable/disable]
       Replay onActivate/onDeactivate 가 호출 → CSV 기록 ON/OFF.

   CSV 컬럼 (md_file/TASK_trajectory_predictor.md 7-state + Option A m_*_atp 반영):
     t_us, sp_v, sp_h, sp_a, sp_fallback,
     x, y, z, vn, ve, vd, yaw, roll, true_airspeed,
     m_x_atp, m_y_atp, m_z_atp, m_vn_atp, m_ve_atp, m_vd_atp,
     m_yaw_atp, m_roll_atp, m_V_atp,
     p_pn_k, p_pe_k, p_h_k, p_V_k, p_psi_k, p_hdot_k, p_phi_k  (k = 0..N-1)
   ════════════════════════════════════════════════════════════════════ */

#include <trajectory_prediction_hils/logging/TrajectoryLogger.hpp>

#include <chrono>
#include <ctime>
#include <cmath>
#include <limits>     /* std::numeric_limits<float>::quiet_NaN */
#include <stdexcept>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using collision_avoidance::estimation::PredictState;
using collision_avoidance::estimation::PoseVel;
using collision_avoidance::estimation::TrajectorySample;

namespace {
inline float quatToYaw(float w, float x, float y, float z) {
    /* ZYX yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2)) */
    return std::atan2(2.f * (w * z + x * y), 1.f - 2.f * (y * y + z * z));
}
inline float quatToRoll(float w, float x, float y, float z) {
    /* ZYX roll = atan2(2(wx + yz), 1 - 2(x^2 + y^2))
       ★ A-1: 7-state Beard-McLain φ 상태의 실측 초기값 추출. */
    const float sinr_cosp = 2.f * (w * x + y * z);
    const float cosr_cosp = 1.f - 2.f * (x * x + y * y);
    return std::atan2(sinr_cosp, cosr_cosp);
}
}  /* anonymous namespace */


TrajectoryLogger::TrajectoryLogger(rclcpp::Node & node,
                                   int vehicle_id,
                                   const std::string & csv_path_prefix,
                                   double log_rate_hz,
                                   const std::string & topic_namespace_prefix)
: _node(node), m_vehicle_id(vehicle_id)
{
    /* CSV 파일 오픈 (라인 버퍼링 → crash 시 부분 회수) */
    m_filename = makeFilename(csv_path_prefix);
    _fp = std::fopen(m_filename.c_str(), "w");
    if (!_fp) {
        throw std::runtime_error("[Logger] CSV 파일 열기 실패: " + m_filename);
    }
    std::setvbuf(_fp, nullptr, _IOLBF, 4096);

    /* CSV 헤더 — 고정 14 컬럼 + 측정@predict 9 컬럼 + 예측 7 × kPredictHorizon 컬럼.
       ★ A-1: roll 컬럼 추가 (yaw 옆). 13 → 14 컬럼.
       ★ 2026-05-13 Option A: m_*_atp 9 컬럼 추가. 14 → 23 컬럼.
         *_atp = "measurement at predict-time" — predict_timer 호출 시점에
         publishPredictBundle() 로 함께 freeze 된 측정 snapshot.
         같은 행의 p_*_0 와 *수학적으로 동일* — t=0 오차 구조적 0 보장. */
    std::fprintf(_fp,
        "t_us,sp_v,sp_h,sp_a,sp_fallback,"
        "x,y,z,vn,ve,vd,yaw,roll,true_airspeed,"
        "m_x_atp,m_y_atp,m_z_atp,m_vn_atp,m_ve_atp,m_vd_atp,"
        "m_yaw_atp,m_roll_atp,m_V_atp");
    for (std::size_t k = 0; k < kPredictHorizon; ++k) {
        std::fprintf(_fp,
            ",p_pn_%zu,p_pe_%zu,p_h_%zu,p_V_%zu,p_psi_%zu,p_hdot_%zu,p_phi_%zu",
            k, k, k, k, k, k, k);
    }
    std::fprintf(_fp, "\n");

    /* ★ 2026-05-20 — spline CSV 파일 오픈 + 헤더 write.
       predict CSV 와 동일 timestamp + suffix "_spline". 호출자 (main.cpp) 변경 0. */
    m_filename_spline = makeFilenameSpline(csv_path_prefix);
    _fp_spline = std::fopen(m_filename_spline.c_str(), "w");
    if (!_fp_spline) {
        throw std::runtime_error("[Logger] spline CSV 파일 열기 실패: " + m_filename_spline);
    }
    std::setvbuf(_fp_spline, nullptr, _IOLBF, 4096);

    /* spline CSV 헤더: t_us + 46점 × (pn_spline, pe_spline, pd_spline, vn_spline, ve_spline, vd_spline) = 277 컬럼.
       predict CSV 의 t_us 와 *완전 동일 값* 보장 (옵션 C tick-aligned) → python 측에서
       df_pred.merge(df_spline, on='t_us') 한 줄로 결합. */
    std::fprintf(_fp_spline, "t_us");
    for (std::size_t k = 0; k < kPredictHorizon; ++k) {
        std::fprintf(_fp_spline,
            ",k%zu_pn_spline,k%zu_pe_spline,k%zu_pd_spline,k%zu_vn_spline,k%zu_ve_spline,k%zu_vd_spline",
            k, k, k, k, k, k);
    }
    std::fprintf(_fp_spline, "\n");

    /* m_pred_bundle_mt 초기화 — predict 한 번도 호출 전이면 NaN.
       valid=false 표식과 함께 traj + m_at_pred 둘 다 NaN 으로 채움
       (CSV 컬럼 수 일관성 — pandas 가 빈 셀 만나면 dtype 추론 깨짐). */
    const PredictState nan_state{
        std::nan(""), std::nan(""), std::nan(""), std::nan(""),
        std::nan(""), std::nan(""), std::nan("")
    };
    m_pred_bundle_mt.traj.fill(nan_state);
    m_pred_bundle_mt.m_at_pred = MeasuredSnapshot{
        NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN
    };
    m_pred_bundle_mt.valid = false;

    /* ★ 2026-05-20 — spline array 도 NaN 초기화 (publishPredictBundle 호출 전 dump 시 NaN row). */
    const PoseVel nan_pose_vel{
        Eigen::Vector3f::Constant(std::numeric_limits<float>::quiet_NaN()),
        Eigen::Vector3f::Constant(std::numeric_limits<float>::quiet_NaN())
    };
    m_pred_bundle_mt.spline.fill(nan_pose_vel);

    RCLCPP_INFO(_node.get_logger(),
        "[Logger] CSV 출력: %s (rate=%.1f Hz, predict_horizon=%zu)",
        m_filename.c_str(), log_rate_hz, kPredictHorizon);

    /* PX4 best-effort QoS */
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    /* PX4 v1.17 부터 일부 메시지가 message versioning (_v1) 적용됨.
       vehicle_local_position, airspeed_validated → _v1 / vehicle_attitude → 그대로.
       namespace prefix 는 파라미터 (single-vehicle: "", multi: "/px4_<id>"). */
    const std::string & ns = topic_namespace_prefix;
    RCLCPP_INFO(_node.get_logger(),
        "[Logger] topic namespace prefix: '%s' (vehicle_id=%d)",
        ns.c_str(), m_vehicle_id);

    _local_pos_sub = _node.create_subscription<VehicleLocalPosition>(
        ns + "/fmu/out/vehicle_local_position_v1", qos,
        [this](const VehicleLocalPosition::UniquePtr msg) {
            m_x.store(msg->x,  std::memory_order_relaxed);
            m_y.store(msg->y,  std::memory_order_relaxed);
            m_z.store(msg->z,  std::memory_order_relaxed);
            m_vn.store(msg->vx, std::memory_order_relaxed);
            m_ve.store(msg->vy, std::memory_order_relaxed);
            m_vd.store(msg->vz, std::memory_order_relaxed);
        });

    _attitude_sub = _node.create_subscription<VehicleAttitude>(
        ns + "/fmu/out/vehicle_attitude", qos,
        [this](const VehicleAttitude::UniquePtr msg) {
            const float yaw  = quatToYaw (msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
            const float roll = quatToRoll(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
            m_yaw .store(yaw,  std::memory_order_relaxed);
            m_roll.store(roll, std::memory_order_relaxed);
        });

    _airspeed_sub = _node.create_subscription<AirspeedValidated>(
        ns + "/fmu/out/airspeed_validated_v1", qos,
        [this](const AirspeedValidated::UniquePtr msg) {
            m_true_airspeed.store(msg->true_airspeed_m_s, std::memory_order_relaxed);
        });

    /* 50Hz 별도 timer — main executor 에서 호출되므로 lock-free atomic load 만 */
    const auto period_ms = std::chrono::milliseconds(
        static_cast<int>(1000.0 / log_rate_hz));
    _tick_timer = _node.create_wall_timer(
        period_ms, [this]() { onTick(); });
}

TrajectoryLogger::~TrajectoryLogger()
{
    if (_fp) {
        std::fflush(_fp);
        std::fclose(_fp);
        _fp = nullptr;
    }
    /* ★ 2026-05-20 — spline CSV 도 close. */
    if (_fp_spline) {
        std::fflush(_fp_spline);
        std::fclose(_fp_spline);
        _fp_spline = nullptr;
    }
}

void TrajectoryLogger::pushAppliedSetpoint(float V, float h_dot, float a_lat, bool is_fallback)
{
    m_sp_v.store(V,        std::memory_order_relaxed);
    m_sp_h.store(h_dot,    std::memory_order_relaxed);
    m_sp_a.store(a_lat,    std::memory_order_relaxed);
    m_sp_fallback.store(is_fallback, std::memory_order_relaxed);
}

void TrajectoryLogger::publishPredictBundle(
    const PredictedTrajectory & traj,
    const MeasuredSnapshot &    m_at_pred)
{
    /* ★ 2026-05-20 — spline 계수 산출 + 46점 평가 (mutex 밖, 호출자 thread).
       reconstruct 자체는 *전역 state 없음 + 멤버 함수 read-only* 가 아니라
       *m_reconstructor_mt 의 detail::segments 갱신* 이 있으므로, *같은 thread*
       (main executor) 가 호출함을 *전제*. publishPredictBundle 자체가 main thread 단일. */
    TrajectorySample sample;
    /* 4 시점 인덱스: 0, 15, 30, 45 (PredictTypes.hpp 의 kSampleIdx_t0/t15/t30/t45). */
    auto state_to_ned = [](const PredictState & s) -> collision_avoidance::estimation::Vec3 {
        /* NED 변환: extractAt 와 동일 — pd = -h. 단 본 메서드는 위치만 추출. */
        return { static_cast<float>(s.p_n), static_cast<float>(s.p_e), static_cast<float>(-s.h) };
    };
    auto state_to_vel_ned = [](const PredictState & s) -> collision_avoidance::estimation::Vec3 {
        /* Beard-McLain coordinate transform — γ = asin(h_dot / V).
           V > 0 가정 (FW cruise). vn = V cosγ cosψ, ve = V cosγ sinψ, vd = -h_dot. */
        const double V         = s.V;
        const double psi       = s.psi;
        const double h_dot     = s.h_dot;
        const double sin_gamma = (V > 1e-3) ? (h_dot / V) : 0.0;
        const double cos_gamma = std::sqrt(std::max(0.0, 1.0 - sin_gamma * sin_gamma));
        return {
            static_cast<float>(V * cos_gamma * std::cos(psi)),
            static_cast<float>(V * cos_gamma * std::sin(psi)),
            static_cast<float>(-h_dot)
        };
    };
    sample.pos_t0  = state_to_ned(traj[0]);
    sample.vel_t0  = state_to_vel_ned(traj[0]);
    sample.pos_t15 = state_to_ned(traj[15]);
    sample.pos_t30 = state_to_ned(traj[30]);
    sample.pos_t45 = state_to_ned(traj[45]);
    sample.vel_t45 = state_to_vel_ned(traj[45]);

    /* spline 계수 산출 (m_reconstructor_mt 의 detail::segments 갱신). */
    m_reconstructor_mt.calculate_clamp_cubic_spline(sample);

    /* 46점 평가 — t = 0.0, 0.1, ..., 4.5 */
    SplineTrajectory spline_local;
    for (std::size_t k = 0; k < kPredictHorizon; ++k) {
        const float t = static_cast<float>(k) * 0.1f;
        spline_local[k] = m_reconstructor_mt.reconstruct(t);
    }

    /* mutex 안에서 traj + m_at_pred + spline 셋 다 *원자적으로* 갱신.
       세 데이터를 한 단위로 publish 해서 onTick 이 *부분적으로 새 traj +
       옛 m_at_pred + 옛 spline* 같은 frankenstein 행을 못 만들게 강제. */
    std::lock_guard<std::mutex> lock(m_predict_mutex_mt);
    m_pred_bundle_mt.traj      = traj;
    m_pred_bundle_mt.m_at_pred = m_at_pred;
    m_pred_bundle_mt.spline    = spline_local;
    m_pred_bundle_mt.valid     = true;
}

TrajectoryLogger::MeasuredSnapshot TrajectoryLogger::getCurrentMeasurements() const
{
    return MeasuredSnapshot{
        m_x.load(std::memory_order_relaxed),
        m_y.load(std::memory_order_relaxed),
        m_z.load(std::memory_order_relaxed),
        m_vn.load(std::memory_order_relaxed),
        m_ve.load(std::memory_order_relaxed),
        m_vd.load(std::memory_order_relaxed),
        m_yaw.load(std::memory_order_relaxed),
        m_roll.load(std::memory_order_relaxed),    /* ★ A-1 */
        m_true_airspeed.load(std::memory_order_relaxed),
    };
}

TrajectoryLogger::SetpointSnapshot TrajectoryLogger::getCurrentSetpointSnapshot() const
{
    return SetpointSnapshot{
        m_sp_v.load(std::memory_order_relaxed),
        m_sp_h.load(std::memory_order_relaxed),
        m_sp_a.load(std::memory_order_relaxed),
        m_sp_fallback.load(std::memory_order_relaxed),
    };
}

void TrajectoryLogger::setReplayTime(double t_relative)
{
    m_replay_t_relative.store(t_relative, std::memory_order_relaxed);
}

double TrajectoryLogger::getReplayTime() const
{
    return m_replay_t_relative.load(std::memory_order_relaxed);
}

/* ★ 신규 — baseline altitude (Replay::onActivate 시점의 cruise alt). */
void TrajectoryLogger::setBaselineAlt(double alt)
{
    m_baseline_alt_mt2rt.store(alt, std::memory_order_relaxed);
}

double TrajectoryLogger::getBaselineAlt() const
{
    return m_baseline_alt_mt2rt.load(std::memory_order_relaxed);
}

void TrajectoryLogger::onTick()
{
    if (!m_enabled.load(std::memory_order_relaxed) || !_fp) return;

    const uint64_t t_us =
        static_cast<uint64_t>(_node.get_clock()->now().nanoseconds() / 1000);

    /* (1) 고정 14 컬럼 (★ A-1: roll 추가) */
    std::fprintf(_fp,
        "%lu,"
        "%.4f,%.4f,%.4f,%d,"
        "%.4f,%.4f,%.4f,"
        "%.4f,%.4f,%.4f,"
        "%.5f,%.5f,%.4f",
        static_cast<unsigned long>(t_us),
        m_sp_v.load(std::memory_order_relaxed),
        m_sp_h.load(std::memory_order_relaxed),
        m_sp_a.load(std::memory_order_relaxed),
        m_sp_fallback.load(std::memory_order_relaxed) ? 1 : 0,
        m_x.load(std::memory_order_relaxed),
        m_y.load(std::memory_order_relaxed),
        m_z.load(std::memory_order_relaxed),
        m_vn.load(std::memory_order_relaxed),
        m_ve.load(std::memory_order_relaxed),
        m_vd.load(std::memory_order_relaxed),
        m_yaw.load(std::memory_order_relaxed),
        m_roll.load(std::memory_order_relaxed),
        m_true_airspeed.load(std::memory_order_relaxed));

    /* (2) PredictBundle local copy — mutex 짧게.
       traj 와 m_at_pred 가 *같은 publish 에서 derive 된 한 쌍* 이라는 invariant
       가 보존됨. fprintf 는 mutex 밖 — 향후 RT thread 분리 + SPSC queue 치환 시
       이 구조가 그대로 살아남도록 의도적으로 짠 패턴. */
    PredictBundle local_bundle;
    {
        std::lock_guard<std::mutex> lock(m_predict_mutex_mt);
        local_bundle = m_pred_bundle_mt;
    }

    /* (2a) 측정 @ predict 시점 — 9 컬럼.
       *_atp 컬럼은 p_*_0 와 *수학적으로 동일* (같은 m 에서 파생).
       compare_single_input.py 가 이 컬럼을 측정값으로 사용 → t=0 오차 ≡ 0. */
    const MeasuredSnapshot & map = local_bundle.m_at_pred;
    std::fprintf(_fp,
        ",%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.5f,%.5f,%.4f",
        map.x, map.y, map.z, map.vn, map.ve, map.vd,
        map.yaw, map.roll, map.true_airspeed);

    /* (2b) 예측 7 × kPredictHorizon 컬럼.
       valid==false (predict 한 번도 호출 전) 인 경우도 NaN 채워진 buf 그대로 출력
       (CSV 컬럼 수 일관성). */
    for (std::size_t k = 0; k < kPredictHorizon; ++k) {
        const PredictState & s = local_bundle.traj[k];
        std::fprintf(_fp,
            ",%.4f,%.4f,%.4f,%.4f,%.5f,%.4f,%.5f",
            s.p_n, s.p_e, s.h, s.V, s.psi, s.h_dot, s.phi);   /* ★ PATCH: a_lat → phi */
    }
    (void)local_bundle.valid;   /* 현재는 NaN 출력으로 일관 — valid 플래그는 향후 분기용 reserve */

    std::fprintf(_fp, "\n");

    /* ★ 2026-05-20 — spline CSV 한 줄 dump (옵션 C tick-aligned: 동일 t_us).
       46점 × (pn, pe, pd, vn, ve, vd) = 276 컬럼 + t_us = 277 컬럼.
       NaN 도 그대로 fprintf — pandas 가 자동 NaN 인식. */
    if (_fp_spline) {
        std::fprintf(_fp_spline, "%lu", static_cast<unsigned long>(t_us));
        for (std::size_t k = 0; k < kPredictHorizon; ++k) {
            const PoseVel & pv = local_bundle.spline[k];
            std::fprintf(_fp_spline,
                ",%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
                pv.pos.x(), pv.pos.y(), pv.pos.z(),
                pv.vel.x(), pv.vel.y(), pv.vel.z());
        }
        std::fprintf(_fp_spline, "\n");
    }
}

std::string TrajectoryLogger::makeFilename(const std::string & prefix)
{
    const auto now  = std::chrono::system_clock::now();
    const auto t_t  = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf;
    localtime_r(&t_t, &tm_buf);
    char stamp[32];
    std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &tm_buf);
    return prefix + "_" + std::string(stamp) + ".csv";
}

/* ★ 2026-05-20 — spline CSV 파일명: prefix + "_spline" + 동일 timestamp.
   원본 makeFilename 의 prefix 구조 (예: "results/cases/R15P") 끝에 "_spline" 붙임.
   결과 예: "results/cases/R15P_spline_20260520_103045.csv". */
std::string TrajectoryLogger::makeFilenameSpline(const std::string & prefix)
{
    return makeFilename(prefix + "_spline");
}
