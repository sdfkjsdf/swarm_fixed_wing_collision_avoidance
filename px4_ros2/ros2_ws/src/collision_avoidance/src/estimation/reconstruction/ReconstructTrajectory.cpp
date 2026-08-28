/* ════════════════════════════════════════════════════════════════════
   ReconstructTrajectory.cpp — extractKeySamples 결과 *수신/사용* 측 함수

   2026-05-13 신설 (테스트용):
     TrajectoryPredict::extractKeySamples 가 추출한 TrajectorySample 을
     *받아서* 사용하는 함수들. 미래에 멀티 에이전트 통신 시나리오에서
     *Agent 2 의 수신측 처리* 의 prototype 으로 사용 예정.

   현재 (Phase 1, 테스트용):
     - debugPrintKeySamples: TrajectorySample 의 6 Vec3 값을 콘솔에 출력
     - validateKeySamples: TrajectorySample 의 NaN / 비정상 값 검출 (sanity check)

   2026-05-20 추가 (clamped cubic spline 재구성):
     - calculate_clamp_cubic_spline: 4 시점 sample → 3 구간 spline 계수
       (segments[3] 의 P, b, c, d, t_start) 산출
     - reconstruct(t): 임의 시각 t ∈ [0.0, 4.5] 에서 위치 + 속도 평가 (PoseVel 반환)

   향후 (멀티 에이전트 통신 단계):
     - deserializeFromMsg(Float32MultiArray) → TrajectorySample
     - checkCollisionAgainstSamples(my_state, neighbor_samples) → bool
     - 등 *수신측 처리* 함수 추가

   설계 원칙:
     - heap 0 / RT-safe: 모든 변수가 *정적 크기* (Eigen::Vector3f / Matrix4f /
       Matrix<float,4,3> / std::array<CubicSegment, 3>) → stack 또는 BSS 할당.
       runtime new/malloc 0.
     - Eigen *header-only* 의존: Vector3f / Matrix4f / Matrix<float,4,3> /
       Vector3f::Constant() 사용. header-only 라 *runtime link 비용 0*,
       *컴파일 시 정적 통합* (expression template + SIMD 자동 활용).
     - rclcpp / px4_msgs 사용 안 함: ROS2 통신 layer 와 *분리* — 본 파일은
       *순수 수신측 처리 알고리즘* 만 담당. ROS2 publish/subscribe 는
       trajectory_prediction 패키지의 main.cpp 가 담당.
   ════════════════════════════════════════════════════════════════════ */

#include <collision_avoidance/estimation/reconstruction/ReconstructTrajectory.hpp>
#include <collision_avoidance/estimation/trajectory_prediction/PredictTypes.hpp>

#include <cmath>      /* std::isfinite */
#include <iostream>   /* std::ostream, std::cout */
#include <iomanip>    /* std::fixed, std::setprecision */
#include <limits>     /* std::numeric_limits<float>::quiet_NaN */


namespace collision_avoidance::estimation
{

/* ─────────────────────────────────────────────────────────────────
   ReconstructTrajectory::calculate_clamp_cubic_spline
   ─────────────────────────────────────────────────────────────────
   Clamped cubic spline 의 b/c/d 계수 계산.
   사용자 명시 (2026-05-13) 사양대로 구현:

   1. set_sample_point(s) → instance sample storage 채움
   2. 4×3 행렬 B, C 를 Zero 로 초기화
   3. B 행 채우기 (각 행 = (1×3) row vector, 시점별 우변 항):
        B.row(0) = ((3/h)·(P2-P1) - 3·V1)^T
        B.row(1) = ((3/h)·(P3-P2) - (3/h)·(P2-P1))^T
        B.row(2) = ((3/h)·(P4-P3) - (3/h)·(P3-P2))^T
        B.row(3) = (3·V4 - (3/h)·(P4-P3))^T
   4. C = inverse_A · B  (4×4 · 4×3 = 4×3)
   5. c1..c4 = C.row(i).transpose()  (각 행을 Vector3f 로)
   6. b1..b3 = (P_{i+1}-P_i)/h - (h/3)·(2·c_i + c_{i+1})
   7. d1..d3 = (c_{i+1} - c_i) / (3·h)
   ───────────────────────────────────────────────────────────────── */
void ReconstructTrajectory::calculate_clamp_cubic_spline(const TrajectorySample & s)
{
    /* (1) set 함수를 통해서 instance sample storage 에 값 할당 */
    set_sample_point(s);

    /* (2) 4×3 행렬 B, C — 초기화된 (Zero) 상태로 선언.
       정적 크기 (Eigen::Matrix<float, 4, 3>) 라 stack 할당, heap 0. */
    Eigen::Matrix<float, 4, 3> B = Eigen::Matrix<float, 4, 3>::Zero();
    Eigen::Matrix<float, 4, 3> C = Eigen::Matrix<float, 4, 3>::Zero();

    /* h = spline_time (1.5s) — 컴파일 타임 상수, 컴파일러가 fold */
    constexpr float h           = detail::spline_time;
    constexpr float three_div_h = 3.0f / h;

    /* (3) B 의 각 행에 우변 항 인가 (1×3 = (3×1)^T) */
    B.row(0) = (three_div_h * (m_points[1] - m_points[0])
              - 3.0f * m_start_velocity).transpose();
    B.row(1) = (three_div_h * (m_points[2] - m_points[1])
              - three_div_h * (m_points[1] - m_points[0])).transpose();
    B.row(2) = (three_div_h * (m_points[3] - m_points[2])
              - three_div_h * (m_points[2] - m_points[1])).transpose();
    B.row(3) = (3.0f * m_end_velocity
              - three_div_h * (m_points[3] - m_points[2])).transpose();

    /* (4) C = inverse_A · B  (4×4 · 4×3 = 4×3) */
    C = detail::inverse_A * B;

    /* (5) C 의 각 행을 transpose → 3×1 Vector3f 로 c1..c4 추출.
       ★ 2026-05-20 refactor: c1..c4 는 *함수 local 변수* (이전엔 detail::cN).
       c1..c3 는 segments[0..2].c 에 저장됨. c4 는 *b3, d3 계산용 임시값* 만이라
       함수 종료 시 자동 소멸 — *detail 보관 불필요*. */
    const Eigen::Vector3f c1 = C.row(0).transpose();
    const Eigen::Vector3f c2 = C.row(1).transpose();
    const Eigen::Vector3f c3 = C.row(2).transpose();
    const Eigen::Vector3f c4 = C.row(3).transpose();

    /* (6) b 계수 — bᵢ = (Pᵢ₊₁ - Pᵢ)/h - (h/3)·(2cᵢ + cᵢ₊₁)
       → m_segments[k].b 에 *직접 저장* */
    constexpr float inv_h    = 1.0f / h;
    constexpr float h_div_3  = h / 3.0f;
    m_segments[0].b = (m_points[1] - m_points[0]) * inv_h
                    - h_div_3 * (2.0f * c1 + c2);
    m_segments[1].b = (m_points[2] - m_points[1]) * inv_h
                    - h_div_3 * (2.0f * c2 + c3);
    m_segments[2].b = (m_points[3] - m_points[2]) * inv_h
                    - h_div_3 * (2.0f * c3 + c4);

    /* (7) c 계수 — segments[k].c = ck */
    m_segments[0].c = c1;
    m_segments[1].c = c2;
    m_segments[2].c = c3;

    /* (8) d 계수 — dᵢ = (cᵢ₊₁ - cᵢ) / (3·h)
       → m_segments[k].d 에 *직접 저장* */
    constexpr float inv_3h = 1.0f / (3.0f * h);
    m_segments[0].d = (c2 - c1) * inv_3h;
    m_segments[1].d = (c3 - c2) * inv_3h;
    m_segments[2].d = (c4 - c3) * inv_3h;

    /* (9) 구간 시작 위치 (P) + 시각 (t_start) 채움.
       evaluate (reconstruct) 함수가 *segments[k] 만 보고* 모든 정보 획득 가능하도록
       *self-contained* 한 segments 보장. */
    m_segments[0].P = m_points[0];
    m_segments[1].P = m_points[1];
    m_segments[2].P = m_points[2];
    m_segments[0].t_start = 0.0f;
    m_segments[1].t_start = h;
    m_segments[2].t_start = 2.0f * h;
}


/* ─────────────────────────────────────────────────────────────────
   ReconstructTrajectory::reconstruct
   ─────────────────────────────────────────────────────────────────
   임의 시각 t ∈ [0.0, 4.5] 에서 *위치 + 속도* 동시 평가.
   사용자 결정 사항 (2026-05-20):
     Q1=B (struct PoseVel 반환)
     Q2=B (if-else 분기 + spline_time 사용)
     Q3=C (외삽 시 NaN 반환 — 호출자가 .pos.allFinite() 로 검사)
     Q4=C (CubicSegment + segments[3] 통합 접근)

   호출자가 보장할 *전제조건*:
     - calculate_clamp_cubic_spline(s) 가 *먼저 호출* 되어 segments 가 채워짐.
     (이 함수는 *전제조건 검사 안 함* — RT path 성능 우선)
   ───────────────────────────────────────────────────────────────── */
PoseVel ReconstructTrajectory::reconstruct(float t)
{
    constexpr float h     = detail::spline_time;     /* 1.5s */
    constexpr float t_max = 3.0f * h;                /* 4.5s */

    /* (Q3-C) 외삽 검사 — t ∉ [0, 4.5] 이면 NaN 반환.
       호출자가 .pos.allFinite() / .vel.allFinite() 로 에러 검출. */
    if (t < 0.0f || t > t_max) {
        const Eigen::Vector3f nan_vec =
            Eigen::Vector3f::Constant(std::numeric_limits<float>::quiet_NaN());
        return PoseVel{ nan_vec, nan_vec };
    }

    /* (Q2-B) 구간 인덱스 결정 — if-else + spline_time 사용.
       반열린 [t_k, t_{k+1}) 컨벤션 + t = t_max 별도 처리 (마지막 구간으로). */
    std::size_t k;
    if (t < h) {
        k = 0;                  /* 구간 0: t ∈ [0.0,  1.5) */
    } else if (t < 2.0f * h) {
        k = 1;                  /* 구간 1: t ∈ [1.5,  3.0) */
    } else {
        k = 2;                  /* 구간 2: t ∈ [3.0,  4.5] — 끝점 포함 */
    }

    /* (Q4-C) segments[k] 통합 접근 */
    const detail::CubicSegment & seg = m_segments[k];
    const float tau = t - seg.t_start;     /* 국소 좌표 τ ∈ [0, h] */

    /* 위치 (Horner): P + τ·(b + τ·(c + τ·d)) */
    const Eigen::Vector3f pos =
        seg.P + tau * (seg.b + tau * (seg.c + tau * seg.d));

    /* 속도 (Horner): b + τ·(2c + τ·3d) — 위치의 1차 미분 */
    const Eigen::Vector3f vel =
        seg.b + tau * (2.0f * seg.c + tau * 3.0f * seg.d);

    return PoseVel{ pos, vel };
}


/* ─────────────────────────────────────────────────────────────────
   debugPrintKeySamples — 4 시점 sample 을 사람-읽기 형식으로 출력.
   호출자: trajectory_replay_node 의 main.cpp predict_timer 람다 (테스트용),
           또는 미래의 Agent 2 수신측 (debug 로깅).
   ───────────────────────────────────────────────────────────────── */
void debugPrintKeySamples(const TrajectorySample & s, std::ostream & os)
{
    auto fmt = [&os](const char * label, const Vec3 & v) {
        os << std::fixed << std::setprecision(4)
           << "  " << label
           << " = (" << std::setw(10) << v.x
           << ", "   << std::setw(10) << v.y
           << ", "   << std::setw(10) << v.z << ")\n";
    };

    os << "─── TrajectorySample (NED, 4 시점) ───\n";
    fmt("t=0.0s  pos", s.pos_t0);
    fmt("t=0.0s  vel", s.vel_t0);
    fmt("t=1.5s  pos", s.pos_t15);
    fmt("t=3.0s  pos", s.pos_t30);
    fmt("t=4.5s  pos", s.pos_t45);
    fmt("t=4.5s  vel", s.vel_t45);
}


/* ─────────────────────────────────────────────────────────────────
   validateKeySamples — NaN / 비정상 값 sanity check.
   true 반환 시 *모든 18 double 값이 finite*.
   false 반환 시 *어딘가 NaN / Inf 있음* (호출자가 fallback 동작 결정).
   ───────────────────────────────────────────────────────────────── */
bool validateKeySamples(const TrajectorySample & s)
{
    auto vec3_finite = [](const Vec3 & v) {
        return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
    };
    return vec3_finite(s.pos_t0)  && vec3_finite(s.vel_t0)  &&
           vec3_finite(s.pos_t15) &&
           vec3_finite(s.pos_t30) &&
           vec3_finite(s.pos_t45) && vec3_finite(s.vel_t45);
}


/* ─────────────────────────────────────────────────────────────────
   computeMaxHorizontalDisplacement — t=0 부터 t=4.5s 까지 *각 sample 까지의*
   수평 거리 중 최댓값 반환. 충돌 회피의 *look-ahead 범위 추정* 에 사용.
   √((pos_tk.x - pos_t0.x)² + (pos_tk.y - pos_t0.y)²) 의 max.
   ───────────────────────────────────────────────────────────────── */
float computeMaxHorizontalDisplacement(const TrajectorySample & s)
{
    /* 내부 계산은 double 로 promote (sqrt 정밀도). 최종 반환만 float narrowing. */
    auto dist = [&s](const Vec3 & p) -> double {
        const double dx = static_cast<double>(p.x) - static_cast<double>(s.pos_t0.x);
        const double dy = static_cast<double>(p.y) - static_cast<double>(s.pos_t0.y);
        return std::sqrt(dx*dx + dy*dy);
    };
    /* fmax 만 사용 — 분기 회피 (CLAUDE 의 §6.2 원칙 정합) */
    double d_max = 0.0;
    d_max = std::fmax(d_max, dist(s.pos_t0));
    d_max = std::fmax(d_max, dist(s.pos_t15));
    d_max = std::fmax(d_max, dist(s.pos_t30));
    d_max = std::fmax(d_max, dist(s.pos_t45));
    return static_cast<float>(d_max);
}

} /* namespace collision_avoidance::estimation */
