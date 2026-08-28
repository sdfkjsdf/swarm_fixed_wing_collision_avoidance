#pragma once

/* ════════════════════════════════════════════════════════════════════
   ReconstructTrajectory.hpp — 예측된 trajectory 에서 *4 시점 sample* 추출

   collision_estimation 모듈의 *두 번째 기능* (첫 번째는 trajectory_prediction):
     - trajectory_prediction: 7-state Beard-McLain RK4 적분기 — *trajectory 생산*
     - reconstruct_trajectory (본 파일): 그 trajectory 에서 *충돌 검사 핵심 시점* 만 추출

   ────────────────────────────────────────────────────────────────────
   기능 명세 (2026-05-13 사용자 요구):

   입력:
     - std::array<PredictState, kPredictHorizon> trajectory   (kPredictHorizon=46, endpoint-inclusive)
     - trajectory[k] = 시각 k·dt 에서의 7-state (dt = 0.1s 가정)
     - ★ endpoint-inclusive 컨벤션: kPredictHorizon = endpoint_s × rate_hz + 1
                                   (예: 4.5 × 10 + 1 = 46 점, t = 0.0, 0.1, ..., 4.5)

   출력 (4 시점 sample):
     | 시점       | 인덱스 k | 추출 데이터        |
     |-----------|---------|------------------|
     | t = 0.0 s |   0     | 3D 위치 + 3D 속도 |
     | t = 1.5 s |  15     | 3D 위치            |
     | t = 3.0 s |  30     | 3D 위치            |
     | t = 4.5 s |  45     | 3D 위치 + 3D 속도 |
     (인덱스는 0-based, *마지막 4.5s* 는 horizon 의 last point — dt=0.1×45)

   ────────────────────────────────────────────────────────────────────
   설계 원칙:
     - 빌드 타임 고정 인덱스 (constexpr) — runtime 분기 / lookup table 0
     - POD 출력 (외부 .cpp/.hpp 에서 그대로 사용 가능)
     - 외부 의존 0 (Eigen / rclcpp / px4_msgs include 금지)
     - stateless 함수 — 호출자가 trajectory 메모리 보유
     - rt_thread 친화: heap 0, lock-free, branch-free 가능
   ════════════════════════════════════════════════════════════════════ */

#include <array>
#include <cstddef>

#include <collision_avoidance/estimation/trajectory_prediction/PredictTypes.hpp>
#include <iosfwd>      /* std::ostream forward declaration */
#include <Eigen/Core>  /* Eigen::Matrix4f — inverse_A 4×4 spline 계수 행렬 */


namespace collision_avoidance::estimation
{

/* ── 내부 상수 (private convention) ───────────────────────────────
   namespace detail 안에 두어 *외부 직접 접근 권장 안 함* 신호.
   C++ namespace 에는 `private` 키워드가 없으므로 *detail* 컨벤션으로 표현.
   외부 코드에서 접근 시 collision_estimation::detail::spline_time 표기 →
   *의도적으로 사용하는 것* 임이 분명히 보임. */
namespace detail {
    inline constexpr float spline_time = 1.5f;   /* [s] spline 시간 — 내부 전용 */

    /* ── inverse_A (4×4 cubic spline 계수 역행렬) ──
       A^{-1} = (1 / (45·h)) · [ 26  -7   2  -1
                                  -7  14  -4   2
                                   2  -4  14  -7
                                  -1   2  -7  26 ]
       h = spline_time (1.5s 간격) 으로 가정 → 1 / (45 × 1.5) = 1/67.5 가
       컴파일 타임 상수로 fold 됨. inline const 라 ODR 안전 (여러 TU 정합).
       정적 할당 — Eigen::Matrix4f 는 16 float = 64 byte stack POD. */
    inline const Eigen::Matrix4f inverse_A =
        (Eigen::Matrix4f() <<
             26.f, -7.f,   2.f,  -1.f,
             -7.f, 14.f,  -4.f,   2.f,
              2.f, -4.f,  14.f,  -7.f,
             -1.f,  2.f,  -7.f,  26.f
        ).finished() / (45.0f * spline_time);

    /* ── CubicSegment — spline 의 *한 구간* 표현 ──────────────────
       2026-05-20 refactor: 기존 b1..b3, c1..c4, d1..d3 (15 분산 변수) 를
       *struct + array* 로 통합. 인덱스 k (0,1,2) 로 *균일 접근* — switch/if
       의 *변수명 매핑 분기* 가 *array indexing* 으로 흡수됨.

       구간 k 의 시간영역: τ = t - t_start, τ ∈ [0, spline_time]
       위치 다항식:        p(τ) = P + b·τ + c·τ² + d·τ³
       속도 다항식:        v(τ) = b + 2c·τ + 3d·τ²

       ★ c4 폐기 이유: C 행렬의 4 행 (c4) 은 *b/d 계수 계산 중 임시값* 일 뿐
       *segments 에 보관 불필요*. calculate_clamp_cubic_spline 함수 *local* 로 이동. */
    struct CubicSegment {
        Eigen::Vector3f P       = Eigen::Vector3f::Zero();   /* 구간 시작 위치 (= P_i) */
        Eigen::Vector3f b       = Eigen::Vector3f::Zero();   /* 1차 계수 */
        Eigen::Vector3f c       = Eigen::Vector3f::Zero();   /* 2차 계수 */
        Eigen::Vector3f d       = Eigen::Vector3f::Zero();   /* 3차 계수 */
        float           t_start = 0.0f;                      /* 구간 시작 시각 [s] */
    };

}


/* ════════════════════════════════════════════════════════════════════
   PoseVel — reconstruct(t) 의 반환 타입 (위치 + 속도 묶음)
   ────────────────────────────────────────────────────────────────────
   호출 표기:
     auto pv = reconstruct.reconstruct(2.3f);
     if (!pv.pos.allFinite()) { handle_error(); }   // 외삽 시 NaN 검출
     use(pv.pos, pv.vel);
   ════════════════════════════════════════════════════════════════════ */
struct PoseVel {
    Eigen::Vector3f pos = Eigen::Vector3f::Zero();   /* NED 위치 [m] */
    Eigen::Vector3f vel = Eigen::Vector3f::Zero();   /* NED 속도 [m/s] */
};

/* ★ 2026-05-13 정리: Vec3, TrajectorySample, kSampleIdx_* constexpr 들은 모두
   PredictTypes.hpp 로 이동 (위 #include 통해 자동 import).

   실제 sample 추출 구현은 TrajectoryPredict 의 *멤버 함수* extractKeySamples()
   (TrajectoryPredict.cpp) 가 보유. 본 헤더의 함수들은 *그 결과를 받아서 사용하는*
   수신/처리 측 함수 모음.

   현재 시점 사용처:
     - TrajectoryPredict::extractKeySamples()  : 알고리즘 layer 직접 호출 (RT-safe)
     - debugPrintKeySamples / validateKeySamples / ... (본 헤더): 추출 데이터 *사용*

   향후 (멀티 에이전트 통신 단계):
     - deserializeFromMsg(Float64MultiArray) → TrajectorySample 추가 예정 */


/* ════════════════════════════════════════════════════════════════════
   ReconstructTrajectory 클래스 — cubic spline 재구성 알고리즘

   각 인스턴스가 sample 과 spline 계수를 독립적으로 소유한다. 따라서 B가
   여러 A의 trajectory를 복원해도 다른 인스턴스의 계수를 덮어쓰지 않는다.
   ════════════════════════════════════════════════════════════════════ */
class ReconstructTrajectory
{
public:
    /* ── set_sample_point ────────────────────────────────────────────
       4 시점 sample (TrajectorySample) 의 위치/속도 값을
       각 ReconstructTrajectory 인스턴스의 sample storage 에 *복사 저장*.
         P1 ← s.pos_t0   (t=0.0 s)
         P2 ← s.pos_t15  (t=1.5 s)
         P3 ← s.pos_t30  (t=3.0 s)
         P4 ← s.pos_t45  (t=4.5 s)
         V1 ← s.vel_t0   (t=0.0 s, clamp 조건)
         V4 ← s.vel_t45  (t=4.5 s, clamp 조건)
       반환: void. 부수효과 (본 인스턴스의 sample storage mutation).
       inline 정의 (header 안) — *지금* 사용자 명시. */
    inline void set_sample_point(const TrajectorySample & s)
    {
        m_points[0] = Eigen::Vector3f{ s.pos_t0.x,  s.pos_t0.y,  s.pos_t0.z };
        m_points[1] = Eigen::Vector3f{ s.pos_t15.x, s.pos_t15.y, s.pos_t15.z };
        m_points[2] = Eigen::Vector3f{ s.pos_t30.x, s.pos_t30.y, s.pos_t30.z };
        m_points[3] = Eigen::Vector3f{ s.pos_t45.x, s.pos_t45.y, s.pos_t45.z };
        m_start_velocity = Eigen::Vector3f{
            s.vel_t0.x, s.vel_t0.y, s.vel_t0.z};
        m_end_velocity = Eigen::Vector3f{
            s.vel_t45.x, s.vel_t45.y, s.vel_t45.z};
    }

    /* ── calculate_clamp_cubic_spline ────────────────────────────────
       4 시점 sample (P1..P4) + 양 끝점 속도 (V1, V4) 를 입력으로 받아
       *clamped cubic spline* 의 계수 (P, b, c, d) 를 *3 구간 segments* 에 저장.
       "clamped" = 양 끝점에서 *1차 도함수 (속도)* 가 V1, V4 로 고정.

       매개변수:
         s : TrajectorySample (4 시점 위치 + 양 끝점 속도)
             — 함수 첫 줄에서 set_sample_point(s) 호출 → instance storage 채움.
       반환: void
         계수 저장 위치 (m_segments[k], k=0,1,2):
           .P       : 구간 시작 위치 (= P1, P2, P3)
           .b       : (Pᵢ₊₁ - Pᵢ)/h - (h/3)·(2cᵢ + cᵢ₊₁)
           .c       : C = inverse_A · B 의 k 번째 행 transpose
           .d       : (cᵢ₊₁ - cᵢ) / (3h)
           .t_start : 0.0, spline_time, 2·spline_time */
    void calculate_clamp_cubic_spline(const TrajectorySample & s);


    /* ── reconstruct ─────────────────────────────────────────────────
       임의 시각 t ∈ [0.0, 4.5] 에서 *위치 + 속도* 동시 평가.
       calculate_clamp_cubic_spline 후 *반드시 호출 전에 계수 준비됨* 가정.

       매개변수:
         t : 평가 시각 [s], 정의역 [0.0, 3·spline_time] = [0.0, 4.5]
       반환:
         PoseVel { pos, vel } — NED 3D 위치 + 속도
         외삽 시 (t < 0 or t > 4.5): pos/vel 모두 NaN 으로 채워서 반환.
         호출자는 *.pos.allFinite()* 로 NaN 검출 필수.

       구간 결정 (if-else 분기, spline_time 사용):
         t < spline_time         → segments[0]   (구간 1: [0.0, 1.5])
         t < 2·spline_time       → segments[1]   (구간 2: [1.5, 3.0])
         otherwise               → segments[2]   (구간 3: [3.0, 4.5])

       평가 (Horner 형식):
         위치 = P + τ·(b + τ·(c + τ·d))
         속도 = b + τ·(2c + τ·3d)              ← 위치의 1차 미분
         (τ = t - segments[k].t_start) */
    PoseVel reconstruct(float t);

private:
    std::array<Eigen::Vector3f, 4> m_points{
        Eigen::Vector3f::Zero(),
        Eigen::Vector3f::Zero(),
        Eigen::Vector3f::Zero(),
        Eigen::Vector3f::Zero()};
    Eigen::Vector3f m_start_velocity{Eigen::Vector3f::Zero()};
    Eigen::Vector3f m_end_velocity{Eigen::Vector3f::Zero()};
    std::array<detail::CubicSegment, 3> m_segments{};
};


/* ── debugPrintKeySamples ─────────────────────────────────────────
   4 시점 sample 을 사람-읽기 형식으로 출력. 테스트 / 디버그 용.
   호출자 예:
     TrajectorySample s = predictor.extractKeySamples(traj);
     collision_estimation::debugPrintKeySamples(s, std::cout);
   호출자가 ostream 명시 (헤더의 <iosfwd> 만으로 std::cout 못 가져옴 → <iostream> 회피). */
void debugPrintKeySamples(const TrajectorySample & s,
                          std::ostream & os);


/* ── validateKeySamples ───────────────────────────────────────────
   NaN / Inf sanity check. true = 모든 18 double finite.
   호출자가 publish / 충돌 검사 *전 단계의 가드* 로 사용. */
bool validateKeySamples(const TrajectorySample & s);


/* ── computeMaxHorizontalDisplacement ─────────────────────────────
   t=0 ~ t=4.5s 까지 *수평 displacement* 의 최댓값.
   *look-ahead 범위* (예: collision check 반경 결정) 에 사용.
   ★ 반환 타입 float — Vec3 와 일관 (내부 계산은 double promote, 반환만 narrowing). */
float computeMaxHorizontalDisplacement(const TrajectorySample & s);


} /* namespace collision_avoidance::estimation */
