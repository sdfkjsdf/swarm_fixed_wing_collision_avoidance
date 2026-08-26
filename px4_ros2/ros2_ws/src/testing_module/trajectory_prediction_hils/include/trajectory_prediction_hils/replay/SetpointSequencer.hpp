#pragma once

/* ────────────────────────────────────────────────────────────
   SetpointSequencer
   책임: yaml 시간-값 테이블 → t_sec → (V_ground_cmd, h_dot, a_lat) lookup

   생성자에서 yaml 1회 적재 → segments 정렬되어 std::vector 보관.
   lookup() 는 rt_thread 에서 단독 호출 — m_cursor_rt 캐시로 amortized O(1).
   시퀀스 종료 시 EndPolicy 정책에 따라 (HoldLast | Cruise | Stop).

   사전 clamp: a_lat 가 tan(roll_max)*g 초과 시, V_ground_cmd 가 설정된
   legacy speed limits 범위 밖일 때 자동 clamp + 적재 시점 1회 WARN.
   ──────────────────────────────────────────────────────────── */

#include <string>
#include <vector>
#include <cstddef>
#include <cmath>
#include <rclcpp/logger.hpp>


class SetpointSequencer
{
public:
    enum class EndPolicy { HoldLast, Cruise, Stop };

    struct Setpoint {
        float V;           /* commanded ground-speed magnitude [m/s] */
        float h_dot;       /* height rate (ENU) [m/s] */
        float a_lat;       /* lateral acceleration [m/s^2] */
        bool  is_fallback; /* true → main thread cruise fallback */
    };

    struct SafetyLimits {
        /* Legacy field names; these limits now bound the ground-speed command. */
        float airspeed_min   = 12.0f;
        float airspeed_max   = 25.0f;
        float max_roll_rad   = 0.873f;  /* 50° */
        float gravity        = 9.80665f;
    };

    SetpointSequencer(const std::string & yaml_path,
                      const SafetyLimits & limits,
                      const rclcpp::Logger & logger);

    /* rt_thread 단독 호출 */
    Setpoint lookup(double t_sec);

    /* onActivate 시 main thread 가 호출 — rt_thread 와 race 없음
       (main thread 가 atomic reinit 신호로 깨운 직후에 rt_thread 가 호출) */
    void resetCursor() { m_cursor_rt = 0; }

    bool empty() const { return m_segments.empty(); }
    EndPolicy endPolicy() const { return m_end_policy; }

    /* 마지막 segment 의 t_end. 시퀀스 비었으면 0.
       호출자가 t > lastTEnd() 로 종료 판정 가능. */
    double lastTEnd() const {
        return m_segments.empty() ? 0.0 : m_segments.back().t_end;
    }

    /* ★ 신규 — 시퀀스의 h_dot 적분으로 t_sec 시점의 *목표 고도* 산출.
       baseline_alt = 시퀀스 시작 시점 (t=0) 의 측정 고도 (Replay::onActivate 에서 캡처).
       PredictInput.h_cmd 의 출처. PX4 의 alt_sp_internal 등가.

       사용 (main.cpp 의 predict 람다):
           const double h_cmd = sequencer->lookupHeightTarget(t_future, baseline_alt);
           PredictInput u{ sp.V, h_cmd, sp.h_dot, sp.a_lat };

       시퀀스 비었거나 baseline NaN 이면 NAN 반환 → stepRK4 가 x.h 로 fallback. */
    double lookupHeightTarget(double t_sec, double baseline_alt) const {
        if (m_segments.empty() || !std::isfinite(baseline_alt)) {
            return std::nan("");
        }
        double h = baseline_alt;
        for (const auto & seg : m_segments) {
            if (t_sec >= seg.t_end) {
                /* 완료된 segment — 전체 duration 적분 */
                h += static_cast<double>(seg.h_dot) * (seg.t_end - seg.t_start);
            } else if (t_sec >= seg.t_start) {
                /* 진행 중 segment — 부분 적분 후 종료 */
                h += static_cast<double>(seg.h_dot) * (t_sec - seg.t_start);
                return h;
            } else {
                /* 아직 시작 안 한 segment — 이전까지 누적치만 반환 */
                return h;
            }
        }
        /* 시퀀스 끝 도달 — 마지막까지 누적된 h_target hold */
        return h;
    }

private:
    struct Segment {
        double t_start;
        double t_end;
        float  V;
        float  h_dot;
        float  a_lat;
    };

    Setpoint toSetpoint(const Segment & s, bool fallback = false) const {
        return {s.V, s.h_dot, s.a_lat, fallback};
    }

    std::vector<Segment> m_segments;
    EndPolicy            m_end_policy{EndPolicy::HoldLast};
    size_t               m_cursor_rt{0};
};
