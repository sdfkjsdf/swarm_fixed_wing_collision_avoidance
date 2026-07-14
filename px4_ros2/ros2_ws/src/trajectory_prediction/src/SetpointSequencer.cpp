/* ════════════════════════════════════════════════════════════════════
   SetpointSequencer.cpp — SetpointSequencer.hpp 구현부
   ────────────────────────────────────────────────────────────────────
   하는 일:
     [생성자] yaml 파일 (setpoint_sequence.yaml) 1회 적재
              → std::vector<Segment> 로 보관 (t_start 기준 정렬)
              → 사전 clamp:
                  V     → [airspeed_min, airspeed_max]
                  a_lat → ±tan(max_roll_rad)·g
                clamp 발생 시 첫 1회만 WARN 로그 (스팸 방지)
              → end_policy 파싱 (hold_last / cruise / stop)

     [lookup(t_sec)] rt_thread 에서 매 1ms 호출:
              - m_cursor_rt 캐시로 amortized O(1) cursor advance
              - 시퀀스 끝 도달 시 EndPolicy 정책에 따라:
                  HoldLast → 마지막 segment 의 (V, h_dot, a_lat) 그대로 반환
                  Cruise/Stop → is_fallback=true 반환
                                (Replay 가 cruise altitude 로 fallback)
              - segment 사이 gap 은 직전 segment 값 hold (ZOH)
              - 첫 segment 시작 전 (t<0 등) 은 cruise fallback

     [resetCursor()] onActivate 시 main thread 가 호출.
              t=0 재캡처 직전 cursor 를 0 으로 리셋.
   ════════════════════════════════════════════════════════════════════ */

#include <trajectory_prediction/SetpointSequencer.hpp>

#include <rclcpp/logging.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>


SetpointSequencer::SetpointSequencer(const std::string & yaml_path,
                                     const SafetyLimits & limits,
                                     const rclcpp::Logger & logger)
{
    YAML::Node root = YAML::LoadFile(yaml_path);

    /* end_policy */
    if (root["end_policy"]) {
        const std::string s = root["end_policy"].as<std::string>();
        if      (s == "hold_last") m_end_policy = EndPolicy::HoldLast;
        else if (s == "cruise")    m_end_policy = EndPolicy::Cruise;
        else if (s == "stop")      m_end_policy = EndPolicy::Stop;
        else {
            RCLCPP_WARN(logger, "[Sequencer] 알 수 없는 end_policy='%s' → hold_last 사용", s.c_str());
            m_end_policy = EndPolicy::HoldLast;
        }
    }

    /* sequence 배열 */
    if (!root["sequence"] || !root["sequence"].IsSequence()) {
        throw std::runtime_error("[Sequencer] yaml 에 'sequence' 배열이 없음: " + yaml_path);
    }

    const float lat_accel_max = std::tan(limits.max_roll_rad) * limits.gravity;
    bool clamp_warned = false;

    for (const auto & node : root["sequence"]) {
        Segment seg;
        seg.t_start = node["t_start"].as<double>();
        seg.t_end   = node["t_end"].as<double>();
        seg.V       = node["airspeed"].as<float>();
        seg.h_dot   = node["height_rate"].as<float>();
        seg.a_lat   = node["lateral_acceleration"].as<float>();

        /* 사전 clamp + 1회 WARN */
        const float V_orig = seg.V;
        const float a_orig = seg.a_lat;
        seg.V     = std::clamp(seg.V, limits.airspeed_min, limits.airspeed_max);
        seg.a_lat = std::clamp(seg.a_lat, -lat_accel_max, lat_accel_max);

        if (!clamp_warned && (seg.V != V_orig || seg.a_lat != a_orig)) {
            RCLCPP_WARN(logger,
                "[Sequencer] segment clamp 발생 (t=%.1f~%.1f): "
                "V %.2f→%.2f, a_lat %.2f→%.2f (이후 동일 종류 WARN 생략)",
                seg.t_start, seg.t_end, V_orig, seg.V, a_orig, seg.a_lat);
            clamp_warned = true;
        }

        m_segments.push_back(seg);
    }

    /* t_start 기준 정렬 */
    std::sort(m_segments.begin(), m_segments.end(),
              [](const Segment & a, const Segment & b) { return a.t_start < b.t_start; });

    RCLCPP_INFO(logger,
        "[Sequencer] 적재 완료: %zu 개 segment, end_policy=%d, "
        "총 시간=%.1fs, lat_accel_max=%.2f m/s^2",
        m_segments.size(), static_cast<int>(m_end_policy),
        m_segments.empty() ? 0.0 : m_segments.back().t_end,
        lat_accel_max);
}

SetpointSequencer::Setpoint SetpointSequencer::lookup(double t_sec)
{
    if (m_segments.empty()) {
        return {NAN, NAN, NAN, true};
    }

    /* cursor advance — 단조 증가 시간 가정. 빈 구간 (gap) 도 자연스럽게 통과 */
    while (m_cursor_rt < m_segments.size()
           && t_sec >= m_segments[m_cursor_rt].t_end) {
        m_cursor_rt++;
    }

    /* 시퀀스 끝 도달 */
    if (m_cursor_rt >= m_segments.size()) {
        switch (m_end_policy) {
            case EndPolicy::HoldLast:
                return toSetpoint(m_segments.back(), /*fallback=*/false);
            case EndPolicy::Cruise:
            case EndPolicy::Stop:
            default:
                return {NAN, NAN, NAN, true};
        }
    }

    const Segment & s = m_segments[m_cursor_rt];

    /* 첫 segment 시작 전 (t_sec < t_start[0]) 또는 segment 사이 gap */
    if (t_sec < s.t_start) {
        if (m_cursor_rt == 0) {
            /* 첫 segment 시작 전 → cruise fallback */
            return {NAN, NAN, NAN, true};
        }
        /* gap 안에 있으면 직전 segment 의 값 hold (ZOH 의도) */
        return toSetpoint(m_segments[m_cursor_rt - 1], /*fallback=*/false);
    }

    return {s.V, s.h_dot, s.a_lat, false};
}
