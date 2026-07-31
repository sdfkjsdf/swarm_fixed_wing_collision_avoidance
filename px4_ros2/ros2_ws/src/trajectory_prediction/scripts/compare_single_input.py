#!/usr/bin/env python3
# Phase 1 case 의 *단일 입력 sustained 4.5초 응답* 비교 — 시각화 + 표
#
# 본 script 는 chunk_analysis.py 와 *상보적*:
#   chunk_analysis.py : rolling CSV → 4.5s 간격 chunk 7개 평균 metric
#   compare_single_input.py (본 script): 자극 시각 t=5.0s 단 한 점 → 4.5s 응답 비교
#
# 명세: ~/.claude/uploads/.../f9bf9292-predict_track_revise.md
# 입력 CSV : ~/ros2_ws/results/cases/{V01~V04,H01~H04,P01~P04}_*.csv
# 출력      : ~/ros2_ws/results/single_input/{case}_compare.png + summary.{csv,png}
#
# 색 규약: 빨강 dashed = predicted, 녹색 solid = measured.

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# 매크로 (chunk_analysis.py 와 일관 — 절대 변경하지 말 것)
# ★ 2026-05-13 endpoint-inclusive 컨벤션: N_PREDICT = round(endpoint × rate) + 1
#   (kPredictHorizon 동일 정의. 양 끝점 포함 → 4.5s × 10 + 1 = 46 점, t=0.0,0.1,...,4.5)
LOG_RATE_HZ               = 50
PREDICT_RATE_HZ           = 10
PREDICT_HORIZON_ENDPOINT_S = 4.5
N_PREDICT                 = int(round(PREDICT_HORIZON_ENDPOINT_S * PREDICT_RATE_HZ)) + 1   # = 46
TICKS_PER_PREDICT         = 5      # LOG_RATE_HZ / PREDICT_RATE_HZ

# ★ 본 작업 (2026-05-13): 동적 자극 시각 검출 — true_airspeed >= 20 도달 첫 row.
# 이전: STIMULUS_TIME_S = 5.0 고정 (CSV 시작 + 5s) — V 도달 보장 안 됨.
# 새: airspeed 도달 후 *진짜 fixed-wing cruise* 자료에서 4.5s window 추출.
AIRSPEED_THRESHOLD = 19.5   # [m/s] 본 임계값 이상 도달한 첫 row 부터 window 시작.
                            #   ★ V_cmd=20 명령 시 asymptotic 도달 (V(t) = 20 - 2e^{-t/5})
                            #     이라 *정확 20* 도달 못 함. PX4 정상상태 ≈ V_cmd ± 0.5 m/s
                            #     자리에서 검출. 사용자 의도 'V=20 도달 후 분석' 의 *실용 임계*.
MIN_SUSTAIN_S      = 1.0    # [s]   CSV 시작 후 최소 대기 (Replay 진입 직후 transient 회피)


# ───────────────────────────────────────────────────────────────────
# 0. Helper — angle wrap-safe 시각화
# ───────────────────────────────────────────────────────────────────

def _unwrap_align_deg(arr_rad, ref_first_rad):
    """rad 시계열을 unwrap 후 ref_first_rad 에 가장 가까운 2π branch 로 align → degrees.

    PX4 yaw 는 [-π, π] wrap, RK4 누적 ψ 는 wrap 없음 → 둘이 *다른 branch* 일 수 있어
    plot 에서 시각적 *360° 갭* 으로 보일 risk. 본 함수가 그 갭 제거.

    err 계산은 sin/cos trick 으로 이미 wrap-safe (compute_errors 참조).
    """
    u   = np.unwrap(arr_rad)
    adj = np.round((u[0] - ref_first_rad) / (2 * np.pi)) * 2 * np.pi
    return np.degrees(u - adj)


# ───────────────────────────────────────────────────────────────────
# 1. 데이터 추출
# ───────────────────────────────────────────────────────────────────

def extract_single_input_data(csv_path):
    """단일 입력 4.5초 데이터 추출 — *true_airspeed ≥ AIRSPEED_THRESHOLD 도달 첫 row* 부터 window.

    Returns:
        dict | None : {case, t, pred, meas, pred_xy_dist, meas_xy_dist}
                       None 이면 airspeed 도달 못 했거나 4.5초 데이터 부족.

    동작:
      1. CSV 시작 + MIN_SUSTAIN_S (1초) 후의 row 인덱스 찾음 (transient 회피)
      2. 그 인덱스 *이후* 에서 true_airspeed ≥ AIRSPEED_THRESHOLD (20 m/s) 첫 row 검출
      3. 그 row 부터 4.5초 window 추출
    """
    df = pd.read_csv(csv_path)

    # ★ CSV 시작 + MIN_SUSTAIN_S (1초) 이후의 첫 row index (Replay 진입 transient 회피)
    t_base = df['t_us'].iloc[0] * 1e-6
    min_t  = t_base + MIN_SUSTAIN_S
    min_idx_mask = df['t_us'] * 1e-6 >= min_t
    if not min_idx_mask.any():
        print(f"[warning] {csv_path}: CSV 길이 < {MIN_SUSTAIN_S}s")
        return None
    min_start_idx = min_idx_mask.idxmax()

    # ★ 자극 시각 검출 — *sp_a 변화 첫 row* (시퀀스 segment 전환 시각)
    # 본 시퀀스 = 2 segment: baseline (sp_a=0) + 자극 (sp_a=비0). sp_a 변화 시점이 자극 시작.
    # min_start_idx 이후에서 sp_a 의 |값| > eps 첫 row 검출.
    eps = 0.01
    sp_a_change_mask = (df.index >= min_start_idx) & (df['sp_a'].abs() > eps)
    if not sp_a_change_mask.any():
        print(f"[warning] {csv_path}: sp_a 변화 (자극) 검출 안 됨")
        return None
    raw_stimulus_row = int(sp_a_change_mask.idxmax())

    # ★ 본 작업 (2026-05-13): predict_timer (10Hz, 100ms 주기) 와 Replay updateSetpoint
    # (30Hz, ~33ms 주기) 의 timing race 해소.
    #
    # 전제: updateSetpoint 가 sp_a 변화를 감지하고 setBaselineAlt() 호출 *직후*,
    #       predict_timer 가 새 baseline 을 사용하기까지 *최대 1 predict cycle (100ms)* 소요.
    # 적용: +5 row (= 100ms) 이동 → 재캡처 *후* 첫 predict 결과 row 사용.
    #
    # ⚠ 본 보정은 *스케줄링 지터* 에 의존:
    #   - 단일 thread executor (rclcpp::spin) 라 *콜백 순차 실행*, race 없음
    #   - 그러나 실제 delay 는 33~200ms 범위 가능 (Gazebo SITL 부하)
    #   - 본 SITL 환경에서 +5 row 실험적 검증 (h end 평균 0.57m → 0.16m, 72% 개선)
    #   - 만약 *지터 큰 환경* 또는 *느린 SITL* 에서는 +10 row (200ms) 필요 가능
    #
    # 검증 기준: R 시리즈 h end < 0.4m 일관 유지. 이상 시 +TICKS_PER_PREDICT*2 시도.
    stimulus_row = raw_stimulus_row + TICKS_PER_PREDICT

    # ★ 추가 안전망 — 자극 시작 시점의 V ≥ AIRSPEED_THRESHOLD 점검 (보고 only)
    V_at_stim = df.iloc[stimulus_row]['true_airspeed']
    t_stimulus = df.iloc[stimulus_row]['t_us'] * 1e-6 - t_base
    if V_at_stim < AIRSPEED_THRESHOLD:
        print(f"[warning] {csv_path}: 자극 시각 (t={t_stimulus:.2f}s) 의 V={V_at_stim:.2f} "
              f"< {AIRSPEED_THRESHOLD} m/s — V 도달 충분치 않음 (baseline 더 늘리기 권장)")

    # 4.5초 (46 점 × 5 ticks, endpoint-inclusive) 만큼 데이터가 남아 있는지 확인
    if stimulus_row + N_PREDICT * TICKS_PER_PREDICT >= len(df):
        print(f"[warning] {csv_path}: 자극 시각 (t={t_stimulus:.2f}s) 후 4.5초 데이터 부족")
        return None

    print(f"[info] {Path(csv_path).stem}: airspeed >= {AIRSPEED_THRESHOLD} 도달 시각 "
          f"t={t_stimulus:.2f}s, V_at_stim={df.iloc[stimulus_row]['true_airspeed']:.2f} m/s")

    head = df.iloc[stimulus_row]
    t = np.arange(N_PREDICT) * 0.1   # 0 ~ 4.5s (0.1s 간격, 46 점 endpoint-inclusive)

    # Predicted trajectory — CSV 의 p_*_k 컬럼 (예측 t=0 인 자극 시각 행)
    pred = {
        'pn':   np.array([head[f'p_pn_{k}']   for k in range(N_PREDICT)]),
        'pe':   np.array([head[f'p_pe_{k}']   for k in range(N_PREDICT)]),
        'h':    np.array([head[f'p_h_{k}']    for k in range(N_PREDICT)]),
        'V':    np.array([head[f'p_V_{k}']    for k in range(N_PREDICT)]),
        'psi':  np.array([head[f'p_psi_{k}']  for k in range(N_PREDICT)]),
        'hdot': np.array([head[f'p_hdot_{k}'] for k in range(N_PREDICT)]),
        'phi':  np.array([head[f'p_phi_{k}']  for k in range(N_PREDICT)]),
    }

    # Measured trajectory — 50Hz CSV 의 row (stimulus_row + 5k) 다운샘플 → 10Hz.
    #
    # ★ Option A (2026-05-13): m_*_atp 컬럼 사용.
    #   *_atp = "measurement at predict-time" — TrajectoryLogger 가 publishPredictBundle()
    #   호출 시점에 freeze 한 측정 snapshot. 같은 행의 p_*_0 와 *수학적으로 동일* 한
    #   변수에서 derive 됨 → t=0 오차 ≡ 0 (예측 100Hz / 측정 50Hz race 제거).
    #
    # ★ 2026-05-13 course-angle 패치: main.cpp 의 x0.psi 가 m.yaw (body) → atan2(m.ve, m.vn)
    #   (course) 로 바뀌었으므로, 여기서도 meas['psi'] = m_yaw_atp 가 *아니라*
    #   atan2(m_ve_atp, m_vn_atp) (course angle) 로 비교해야 *같은 정의 vs 같은 정의*.
    #   t=0 에서 p_psi_0 = atan2(m_ve_atp, m_vn_atp) 이고 meas['psi'][0] 도 같으므로
    #   ψ 오차 ≡ 0. 부수 효과: 3D velocity 벡터 오차도 *V·(cos,sin)(course) vs ground v*
    #   의 비교가 되어 sideslip artifact (1.15 m/s) 제거 → t=0 = 0.
    #
    # 이전 reference: m_yaw_atp 는 *body yaw (quaternion)*, course 와 sideslip 차이 발생.
    # vn/ve/vd 는 PX4 EKF 의 NED ground velocity. 풍 0 가정 시 horizontal ground speed
    # ≈ airspeed → V·cos(course) ≈ vn.
    meas = {
        'pn':   np.array([df.iloc[stimulus_row + 5*k]['m_x_atp']    for k in range(N_PREDICT)]),
        'pe':   np.array([df.iloc[stimulus_row + 5*k]['m_y_atp']    for k in range(N_PREDICT)]),
        'h':    np.array([-df.iloc[stimulus_row + 5*k]['m_z_atp']   for k in range(N_PREDICT)]),   # NED → ENU
        # ★ V = √(vn²+ve²+vd²) — main.cpp 의 x0.V 와 *같은 source* 로 통일.
        #   이전 m_V_atp 는 airspeed_validated 토픽 (별도 EKF 출력) 으로, vn/ve/vd 와
        #   inconsistency 발생. 외란 (풍) 부재 가정 하에 V_air ≡ V_g 라 *같은 EKF v 벡터 크기*
        #   하나로 통일 → t=0 V error 도 0 보장.
        'V':    np.array([np.sqrt(df.iloc[stimulus_row + 5*k]['m_vn_atp']**2 +
                                   df.iloc[stimulus_row + 5*k]['m_ve_atp']**2 +
                                   df.iloc[stimulus_row + 5*k]['m_vd_atp']**2)
                          for k in range(N_PREDICT)]),
        # ★ course angle = atan2(ve, vn). body yaw 가 아닌 ground track 방향.
        'psi':  np.array([np.arctan2(df.iloc[stimulus_row + 5*k]['m_ve_atp'],
                                      df.iloc[stimulus_row + 5*k]['m_vn_atp']) for k in range(N_PREDICT)]),
        'hdot': np.array([-df.iloc[stimulus_row + 5*k]['m_vd_atp']  for k in range(N_PREDICT)]),   # NED → +up
        'phi':  np.array([df.iloc[stimulus_row + 5*k]['m_roll_atp'] for k in range(N_PREDICT)]),
        'vn':   np.array([df.iloc[stimulus_row + 5*k]['m_vn_atp']   for k in range(N_PREDICT)]),
        've':   np.array([df.iloc[stimulus_row + 5*k]['m_ve_atp']   for k in range(N_PREDICT)]),
        'vd':   np.array([df.iloc[stimulus_row + 5*k]['m_vd_atp']   for k in range(N_PREDICT)]),
        # ★ 디버그용 — body yaw 도 함께 가져와서 *crab angle* 모니터링 가능
        'yaw_body': np.array([df.iloc[stimulus_row + 5*k]['m_yaw_atp'] for k in range(N_PREDICT)]),
    }

    # 시작점 기준 XY 이동 거리 (자극 시각 부터의 trajectory length)
    pred_xy_dist = np.sqrt((pred['pn'] - pred['pn'][0])**2
                           + (pred['pe'] - pred['pe'][0])**2)
    meas_xy_dist = np.sqrt((meas['pn'] - meas['pn'][0])**2
                           + (meas['pe'] - meas['pe'][0])**2)

    return {
        'case':          Path(csv_path).stem.split('_')[0],
        't':             t,
        'pred':          pred,
        'meas':          meas,
        'pred_xy_dist':  pred_xy_dist,
        'meas_xy_dist':  meas_xy_dist,
    }


# ───────────────────────────────────────────────────────────────────
# 2. 오차 계산
# ───────────────────────────────────────────────────────────────────

def compute_errors(data):
    """4.5초 전 구간 통계: end (k=44) + mean (bias) + std (산포) + RMS (통합).

    RMS = sqrt(mean(err²)) = sqrt(mean² + std²)  (단일 통합 지표).
    """
    pred = data['pred']
    meas = data['meas']

    err_xy_t = np.sqrt((pred['pn'] - meas['pn'])**2
                       + (pred['pe'] - meas['pe'])**2)
    err_h    = np.abs(pred['h']    - meas['h'])
    err_V    = np.abs(pred['V']    - meas['V'])
    # angle wrap [-π, π] — atan2(sin, cos) trick
    err_psi  = np.abs(np.arctan2(np.sin(pred['psi'] - meas['psi']),
                                  np.cos(pred['psi'] - meas['psi'])))
    err_phi  = np.abs(np.arctan2(np.sin(pred['phi'] - meas['phi']),
                                  np.cos(pred['phi'] - meas['phi'])))
    err_hdot = np.abs(pred['hdot'] - meas['hdot'])

    def _stat(arr, to_deg=False):
        a = np.degrees(arr) if to_deg else arr
        return (float(a[-1]), float(a.mean()),
                float(a.std()), float(np.sqrt((a**2).mean())))

    xy_e,   xy_m,   xy_s,   xy_r   = _stat(err_xy_t)
    h_e,    h_m,    h_s,    h_r    = _stat(err_h)
    V_e,    V_m,    V_s,    V_r    = _stat(err_V)
    psi_e,  psi_m,  psi_s,  psi_r  = _stat(err_psi,  to_deg=True)
    phi_e,  phi_m,  phi_s,  phi_r  = _stat(err_phi,  to_deg=True)
    hd_e,   hd_m,   hd_s,   hd_r   = _stat(err_hdot)

    return {
        'xy_end':    xy_e,  'xy_mean':    xy_m,  'xy_std':    xy_s,  'xy_rms':    xy_r,
        'h_end':     h_e,   'h_mean':     h_m,   'h_std':     h_s,   'h_rms':     h_r,
        'V_end':     V_e,   'V_mean':     V_m,   'V_std':     V_s,   'V_rms':     V_r,
        'psi_end':   psi_e, 'psi_mean':   psi_m, 'psi_std':   psi_s, 'psi_rms':   psi_r,
        'phi_end':   phi_e, 'phi_mean':   phi_m, 'phi_std':   phi_s, 'phi_rms':   phi_r,
        'hdot_end':  hd_e,  'hdot_mean':  hd_m,  'hdot_std':  hd_s,  'hdot_rms':  hd_r,
    }


# ───────────────────────────────────────────────────────────────────
# 3. case 별 시각화 (3x3 격자, 7 채널 + Errors text + 빈 자리)
# ───────────────────────────────────────────────────────────────────

def plot_case(data, description, output_path):
    """case 의 7 채널 측정 vs 예측 시각화."""
    fig, axes = plt.subplots(3, 3, figsize=(18, 12))
    case_id = data['case']
    fig.suptitle(
        f'{case_id} -- {description}\n'
        f't = 5.0~9.5s (single input 4.5s response)',
        fontsize=14,
    )

    t    = data['t']
    pred = data['pred']
    meas = data['meas']

    # ★ 2026-05-13: 전 채널 *오차 norm 단일 라인* 자리. 측정 vs 예측 두 라인 자리 폐기.
    #   식은 compute_errors (line 173-211) 와 동일 — atan2(sin,cos) wrap-safe trick + |·|.
    #   색/스타일은 3D 위치/속도 norm ([0,2], [2,0]) 자리 와 일관 — 검정 단일 라인.
    err_style = {'color': 'black', 'linewidth': 2.5}

    # 채널별 시간축 오차 — scalar 5 자리 + 3D norm 2 자리
    err_V_t    = np.abs(pred['V']    - meas['V'])
    err_h_t    = np.abs(pred['h']    - meas['h'])
    err_hdot_t = np.abs(pred['hdot'] - meas['hdot'])
    err_psi_t  = np.degrees(np.abs(np.arctan2(np.sin(pred['psi'] - meas['psi']),
                                               np.cos(pred['psi'] - meas['psi']))))
    err_phi_t  = np.degrees(np.abs(np.arctan2(np.sin(pred['phi'] - meas['phi']),
                                               np.cos(pred['phi'] - meas['phi']))))

    # [0,0] V error
    axes[0, 0].plot(t, err_V_t, **err_style,
                    label=r'$|V_{pred} - V_{meas}|$')
    axes[0, 0].set_title('V error (airspeed)')
    axes[0, 0].set_xlabel('t [s]')
    axes[0, 0].set_ylabel('V error [m/s]')
    axes[0, 0].grid(alpha=0.3); axes[0, 0].legend(fontsize=9)

    # [0,1] h error
    axes[0, 1].plot(t, err_h_t, **err_style,
                    label=r'$|h_{pred} - h_{meas}|$')
    axes[0, 1].set_title('h error (altitude)')
    axes[0, 1].set_xlabel('t [s]')
    axes[0, 1].set_ylabel('h error [m]')
    axes[0, 1].grid(alpha=0.3); axes[0, 1].legend(fontsize=9)

    # [0,2] 3D 위치 노름 오차 — ‖p_pred − p_meas‖₂
    pos_err = np.sqrt((pred['pn'] - meas['pn'])**2
                    + (pred['pe'] - meas['pe'])**2
                    + (pred['h']  - meas['h'])**2)
    axes[0, 2].plot(t, pos_err, **err_style,
                    label=r'$\|\mathbf{p}_{pred} - \mathbf{p}_{meas}\|_2$')
    axes[0, 2].set_title('3D position error norm')
    axes[0, 2].set_xlabel('t [s]')
    axes[0, 2].set_ylabel('position error [m]')
    axes[0, 2].grid(alpha=0.3); axes[0, 2].legend(fontsize=9)

    # [1,0] psi error — wrap-safe (atan2(sin, cos) trick), deg 단위.
    # ★ 2026-05-13: psi = course angle (atan2(ve, vn)) — body yaw 아님.
    #   trajectory prediction 의 *방향* state 의 의미가 ground track 으로 일관됨.
    axes[1, 0].plot(t, err_psi_t, **err_style,
                    label=r'$|\chi_{pred} - \chi_{meas}|$  (course, wrap-safe)')
    axes[1, 0].set_title('psi error (course angle)')
    axes[1, 0].set_xlabel('t [s]')
    axes[1, 0].set_ylabel('course angle error [deg]')
    axes[1, 0].grid(alpha=0.3); axes[1, 0].legend(fontsize=9)

    # [1,1] phi error — wrap-safe (실용상 |phi|<π/2 라 wrap 거의 없음, 일관성 위해)
    axes[1, 1].plot(t, err_phi_t, **err_style,
                    label=r'$|\phi_{pred} - \phi_{meas}|$  (wrap-safe)')
    axes[1, 1].set_title('phi error (roll)')
    axes[1, 1].set_xlabel('t [s]')
    axes[1, 1].set_ylabel('phi error [deg]')
    axes[1, 1].grid(alpha=0.3); axes[1, 1].legend(fontsize=9)

    # [1,2] h_dot error
    axes[1, 2].plot(t, err_hdot_t, **err_style,
                    label=r'$|\dot{h}_{pred} - \dot{h}_{meas}|$')
    axes[1, 2].set_title('h_dot error (climb rate)')
    axes[1, 2].set_xlabel('t [s]')
    axes[1, 2].set_ylabel('h_dot error [m/s]')
    axes[1, 2].grid(alpha=0.3); axes[1, 2].legend(fontsize=9)

    # [2,0] ★ 3D 속도 벡터 노름 오차 — ‖v_pred − v_meas‖₂
    # predicted velocity (식 기반: V_h·cos(psi), V_h·sin(psi), -h_dot)
    vn_p, ve_p, vd_p = _pred_ned_velocity(pred)
    # measured velocity (PX4 EKF, NED ground velocity)
    vn_m, ve_m, vd_m = meas['vn'], meas['ve'], meas['vd']
    vel_err = np.sqrt((vn_p - vn_m)**2 + (ve_p - ve_m)**2 + (vd_p - vd_m)**2)
    axes[2, 0].plot(t, vel_err, color='black', linewidth=2.5,
                    label=r'$\|\mathbf{v}_{pred} - \mathbf{v}_{meas}\|_2$')
    axes[2, 0].set_title('3D velocity error norm  (★ 신규)')
    axes[2, 0].set_xlabel('t [s]')
    axes[2, 0].set_ylabel('velocity error [m/s]')
    axes[2, 0].grid(alpha=0.3); axes[2, 0].legend(fontsize=9)

    # [2,1] Errors text — 4.5초 전 구간 통계: mean±std + RMS + end
    err  = compute_errors(data)
    text = (
        f"4.5s trajectory error stats:\n"
        f"  channel    mean +/- std       RMS      end\n"
        f"  --------  ----------------  -------  -------\n"
        f"  XY [m]    {err['xy_mean']:5.2f} +/- {err['xy_std']:4.2f}    "
            f"{err['xy_rms']:5.2f}    {err['xy_end']:5.2f}\n"
        f"  h  [m]    {err['h_mean']:5.2f} +/- {err['h_std']:4.2f}    "
            f"{err['h_rms']:5.2f}    {err['h_end']:5.2f}\n"
        f"  V  [m/s]  {err['V_mean']:5.2f} +/- {err['V_std']:4.2f}    "
            f"{err['V_rms']:5.2f}    {err['V_end']:5.2f}\n"
        f"  psi[deg]  {err['psi_mean']:5.2f} +/- {err['psi_std']:4.2f}    "
            f"{err['psi_rms']:5.2f}    {err['psi_end']:5.2f}\n"
        f"  phi[deg]  {err['phi_mean']:5.2f} +/- {err['phi_std']:4.2f}    "
            f"{err['phi_rms']:5.2f}    {err['phi_end']:5.2f}\n"
        f"  hdot[m/s] {err['hdot_mean']:5.2f} +/- {err['hdot_std']:4.2f}    "
            f"{err['hdot_rms']:5.2f}    {err['hdot_end']:5.2f}\n"
        f"\n"
        f"Reading:\n"
        f"  mean = bias (systematic offset)\n"
        f"  std  = deviation around mean\n"
        f"  RMS  = sqrt(mean^2 + std^2) integrated\n"
        f"  end  = error at t=4.5s (1 point)\n"
    )
    axes[2, 1].text(0.05, 0.95, text, transform=axes[2, 1].transAxes,
                    verticalalignment='top', fontsize=9, family='monospace')
    axes[2, 1].axis('off')
    axes[2, 1].set_title('Errors (4.5s window)')

    # [2,2] empty
    axes[2, 2].axis('off')

    plt.tight_layout()
    plt.savefig(output_path, dpi=100)
    plt.close()
    print(f"[output] {case_id} PNG: {output_path}")


# ───────────────────────────────────────────────────────────────────
# 3.5. Velocity arrow trajectory (별도 PNG)
# ───────────────────────────────────────────────────────────────────

def _pred_ned_velocity(pred):
    """predicted state → NED velocity (vn, ve, vd).

    본 모델은 airspeed (V) + heading (psi) + climb rate (h_dot) 기반.
    측정 PX4 vn/ve/vd 는 *ground velocity* (풍속 포함) → 풍 0 가정 시 동치.
    풍속 ≠ 0 인 자리에서는 *systematic offset* 발생 — 본 모델 한계의 정량 시그널.
    """
    V_h = np.sqrt(np.maximum(pred['V']**2 - pred['hdot']**2, 0.0))
    vn  = V_h * np.cos(pred['psi'])
    ve  = V_h * np.sin(pred['psi'])
    vd  = -pred['hdot']     # h_dot (+up, ENU) → NED v_down (+down) 부호 반전
    return vn, ve, vd


def plot_velocity_arrows(data, description, output_path):
    """단순 1x2 layout: 큰 top-down XY view + velocity error.

    [0] Top-down XY trajectory + horizontal velocity 화살표 (큼)
    [1] Velocity error — |v_pred - v_meas| magnitude + 성분별 (vn/ve/vd) 오차
    """
    pred = data['pred']
    meas = data['meas']
    t    = data['t']

    vn_p, ve_p, vd_p = _pred_ned_velocity(pred)
    vn_m, ve_m, vd_m = meas['vn'], meas['ve'], meas['vd']

    # 성분별 부호 있는 오차 (pred - meas)
    err_vn  = vn_p - vn_m
    err_ve  = ve_p - ve_m
    err_vd  = vd_p - vd_m
    # 벡터 오차 magnitude — 3차원 합성
    err_mag = np.sqrt(err_vn**2 + err_ve**2 + err_vd**2)

    # quiver — 5 점마다 (0.5s 간격, 화살표 9 개)
    idx         = np.arange(0, N_PREDICT, 5)
    arrow_scale = 0.3   # [m / (m/s)]

    fig, axes = plt.subplots(1, 2, figsize=(18, 8))
    fig.suptitle(
        f'{data["case"]} -- {description}\n'
        f'XY trajectory (with velocity arrows) | velocity error',
        fontsize=13,
    )

    qv_args = dict(scale_units='xy', angles='xy', scale=1, width=0.004, alpha=0.7)

    # ── [0] Top-down XY + horizontal velocity arrows ──
    ax = axes[0]
    ax.plot(pred['pe'], pred['pn'], 'r--', linewidth=2.5, label='predicted path')
    ax.plot(meas['pe'], meas['pn'], 'g-',  linewidth=2.5, label='measured path')
    ax.quiver(pred['pe'][idx], pred['pn'][idx],
              ve_p[idx] * arrow_scale, vn_p[idx] * arrow_scale,
              color='red',   label='pred v', **qv_args)
    ax.quiver(meas['pe'][idx], meas['pn'][idx],
              ve_m[idx] * arrow_scale, vn_m[idx] * arrow_scale,
              color='green', label='meas v', **qv_args)
    ax.scatter([pred['pe'][0]],  [pred['pn'][0]],  c='black', s=120, marker='o',
               label='start',     zorder=5)
    ax.scatter([pred['pe'][-1]], [pred['pn'][-1]], c='red',   s=120, marker='^',
               label='pred end',  zorder=5)
    ax.scatter([meas['pe'][-1]], [meas['pn'][-1]], c='green', s=120, marker='s',
               label='meas end',  zorder=5)
    ax.set(title='Top-down XY trajectory  (arrows = velocity vectors, 0.5s 간격)',
           xlabel='East [m]', ylabel='North [m]')
    ax.set_aspect('equal'); ax.grid(alpha=0.3)
    ax.legend(fontsize=10, loc='best')

    # ── [1] Velocity error (vector magnitude + 성분별) ──
    ax = axes[1]
    ax.plot(t, err_mag, color='black',   linewidth=2.8,
            label=r'$|\mathbf{v}_{pred} - \mathbf{v}_{meas}|$  (vector magnitude)')
    ax.plot(t, err_vn,  color='#1f77b4', linestyle='--', linewidth=1.6,
            label='vn err (North,  pred − meas)')
    ax.plot(t, err_ve,  color='#ff7f0e', linestyle='--', linewidth=1.6,
            label='ve err (East,   pred − meas)')
    ax.plot(t, err_vd,  color='#2ca02c', linestyle='--', linewidth=1.6,
            label='vd err (Down,   pred − meas)')
    ax.axhline(0, color='gray', linewidth=0.5)

    mag_mean = err_mag.mean()
    mag_std  = err_mag.std()
    mag_max  = err_mag.max()
    mag_end  = err_mag[-1]
    ax.set(
        title=(f'Velocity error  '
               f'|Δv|: mean={mag_mean:.3f}, std={mag_std:.3f}, '
               f'max={mag_max:.3f}, end={mag_end:.3f}  [m/s]'),
        xlabel='t [s]', ylabel='velocity error [m/s]',
    )
    ax.grid(alpha=0.3); ax.legend(fontsize=10, loc='best')

    plt.tight_layout()
    plt.savefig(output_path, dpi=100)
    plt.close()
    print(f"[output] {data['case']} velocity PNG: {output_path}")


# ───────────────────────────────────────────────────────────────────
# 4. 종합 출력 (표, CSV, 막대 그래프)
# ───────────────────────────────────────────────────────────────────

def print_summary_table(results):
    """3 section 출력: mean±std (전 구간 산포) / RMS (통합) / end (1 점)."""

    print("\n=== 4.5초 전 구간 평균 ± 표준편차 (mean ± std) ===\n")
    print(f"{'Case':<5} {'Input':<35} "
          f"{'XY mean±std':<14} {'h mean±std':<13} {'V mean±std':<14} "
          f"{'psi mean±std':<14} {'phi mean±std':<14}")
    print("-" * 115)
    for r in results:
        print(f"{r['case']:<5} {r['description']:<35} "
              f"{r['xy_mean']:5.2f}±{r['xy_std']:4.2f} m  "
              f"{r['h_mean']:5.2f}±{r['h_std']:4.2f} m "
              f"{r['V_mean']:5.2f}±{r['V_std']:4.2f} m/s "
              f"{r['psi_mean']:5.2f}±{r['psi_std']:4.2f} ° "
              f"{r['phi_mean']:5.2f}±{r['phi_std']:4.2f} °")

    print("\n=== 4.5초 RMS = sqrt(mean^2 + std^2) (bias + 산포 통합) ===\n")
    print(f"{'Case':<5} {'XY RMS':<10} {'h RMS':<10} {'V RMS':<11} "
          f"{'psi RMS':<11} {'phi RMS':<11}")
    print("-" * 70)
    for r in results:
        print(f"{r['case']:<5} {r['xy_rms']:>7.2f}m  {r['h_rms']:>6.2f}m   "
              f"{r['V_rms']:>6.2f}m/s {r['psi_rms']:>7.2f}°   "
              f"{r['phi_rms']:>7.2f}°")

    print("\n=== 4.5초 끝점 (t=4.5s, 1 점) ===\n")
    print(f"{'Case':<5} {'XY end':<10} {'h end':<10} {'V end':<11} "
          f"{'psi end':<11} {'phi end':<11}")
    print("-" * 70)
    for r in results:
        print(f"{r['case']:<5} {r['xy_end']:>7.2f}m  {r['h_end']:>6.2f}m   "
              f"{r['V_end']:>6.2f}m/s {r['psi_end']:>7.2f}°   "
              f"{r['phi_end']:>7.2f}°")


def save_summary_csv(results, output_path):
    if not results:
        return
    with open(output_path, 'w') as f:
        writer = csv.DictWriter(f, fieldnames=results[0].keys())
        writer.writeheader()
        for r in results:
            writer.writerow(r)
    print(f"\n[output] Summary CSV: {output_path}")


def plot_summary_bars(results, output_path):
    """채널별 mean ± std 막대 그래프 (2x3 격자).

    bar 높이 = mean, error bar = std. RMS 는 별도 marker (X)로 overlay.
    """
    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    fig.suptitle('Single Input 4.5s Response -- Trajectory Error (mean ± std, RMS marker)')

    case_ids = [r['case'] for r in results]
    colors   = []
    for case_id in case_ids:
        if   case_id.startswith('V'): colors.append('#1f77b4')
        elif case_id.startswith('H'): colors.append('#ff7f0e')
        elif case_id.startswith('P'): colors.append('#2ca02c')
        else:                          colors.append('#7f7f7f')

    channels = [('xy',  'XY [m]'),
                ('h',   'h [m]'),
                ('V',   'V [m/s]'),
                ('psi', 'psi [deg]'),
                ('phi', 'phi [deg]')]

    for idx, (key, label) in enumerate(channels):
        ax    = axes[idx // 3, idx % 3]
        means = [r[f'{key}_mean'] for r in results]
        stds  = [r[f'{key}_std']  for r in results]
        rmses = [r[f'{key}_rms']  for r in results]

        ax.bar(case_ids, means, yerr=stds, color=colors,
               capsize=4, ecolor='black', alpha=0.85, label='mean ± std')
        ax.scatter(case_ids, rmses, color='red', marker='x', s=60,
                   zorder=5, label='RMS')
        ax.set_title(label)
        ax.tick_params(axis='x', rotation=45)
        ax.grid(axis='y', alpha=0.3)
        if idx == 0:
            ax.legend(loc='upper left', fontsize=9)

    # [1,2] 색 범례
    axes[1, 2].axis('off')
    axes[1, 2].legend(
        handles=[
            plt.Rectangle((0, 0), 1, 1, color='#1f77b4', label='V channel (airspeed)'),
            plt.Rectangle((0, 0), 1, 1, color='#ff7f0e', label='H channel (h_dot)'),
            plt.Rectangle((0, 0), 1, 1, color='#2ca02c', label='P channel (phi)'),
            plt.Line2D([0], [0], marker='x', color='red', linestyle='',
                       markersize=10, label='RMS (mean^2 + std^2)^0.5'),
        ],
        loc='center', fontsize=11,
    )

    plt.tight_layout()
    plt.savefig(output_path, dpi=100)
    plt.close()
    print(f"[output] Summary PNG: {output_path}")


# ───────────────────────────────────────────────────────────────────
# 5. Main
# ───────────────────────────────────────────────────────────────────

PHASE_1_CASES = {
    # ★ R 시리즈 만 활성 — roll setpoint sweep (±5°/±15°/±25°), V=20 cruise
    'R05P': 'right turn phi=+5°  (a_lat +0.858, V=20, alt hold)',
    'R05M': 'left turn  phi=-5°  (a_lat -0.858, V=20, alt hold)',
    'R15P': 'right turn phi=+15° (a_lat +2.628, V=20, alt hold)',
    'R15M': 'left turn  phi=-15° (a_lat -2.628, V=20, alt hold)',
    'R25P': 'right turn phi=+25° (a_lat +4.572, V=20, alt hold)',
    'R25M': 'left turn  phi=-25° (a_lat -4.572, V=20, alt hold)',
}


def main():
    parser = argparse.ArgumentParser(
        description='Phase 1 case 의 단일 입력 4.5초 응답 비교',
    )
    parser.add_argument('--results-dir', default='~/ros2_ws/results/cases/',
                        help='Phase 1 CSV 디렉터리')
    parser.add_argument('--output-dir',  default='~/ros2_ws/results/single_input/',
                        help='출력 PNG/CSV 디렉터리')
    args = parser.parse_args()

    results_dir = Path(args.results_dir).expanduser()
    output_dir  = Path(args.output_dir).expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)

    all_results = []
    for case_id, description in PHASE_1_CASES.items():
        # 가장 최신 timestamp CSV 선택 (같은 case_id 의 여러 run)
        csv_files = sorted(results_dir.glob(f'{case_id}_*.csv'))
        if not csv_files:
            print(f"[warning] {case_id}: CSV 없음 (skip)")
            continue
        csv_path = csv_files[-1]

        data = extract_single_input_data(str(csv_path))
        if data is None:
            continue

        plot_case(data, description, output_dir / f'{case_id}_compare.png')
        plot_velocity_arrows(data, description, output_dir / f'{case_id}_velocity.png')

        err = compute_errors(data)
        err['case']        = case_id
        err['description'] = description
        all_results.append(err)

    if not all_results:
        print("\n[error] 처리된 case 없음 — results-dir 확인")
        return

    print_summary_table(all_results)
    save_summary_csv(all_results, output_dir / 'summary.csv')
    plot_summary_bars(all_results, output_dir / 'summary.png')


if __name__ == '__main__':
    main()
