#!/usr/bin/env python3
"""
analyze_cases.py — Stage A-3 (claude_code_task_spec.md §A-3-2)
===============================================================

run_all_cases.sh 가 생성한 results/cases/{ID}_<stamp>.csv 들을 자동 분석:

    Phase 1 (12): step input → 1차 lag fitting → τ + R²
    Phase 2 ( 6): sustained → steady bias (예측 vs 측정 평균 오차)
    Phase 3 ( 4): coupling → 단일 채널 모델 예측 vs 결합 case 실측
    Phase 4 ( 3): avoidance → 4.5초 호라이즌 95-percentile 오차 (ε_TPA)

출력:
    <out>/phase1_fitting.png        (12 subplot)
    <out>/phase2_bias.png           ( 6 subplot)
    <out>/phase3_coupling.png       ( 4 subplot)
    <out>/phase4_avoidance.png      ( 3 subplot)
    <out>/analysis_report.md        (4 표 + 권장 사항)

사용:
    ./analyze_cases.py --results results/cases/
    ./analyze_cases.py --results results/cases/ --phase 1
    ./analyze_cases.py --results results/cases/ --out-dir results/
"""

import argparse
import glob
import math
import os
import sys
from typing import Optional

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml

try:
    from scipy.optimize import curve_fit
except ImportError:
    sys.exit('ERROR: scipy가 설치된 Python 환경이 필요합니다.')


# ─── 상수 ──────────────────────────────────────────────────────────
GRAVITY = 9.80665
PREDICT_HORIZON_S = 4.5
N_PREDICT         = 45


# ─── helper ────────────────────────────────────────────────────────
def first_order_lag(t, y_inf, y_0, tau):
    """ 1차 lag: y(t) = y_inf + (y_0 - y_inf) * exp(-t/tau) """
    return y_inf + (y_0 - y_inf) * np.exp(-t / tau)


def load_case_matrix(matrix_path: str):
    with open(matrix_path) as f:
        return yaml.safe_load(f)


def find_csv(results_dir: str, case_id: str) -> Optional[str]:
    """ results/cases/{ID}_*.csv 중 최신 1개. 없으면 None. """
    files = sorted(glob.glob(os.path.join(results_dir, f'{case_id}_*.csv')),
                   key=os.path.getmtime)
    return files[-1] if files else None


def load_csv(csv_path: str) -> pd.DataFrame:
    df = pd.read_csv(csv_path)
    df['t_rel'] = (df['t_us'] - df['t_us'].iloc[0]) / 1e6  # seconds
    return df


# ═══════════════════════════════════════════════════════════════════
# Phase 1: step input → τ fitting
# ═══════════════════════════════════════════════════════════════════
def analyze_step_response(csv_path: str, meta: dict) -> dict:
    """
    1차 lag fitting.
    meta: {id, channel, baseline, step_value, t_step}
    """
    df = load_csv(csv_path)
    channel    = meta['channel']
    t_step     = float(meta['t_step'])
    step_value = float(meta['step_value'])
    # step 후 15초만 fitting (settling 보장)
    mask = (df['t_rel'] >= t_step) & (df['t_rel'] <= t_step + 15.0)
    sub  = df.loc[mask].reset_index(drop=True)
    if len(sub) < 50:
        return {'id': meta['id'], 'channel': channel, 'error': 'insufficient data'}

    t = (sub['t_rel'] - t_step).to_numpy()

    if channel == 'V':
        y_meas = sub['true_airspeed'].to_numpy()
        unit, sp_label = 'm/s', 'V'
    elif channel == 'hdot':
        y_meas = -sub['vd'].to_numpy()       # NED vd (down) → climb-rate (up)
        unit, sp_label = 'm/s', 'h_dot'
    elif channel == 'phi':
        if 'roll' not in sub.columns:
            return {'id': meta['id'], 'channel': channel,
                    'error': 'CSV 에 roll 컬럼 없음 — A-1 후 재실행 필요'}
        y_meas = sub['roll'].to_numpy()
        # step_value 는 a_lat → 등가 phi 로 변환
        step_value = math.atan2(step_value, GRAVITY)
        unit, sp_label = 'rad', 'phi'
    else:
        return {'id': meta['id'], 'channel': channel, 'error': f'unknown channel {channel}'}

    # NaN guard
    if np.any(np.isnan(y_meas)):
        finite = ~np.isnan(y_meas)
        t, y_meas = t[finite], y_meas[finite]
    if len(y_meas) < 50:
        return {'id': meta['id'], 'channel': channel, 'error': 'too many NaN'}

    # initial guess
    y0_g  = y_meas[0]
    yinf_g = y_meas[-5:].mean() if len(y_meas) >= 5 else y_meas[-1]
    tau_g = 2.0

    try:
        popt, _ = curve_fit(first_order_lag, t, y_meas,
                            p0=[yinf_g, y0_g, tau_g],
                            bounds=([-1e3, -1e3, 0.05], [1e3, 1e3, 30.0]),
                            maxfev=5000)
        y_fit = first_order_lag(t, *popt)
        ss_res = np.sum((y_meas - y_fit)**2)
        ss_tot = np.sum((y_meas - y_meas.mean())**2)
        r_sq = 1 - ss_res / ss_tot if ss_tot > 1e-12 else 0.0
        return {
            'id': meta['id'], 'channel': channel,
            'sp_label': sp_label, 'unit': unit,
            'amplitude': meta.get('amplitude'),
            'y_inf_target': step_value,
            'y_inf_fit': popt[0], 'y_0_fit': popt[1], 'tau': popt[2],
            'r_sq': r_sq,
            't': t, 'y_meas': y_meas, 'y_fit': y_fit,
        }
    except Exception as e:
        return {'id': meta['id'], 'channel': channel, 'error': str(e)}


def plot_phase1(results: list, out_path: str):
    """ 12 case fitting curve 한 PNG (4×3 grid). """
    n = len(results)
    cols = 4
    rows = (n + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(4 * cols, 3 * rows), squeeze=False)
    fig.suptitle('Phase 1: Step Input — 1st-order Lag Fitting', fontsize=13)

    for i, r in enumerate(results):
        ax = axes[i // cols][i % cols]
        if 'error' in r:
            ax.text(0.5, 0.5, f"{r['id']}\n{r['error']}",
                    ha='center', va='center', transform=ax.transAxes)
            ax.set_title(r['id'])
            ax.axis('off')
            continue
        ax.plot(r['t'], r['y_meas'], 'b-',  linewidth=1.5, label='measured')
        ax.plot(r['t'], r['y_fit'],  'r--', linewidth=1.5, label='fit')
        ax.axhline(r['y_inf_target'], color='g', linestyle=':', alpha=0.6, label='target')
        status = 'OK' if r['r_sq'] >= 0.95 else ('WARN' if r['r_sq'] >= 0.85 else 'REJECT')
        ax.set_title(f"{r['id']} ({r['channel']}, Δ={r['amplitude']}) "
                     f"τ={r['tau']:.3f}s R²={r['r_sq']:.3f} [{status}]", fontsize=9)
        ax.set_xlabel('t after step [s]')
        ax.set_ylabel(f"{r['sp_label']} [{r['unit']}]")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7, loc='best')

    for i in range(n, rows * cols):
        axes[i // cols][i % cols].axis('off')

    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)
    print(f'[plot] {out_path}')


# ═══════════════════════════════════════════════════════════════════
# Phase 2: sustained → steady bias
# ═══════════════════════════════════════════════════════════════════
def analyze_steady_bias(csv_path: str, meta: dict) -> dict:
    """ 마지막 5초 구간의 예측 vs 측정 평균 차이 (steady bias). """
    df = load_csv(csv_path)
    t_max = df['t_rel'].iloc[-1]
    mask = df['t_rel'] >= t_max - 5.0
    sub = df.loc[mask].reset_index(drop=True)
    if len(sub) < 50:
        return {'id': meta['id'], 'error': 'insufficient data'}

    # chunk_analysis 와 동일한 채널 정의로 평균 오차
    pred_pn  = sub[[f'p_pn_{k}'  for k in range(N_PREDICT)]].iloc[0].to_numpy()
    pred_pe  = sub[[f'p_pe_{k}'  for k in range(N_PREDICT)]].iloc[0].to_numpy()
    pred_h   = sub[[f'p_h_{k}'   for k in range(N_PREDICT)]].iloc[0].to_numpy()
    pred_V   = sub[[f'p_V_{k}'   for k in range(N_PREDICT)]].iloc[0].to_numpy()
    pred_psi = sub[[f'p_psi_{k}' for k in range(N_PREDICT)]].iloc[0].to_numpy()
    pred_phi = sub[[f'p_phi_{k}' for k in range(N_PREDICT)]].iloc[0].to_numpy()

    # 측정 단순 평균 (steady state)
    m_x   = sub['x'].mean()
    m_y   = sub['y'].mean()
    m_h   = -sub['z'].mean()
    m_V   = sub['true_airspeed'].mean()
    m_psi = sub['yaw'].mean()
    m_phi = sub['roll'].mean() if 'roll' in sub.columns else np.nan

    # 예측의 끝점 (4.5s 미래) vs 현재 측정 (대략적)
    return {
        'id': meta['id'],
        'scenario': meta.get('scenario'),
        'xy_bias': float(np.hypot(pred_pn[-1] - m_x, pred_pe[-1] - m_y)),
        'h_bias':  float(abs(pred_h[-1]  - m_h)),
        'V_bias':  float(abs(pred_V[-1]  - m_V)),
        'psi_bias': float(np.degrees(abs(np.arctan2(np.sin(pred_psi[-1] - m_psi),
                                                     np.cos(pred_psi[-1] - m_psi))))),
        'phi_bias': float(np.degrees(abs(pred_phi[-1] - m_phi)))
                    if not np.isnan(m_phi) else np.nan,
    }


def plot_phase2(results: list, out_path: str):
    if not results:
        return
    fig, ax = plt.subplots(figsize=(12, 5))
    ids = [r['id'] for r in results if 'error' not in r]
    metrics = ['xy_bias', 'h_bias', 'V_bias', 'psi_bias', 'phi_bias']
    x = np.arange(len(ids))
    width = 0.15
    for i, m in enumerate(metrics):
        vals = [r[m] if r.get(m) is not None and not (isinstance(r[m], float) and np.isnan(r[m])) else 0
                for r in results if 'error' not in r]
        ax.bar(x + i*width, vals, width, label=m)
    ax.set_xticks(x + width*2)
    ax.set_xticklabels(ids)
    ax.set_ylabel('bias')
    ax.set_title('Phase 2: Steady-State Bias (predicted 4.5s end vs measured mean)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)
    print(f'[plot] {out_path}')


# ═══════════════════════════════════════════════════════════════════
# Phase 3: coupling
# ═══════════════════════════════════════════════════════════════════
def analyze_coupling(csv_path: str, meta: dict) -> dict:
    """ 결합 case 의 mean error (단순 chunk_analysis 와 같은 metric). """
    df = load_csv(csv_path)
    # 마지막 5초 중 첫 행의 predict trajectory vs 그 후 4.5초 measured
    mask = df['t_rel'] >= 5.0   # step 이후
    sub  = df.loc[mask].reset_index(drop=True)
    if len(sub) < 250:
        return {'id': meta['id'], 'error': 'insufficient data'}

    head = sub.iloc[0]
    pred_pn = np.array([head[f'p_pn_{k}'] for k in range(N_PREDICT)])
    pred_pe = np.array([head[f'p_pe_{k}'] for k in range(N_PREDICT)])
    pred_h  = np.array([head[f'p_h_{k}']  for k in range(N_PREDICT)])
    pred_V  = np.array([head[f'p_V_{k}']  for k in range(N_PREDICT)])

    # measured (0, 5, 10, ..., 220 행 — 0.1s 간격)
    rows = list(range(0, N_PREDICT * 5, 5))
    rows = [r for r in rows if r < len(sub)]
    if len(rows) < N_PREDICT:
        return {'id': meta['id'], 'error': 'insufficient samples'}
    s2 = sub.iloc[rows]
    m_pn = s2['x'].to_numpy()
    m_pe = s2['y'].to_numpy()
    m_h  = -s2['z'].to_numpy()
    m_V  = s2['true_airspeed'].to_numpy()

    err_xy = np.hypot(pred_pn - m_pn, pred_pe - m_pe)
    err_h  = np.abs(pred_h - m_h)
    err_V  = np.abs(pred_V - m_V)

    return {
        'id': meta['id'],
        'channels': meta.get('channels'),
        'xy_end': float(err_xy[-1]), 'xy_mean': float(err_xy.mean()),
        'h_end':  float(err_h[-1]),  'h_mean':  float(err_h.mean()),
        'V_end':  float(err_V[-1]),  'V_mean':  float(err_V.mean()),
    }


def plot_phase3(results: list, out_path: str):
    if not results:
        return
    valid = [r for r in results if 'error' not in r]
    if not valid:
        return
    fig, axes = plt.subplots(1, 3, figsize=(15, 4))
    ids = [r['id'] for r in valid]
    x = np.arange(len(ids))
    axes[0].bar(x, [r['xy_mean'] for r in valid]); axes[0].set_title('XY mean error [m]')
    axes[1].bar(x, [r['h_mean']  for r in valid]); axes[1].set_title('h mean error [m]')
    axes[2].bar(x, [r['V_mean']  for r in valid]); axes[2].set_title('V mean error [m/s]')
    for ax in axes:
        ax.set_xticks(x); ax.set_xticklabels(ids); ax.grid(True, alpha=0.3)
    fig.suptitle('Phase 3: Coupling — combined channel errors')
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)
    print(f'[plot] {out_path}')


# ═══════════════════════════════════════════════════════════════════
# Phase 4: avoidance → ε_TPA
# ═══════════════════════════════════════════════════════════════════
def analyze_avoidance(csv_path: str, meta: dict) -> dict:
    """
    회피 시나리오의 4.5s rolling horizon 95-percentile XY 오차.
    rolling chunk_analysis 와 동일 원리.
    """
    df = load_csv(csv_path)
    # 모든 row 의 predict end (k=44) vs 그 row + 4.5s 측정의 차이
    rows_total = len(df)
    chunk_ticks = 225   # 4.5s × 50Hz
    end_errs_xy = []
    end_errs_h  = []
    for r in range(0, rows_total - chunk_ticks, 5):    # 10Hz sampling
        head = df.iloc[r]
        # predict end
        pe_x = head[f'p_pn_{N_PREDICT-1}']
        pe_y = head[f'p_pe_{N_PREDICT-1}']
        pe_h = head[f'p_h_{N_PREDICT-1}']
        # measured 4.5s 후
        m = df.iloc[r + chunk_ticks - 1]
        if any(np.isnan([pe_x, pe_y, pe_h, m['x'], m['y'], m['z']])):
            continue
        end_errs_xy.append(math.hypot(pe_x - m['x'], pe_y - m['y']))
        end_errs_h.append(abs(pe_h - (-m['z'])))

    if not end_errs_xy:
        return {'id': meta['id'], 'error': 'insufficient rolling samples'}
    xy = np.array(end_errs_xy)
    h  = np.array(end_errs_h)
    return {
        'id': meta['id'],
        'scenario': meta.get('scenario'),
        'xy_p50': float(np.percentile(xy, 50)),
        'xy_p95': float(np.percentile(xy, 95)),   # ★ ε_TPA candidate
        'xy_max': float(xy.max()),
        'h_p50':  float(np.percentile(h, 50)),
        'h_p95':  float(np.percentile(h, 95)),
        'n_samples': int(len(xy)),
    }


def plot_phase4(results: list, out_path: str):
    if not results:
        return
    valid = [r for r in results if 'error' not in r]
    if not valid:
        return
    fig, ax = plt.subplots(figsize=(10, 5))
    ids = [r['id'] for r in valid]
    x = np.arange(len(ids))
    width = 0.25
    ax.bar(x - width, [r['xy_p50'] for r in valid], width, label='XY p50')
    ax.bar(x,         [r['xy_p95'] for r in valid], width, label='XY p95 (ε_TPA)')
    ax.bar(x + width, [r['xy_max'] for r in valid], width, label='XY max')
    ax.set_xticks(x); ax.set_xticklabels(ids)
    ax.set_title('Phase 4: Avoidance — rolling 4.5s horizon XY error')
    ax.set_ylabel('XY error [m]')
    ax.legend(); ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)
    print(f'[plot] {out_path}')


# ═══════════════════════════════════════════════════════════════════
# Markdown 보고서
# ═══════════════════════════════════════════════════════════════════
def write_report(out_dir: str, p1: list, p2: list, p3: list, p4: list):
    lines = []
    lines.append('# Trajectory Predictor 모델 검증 보고서')
    lines.append('')
    lines.append(f'_생성: `analyze_cases.py`, 결과 dir: `{out_dir}`_')
    lines.append('')

    # Phase 1 — 사용자 결정 (2026-05-11): R² ≥ 0.95 만 평균에 포함, R² < 0.95 는 별도 보고
    lines.append('## τ 측정 결과 (Phase 1)')
    lines.append('')
    lines.append('| 채널 | Case | 진폭 | τ_fit [s] | R² | Status |')
    lines.append('|---|---|---|---|---|---|')
    # 채널-키 매핑 (사용자 결정: airframe_spec.yaml 의 tc_tas/tc_alt/tc_roll 그대로 유지)
    CHANNEL_KEY = {'V': 'tc_tas', 'hdot': 'tc_alt', 'phi': 'tc_roll'}
    channel_taus = {'V': [], 'hdot': [], 'phi': []}
    rejected = []
    for r in p1:
        if 'error' in r:
            lines.append(f"| {r['channel']} | {r['id']} | — | — | — | ERROR: {r['error']} |")
            continue
        # ★ 사용자 결정: R² ≥ 0.95 = OK 만 평균에 포함. 0.95 미만은 모두 REJECT 로 표기.
        status = 'OK' if r['r_sq'] >= 0.95 else 'REJECT'
        if status == 'OK':
            channel_taus[r['channel']].append(r['tau'])
        else:
            rejected.append(r)
        lines.append(f"| {r['channel']} | {r['id']} | {r['amplitude']:+} "
                     f"| {r['tau']:.3f} | {r['r_sq']:.3f} | {status} |")
    lines.append('')

    lines.append('## 채널별 최종 τ (R² ≥ 0.95 평균)')
    lines.append('')
    for ch, taus in channel_taus.items():
        key = CHANNEL_KEY[ch]
        if taus:
            mean = np.mean(taus); std = np.std(taus)
            lines.append(f'- **τ_{ch} ({key})**: {mean:.3f} s  '
                         f'(n={len(taus)}, σ={std:.3f})')
        else:
            lines.append(f'- **τ_{ch} ({key})**: N/A — 모든 case R² < 0.95')
    lines.append('')

    # R² < 0.95 보고 (거르지 말고)
    lines.append('## R² < 0.95 케이스 보고 (사용자 결정: 거르지 말고 그대로 보고)')
    lines.append('')
    if rejected:
        lines.append('| 채널 | Case | 진폭 | τ_fit [s] | R² | 영역 |')
        lines.append('|---|---|---|---|---|---|')
        for r in rejected:
            region = f"{r['channel']} Δ={r['amplitude']:+}"
            lines.append(f"| {r['channel']} | {r['id']} | {r['amplitude']:+} "
                         f"| {r['tau']:.3f} | {r['r_sq']:.3f} | {region} |")
        lines.append('')
        lines.append('> 위 case 의 가정 깨지는 영역 (graceful degradation 임계 도출) 은 본 작업 외 — 별도 작업.')
    else:
        lines.append('R² < 0.95 case 없음 — 모든 case 의 1차 lag 가정 유효.')
    lines.append('')

    # Phase 2
    if p2:
        lines.append('## Phase 2: Steady-State Bias')
        lines.append('')
        lines.append('| Case | Scenario | XY bias [m] | h bias [m] | V bias [m/s] | psi bias [°] | phi bias [°] |')
        lines.append('|---|---|---|---|---|---|---|')
        for r in p2:
            if 'error' in r:
                lines.append(f"| {r['id']} | — | — | — | — | — | ERROR |")
                continue
            phi_s = f"{r['phi_bias']:.2f}" if not np.isnan(r.get('phi_bias', np.nan)) else 'N/A'
            lines.append(f"| {r['id']} | {r['scenario']} "
                         f"| {r['xy_bias']:.2f} | {r['h_bias']:.3f} "
                         f"| {r['V_bias']:.3f} | {r['psi_bias']:.2f} | {phi_s} |")
        lines.append('')

    # Phase 3
    if p3:
        lines.append('## Phase 3: Coupling')
        lines.append('')
        lines.append('| Case | Channels | XY end [m] | XY mean [m] | h mean [m] | V mean [m/s] |')
        lines.append('|---|---|---|---|---|---|')
        for r in p3:
            if 'error' in r:
                lines.append(f"| {r['id']} | — | ERROR | — | — | — |")
                continue
            ch_str = ','.join(r['channels']) if r.get('channels') else '—'
            lines.append(f"| {r['id']} | {ch_str} | {r['xy_end']:.2f} "
                         f"| {r['xy_mean']:.2f} | {r['h_mean']:.3f} | {r['V_mean']:.3f} |")
        lines.append('')

    # Phase 4
    if p4:
        lines.append('## Phase 4: Avoidance — ε_TPA (95-percentile rolling 4.5s XY)')
        lines.append('')
        lines.append('| Case | Scenario | XY p50 [m] | XY p95 (ε_TPA) [m] | XY max [m] | h p95 [m] | n |')
        lines.append('|---|---|---|---|---|---|---|')
        for r in p4:
            if 'error' in r:
                lines.append(f"| {r['id']} | — | ERROR | — | — | — | — |")
                continue
            lines.append(f"| {r['id']} | {r['scenario']} "
                         f"| {r['xy_p50']:.2f} | {r['xy_p95']:.2f} | {r['xy_max']:.2f} "
                         f"| {r['h_p95']:.3f} | {r['n_samples']} |")
        lines.append('')

    # 다음 단계 제안 (사용자 결정 요청 자리)
    lines.append('## 다음 단계 제안')
    lines.append('')
    if any(channel_taus.values()):
        lines.append('### B-3 코드 갱신 후보 (config/airframe_spec.yaml)')
        lines.append('')
        lines.append('```yaml')
        for ch, taus in channel_taus.items():
            if taus:
                mean = np.mean(taus)
                key = CHANNEL_KEY[ch]
                lines.append(f'{key}: {mean:.3f}    # fitting mean of {len(taus)} valid case(s) (R²≥0.95)')
        lines.append('```')
        lines.append('')
        all_channels_have_tau = all(len(taus) > 0 for taus in channel_taus.values())
        if all_channels_have_tau and not rejected:
            lines.append('→ **B-3 코드 갱신 진행해도 OK**: 3 채널 모두 OK, REJECT 없음.')
        elif all_channels_have_tau:
            lines.append('→ **B-3 코드 갱신 진행 가능** (3 채널 평균 모두 확보).')
            lines.append('   단, REJECT case 의 graceful degradation 임계는 별도 작업.')
        else:
            missing = [CHANNEL_KEY[ch] for ch, taus in channel_taus.items() if not taus]
            lines.append(f'→ **사용자 결정 필요**: {", ".join(missing)} 채널의 모든 case R² < 0.95.')
            lines.append('   - 옵션 A: 해당 채널은 placeholder 유지하고 다른 채널만 갱신')
            lines.append('   - 옵션 B: 측정 시퀀스 재설계 후 재실행')
    else:
        lines.append('→ **사용자 결정 필요**: 모든 채널 평균 τ 미확보.')
    lines.append('')

    report_path = os.path.join(out_dir, 'analysis_report.md')
    with open(report_path, 'w') as f:
        f.write('\n'.join(lines))
    print(f'[report] {report_path}')


# ═══════════════════════════════════════════════════════════════════
# main
# ═══════════════════════════════════════════════════════════════════
def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--results', required=True, help='case CSV 디렉토리')
    p.add_argument('--matrix', required=True, help='case_matrix.yaml 경로')
    p.add_argument('--out-dir', required=True, help='plot + 보고서 출력 디렉토리')
    p.add_argument('--phase', default='all',
                   help='분석 phase: 1 / 2 / 3 / 4 / 1,2,3,4 / all')
    args = p.parse_args()

    if not os.path.isdir(args.results):
        sys.exit(f'ERROR: results dir 없음: {args.results}')
    os.makedirs(args.out_dir, exist_ok=True)

    matrix = load_case_matrix(args.matrix)
    phases = args.phase.split(',') if args.phase != 'all' else ['1', '2', '3', '4']

    p1, p2, p3, p4 = [], [], [], []

    # Phase 1
    if '1' in phases:
        print('\n=== Phase 1: Step Input τ Fitting ===')
        for meta in matrix.get('phase_1_step_input', []):
            csv = find_csv(args.results, meta['id'])
            if not csv:
                print(f"  [{meta['id']}] CSV 없음 — skip")
                p1.append({'id': meta['id'], 'channel': meta['channel'], 'error': 'no CSV'})
                continue
            r = analyze_step_response(csv, meta)
            p1.append(r)
            if 'error' in r:
                print(f"  [{r['id']}] ERROR: {r['error']}")
            else:
                print(f"  [{r['id']}] τ={r['tau']:.3f}s  R²={r['r_sq']:.3f}")
        if p1:
            plot_phase1(p1, os.path.join(args.out_dir, 'phase1_fitting.png'))

    # Phase 2
    if '2' in phases:
        print('\n=== Phase 2: Steady Bias ===')
        for meta in matrix.get('phase_2_sustained', []):
            csv = find_csv(args.results, meta['id'])
            if not csv:
                p2.append({'id': meta['id'], 'error': 'no CSV'})
                continue
            r = analyze_steady_bias(csv, meta)
            p2.append(r)
            if 'error' in r:
                print(f"  [{r['id']}] ERROR: {r['error']}")
            else:
                print(f"  [{r['id']}] xy_bias={r['xy_bias']:.2f}m")
        if p2:
            plot_phase2(p2, os.path.join(args.out_dir, 'phase2_bias.png'))

    # Phase 3
    if '3' in phases:
        print('\n=== Phase 3: Coupling ===')
        for meta in matrix.get('phase_3_coupling', []):
            csv = find_csv(args.results, meta['id'])
            if not csv:
                p3.append({'id': meta['id'], 'error': 'no CSV'})
                continue
            r = analyze_coupling(csv, meta)
            p3.append(r)
            if 'error' in r:
                print(f"  [{r['id']}] ERROR: {r['error']}")
            else:
                print(f"  [{r['id']}] xy_mean={r['xy_mean']:.2f}m")
        if p3:
            plot_phase3(p3, os.path.join(args.out_dir, 'phase3_coupling.png'))

    # Phase 4
    if '4' in phases:
        print('\n=== Phase 4: Avoidance ε_TPA ===')
        for meta in matrix.get('phase_4_avoidance', []):
            csv = find_csv(args.results, meta['id'])
            if not csv:
                p4.append({'id': meta['id'], 'error': 'no CSV'})
                continue
            r = analyze_avoidance(csv, meta)
            p4.append(r)
            if 'error' in r:
                print(f"  [{r['id']}] ERROR: {r['error']}")
            else:
                print(f"  [{r['id']}] ε_TPA (p95)={r['xy_p95']:.2f}m  max={r['xy_max']:.2f}m")
        if p4:
            plot_phase4(p4, os.path.join(args.out_dir, 'phase4_avoidance.png'))

    write_report(args.out_dir, p1, p2, p3, p4)
    print('\n[analyze_cases] 완료.')


if __name__ == '__main__':
    main()
