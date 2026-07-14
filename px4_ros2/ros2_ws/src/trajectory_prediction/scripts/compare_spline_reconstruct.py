#!/usr/bin/env python3
"""compare_spline_reconstruct.py — spline 재구성 정확도 검증.

★ 본 script 의 목적:
  TrajectoryPredict 의 *RK4 적분 결과 (46 점)* 와 ReconstructTrajectory 의
  *clamped cubic spline 복원 (46 점)* 을 시각적으로 비교 → spline 의 *fitting
  정확도* 정량화.

★ compare_single_input.py 와의 차이:
  | 비교 종류    | compare_single_input.py | 본 script                  |
  |-------------|--------------------------|----------------------------|
  | 시리즈 A    | predicted (RK4)          | predicted (RK4)            |
  | 시리즈 B    | measured (PX4 실제)      | reconstructed (spline)     |
  | 데이터 출처  | predict CSV 1 개         | predict CSV + spline CSV   |

★ 입력 (옵션 C tick-aligned):
  - predict CSV: results/cases/{case}_<timestamp>.csv  (50Hz, ~1500 row)
  - spline  CSV: results/cases/{case}_spline_<timestamp>.csv  (50Hz, ~1500 row)
  - 두 CSV 의 t_us 는 *완전 동일* (TrajectoryLogger.onTick 이 한 번에 둘 다 write)

★ 출력:
  - results/reconstruct_spline/{case}_spline_compare.png  (8 panel)

★ 색 규약 (compare_single_input.py 와 통일):
  - 빨강 dashed = predicted (RK4)
  - 파랑 solid  = reconstructed (spline)
"""

import argparse
import glob
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


# ────────────────────────────────────────────────────────────
# 상수 (TrajectoryLogger / TrajectoryPredict 와 동기화)
# ────────────────────────────────────────────────────────────
LOG_RATE_HZ                = 50
PREDICT_RATE_HZ            = 10
PREDICT_HORIZON_ENDPOINT_S = 4.5
N_PREDICT                  = int(round(PREDICT_HORIZON_ENDPOINT_S * PREDICT_RATE_HZ)) + 1  # = 46

AIRSPEED_THRESHOLD = 19.5   # compare_single_input.py 와 동일
MIN_SUSTAIN_S      = 1.0


# ────────────────────────────────────────────────────────────
# 1. 데이터 로드 + 자극 시점 검출
# ────────────────────────────────────────────────────────────

def find_latest_csv_pair(results_cases_dir: Path, case: str):
    """{case}_<timestamp>.csv 와 {case}_spline_<timestamp>.csv 짝 검색.

    동일 STAMP 가진 두 파일을 반환. spline CSV 는 _spline_ 접두 패턴.
    가장 최근 (timestamp 기준) 짝 반환.
    """
    predict_csvs = sorted(results_cases_dir.glob(f"{case}_2*.csv"),
                          key=lambda p: p.stat().st_mtime, reverse=True)
    if not predict_csvs:
        raise FileNotFoundError(
            f"predict CSV 없음 — {results_cases_dir}/{case}_2*.csv")
    predict_csv = predict_csvs[0]

    # 동일 timestamp 의 spline CSV 검색
    stamp = predict_csv.stem.replace(f"{case}_", "")   # e.g. "20260520_120000"
    spline_csv = results_cases_dir / f"{case}_spline_{stamp}.csv"
    if not spline_csv.exists():
        # 가장 최근 spline 으로 fallback
        spline_csvs = sorted(results_cases_dir.glob(f"{case}_spline_*.csv"),
                             key=lambda p: p.stat().st_mtime, reverse=True)
        if not spline_csvs:
            raise FileNotFoundError(
                f"spline CSV 없음 — {results_cases_dir}/{case}_spline_*.csv")
        spline_csv = spline_csvs[0]
        print(f"[warn] 동일 timestamp spline CSV 없음 — 최신 fallback: {spline_csv.name}")

    return predict_csv, spline_csv


def load_and_merge(predict_csv: Path, spline_csv: Path) -> pd.DataFrame:
    """t_us 기반 inner join.

    옵션 C tick-aligned 덕분에 *완전 일치 row 수* 보장.
    """
    df_p = pd.read_csv(predict_csv)
    df_s = pd.read_csv(spline_csv)
    df   = df_p.merge(df_s, on='t_us', how='inner', suffixes=('', '_s'))
    if len(df) == 0:
        raise RuntimeError(
            f"merge 결과 0 row — t_us 불일치? "
            f"predict={len(df_p)}, spline={len(df_s)}")
    print(f"[info] merge: predict={len(df_p)}, spline={len(df_s)}, joined={len(df)}")
    return df


def detect_stimulus_row(df: pd.DataFrame) -> int:
    """compare_single_input.py 의 로직 단순화.

    1. CSV 시작 + MIN_SUSTAIN_S 이후의 row 인덱스 찾음
    2. airspeed >= AIRSPEED_THRESHOLD 도달 첫 row 검출
    3. sp_a 변화 첫 row (자극 시작) 검출
    4. predict_timer 5 row (100ms) 이동 후 안정화된 row 반환
    """
    t_base = df['t_us'].iloc[0] * 1e-6
    min_t  = t_base + MIN_SUSTAIN_S

    # MIN_SUSTAIN_S 이후 + airspeed 도달 + sp_a 변화 동시 조건
    cond = (
        (df['t_us'] * 1e-6 >= min_t) &
        (df['true_airspeed'] >= AIRSPEED_THRESHOLD) &
        (df['sp_a'].abs() > 0.01)
    )
    if not cond.any():
        raise RuntimeError(
            f"자극 시점 검출 실패 — airspeed 도달 안 했거나 sp_a 변화 없음")
    raw_idx = int(cond.idxmax())

    # predict_timer (10Hz, 100ms) 안정화 — 5 tick (= 100ms @ 50Hz) 이동
    stable_idx = min(raw_idx + 5, len(df) - 1)
    print(f"[info] 자극 시점 row index: raw={raw_idx}, stable (+5)={stable_idx}, "
          f"t={df['t_us'].iloc[stable_idx] * 1e-6:.3f}s")
    return stable_idx


# ────────────────────────────────────────────────────────────
# 2. 46점 추출 (predict / spline 둘 다 NED)
# ────────────────────────────────────────────────────────────

def extract_predict_46(df_row: pd.Series) -> dict:
    """predict CSV 의 p_*_k 컬럼 (k=0..45) → NED 6 컬럼.

    PredictState: p_n, p_e, h (positive up), V, psi, h_dot, phi.
    NED 변환:
      pn = p_n
      pe = p_e
      pd = -h
      vn = V·cos(γ)·cos(ψ)    where γ = asin(h_dot / V)
      ve = V·cos(γ)·sin(ψ)
      vd = -h_dot
    """
    pn, pe, pd = [], [], []
    vn, ve, vd = [], [], []
    for k in range(N_PREDICT):
        p_n   = df_row[f'p_pn_{k}']
        p_e   = df_row[f'p_pe_{k}']
        h     = df_row[f'p_h_{k}']
        V     = df_row[f'p_V_{k}']
        psi   = df_row[f'p_psi_{k}']
        h_dot = df_row[f'p_hdot_{k}']

        pn.append(p_n)
        pe.append(p_e)
        pd.append(-h)

        # Beard-McLain γ
        sin_gamma = h_dot / V if V > 1e-3 else 0.0
        sin_gamma = max(-1.0, min(1.0, sin_gamma))   # clamp
        cos_gamma = np.sqrt(max(0.0, 1.0 - sin_gamma**2))
        vn.append(V * cos_gamma * np.cos(psi))
        ve.append(V * cos_gamma * np.sin(psi))
        vd.append(-h_dot)
    return dict(pn=np.array(pn), pe=np.array(pe), pd=np.array(pd),
                vn=np.array(vn), ve=np.array(ve), vd=np.array(vd))


def extract_spline_46(df_row: pd.Series) -> dict:
    """spline CSV 의 k*_*_spline 컬럼 (k=0..45) → NED 6 컬럼.

    spline 결과는 이미 NED (TrajectoryLogger 가 변환 후 dump).
    """
    pn, pe, pd = [], [], []
    vn, ve, vd = [], [], []
    for k in range(N_PREDICT):
        pn.append(df_row[f'k{k}_pn_spline'])
        pe.append(df_row[f'k{k}_pe_spline'])
        pd.append(df_row[f'k{k}_pd_spline'])
        vn.append(df_row[f'k{k}_vn_spline'])
        ve.append(df_row[f'k{k}_ve_spline'])
        vd.append(df_row[f'k{k}_vd_spline'])
    return dict(pn=np.array(pn), pe=np.array(pe), pd=np.array(pd),
                vn=np.array(vn), ve=np.array(ve), vd=np.array(vd))


# ────────────────────────────────────────────────────────────
# 3. 8 panel PNG 작성
# ────────────────────────────────────────────────────────────

def plot_8panel(t: np.ndarray, pred: dict, spline: dict,
                case: str, output_path: Path):
    """8 panel grid:
      [0,0] pn(t)     [0,1] pe(t)     [0,2] pd(t)
      [1,0] vn(t)     [1,1] ve(t)     [1,2] vd(t)
      [2,0] ‖Δp‖(t)   [2,1] ‖Δv‖(t)   [2,2] (legend / table)
    """
    fig, axes = plt.subplots(3, 3, figsize=(15, 10))
    fig.suptitle(f"{case} — spline reconstruct vs RK4 predict (4.5s window)",
                 fontsize=14, fontweight='bold')

    panels = [
        ('pn', 'p_n [m]'),
        ('pe', 'p_e [m]'),
        ('pd', 'p_d [m] (NED, +down)'),
        ('vn', 'v_n [m/s]'),
        ('ve', 'v_e [m/s]'),
        ('vd', 'v_d [m/s] (NED, +down)'),
    ]
    for i, (key, ylabel) in enumerate(panels):
        ax = axes[i // 3, i % 3]
        ax.plot(t, pred[key],   'r--', label='predicted (RK4)',     linewidth=1.5)
        ax.plot(t, spline[key], 'b-',  label='reconstructed (spline)', linewidth=1.2)
        ax.set_xlabel('t [s]')
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8, loc='best')

    # ── 위치 오차 norm ──
    dp = np.array([pred['pn'] - spline['pn'],
                   pred['pe'] - spline['pe'],
                   pred['pd'] - spline['pd']])
    err_p = np.linalg.norm(dp, axis=0)
    ax_p = axes[2, 0]
    ax_p.plot(t, err_p, 'k-', linewidth=1.5)
    ax_p.set_xlabel('t [s]')
    ax_p.set_ylabel('‖p_pred − p_spline‖ [m]')
    ax_p.set_title('3D position error norm')
    ax_p.grid(True, alpha=0.3)

    # ── 속도 오차 norm ──
    dv = np.array([pred['vn'] - spline['vn'],
                   pred['ve'] - spline['ve'],
                   pred['vd'] - spline['vd']])
    err_v = np.linalg.norm(dv, axis=0)
    ax_v = axes[2, 1]
    ax_v.plot(t, err_v, 'k-', linewidth=1.5)
    ax_v.set_xlabel('t [s]')
    ax_v.set_ylabel('‖v_pred − v_spline‖ [m/s]')
    ax_v.set_title('3D velocity error norm')
    ax_v.grid(True, alpha=0.3)

    # ── 통계 표 ──
    ax_t = axes[2, 2]
    ax_t.axis('off')
    sample_ts = [0.0, 1.5, 3.0, 4.5]
    sample_idx = [int(round(s * PREDICT_RATE_HZ)) for s in sample_ts]
    stats_text = (
        f"$\\bf{{RMS\\ errors\\ (over\\ 4.5s)}}$\n"
        f"  pos: {np.sqrt(np.mean(err_p**2)):.4f} m\n"
        f"  vel: {np.sqrt(np.mean(err_v**2)):.4f} m/s\n\n"
        f"$\\bf{{Max\\ errors}}$\n"
        f"  pos: {err_p.max():.4f} m (at t={t[err_p.argmax()]:.2f}s)\n"
        f"  vel: {err_v.max():.4f} m/s (at t={t[err_v.argmax()]:.2f}s)\n\n"
        f"$\\bf{{Errors\\ at\\ key\\ samples}}$\n"
        f"  t=0.0s: pos={err_p[sample_idx[0]]:.4f}m, vel={err_v[sample_idx[0]]:.4f}m/s\n"
        f"  t=1.5s: pos={err_p[sample_idx[1]]:.4f}m, vel={err_v[sample_idx[1]]:.4f}m/s\n"
        f"  t=3.0s: pos={err_p[sample_idx[2]]:.4f}m, vel={err_v[sample_idx[2]]:.4f}m/s\n"
        f"  t=4.5s: pos={err_p[sample_idx[3]]:.4f}m, vel={err_v[sample_idx[3]]:.4f}m/s\n\n"
        f"$\\bf{{Expected}}$: errors at\n"
        f"  t=0.0, 4.5s ≈ 0 (clamp)\n"
        f"  t=1.5, 3.0s pos ≈ 0 (knot)\n"
    )
    ax_t.text(0.0, 1.0, stats_text, fontfamily='monospace', fontsize=9,
              verticalalignment='top', transform=ax_t.transAxes)

    plt.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(output_path, dpi=120, bbox_inches='tight')
    plt.close(fig)
    print(f"[info] PNG 저장: {output_path}")


# ────────────────────────────────────────────────────────────
# 3b. XY + velocity error 2 panel (★ single_input/*_velocity.png 스타일)
# ────────────────────────────────────────────────────────────

def plot_xy_velocity(t: np.ndarray, pred: dict, spline: dict,
                     case: str, output_path):
    """Top-down XY trajectory (with velocity arrows) + velocity error 2 panel.

    기존 results/single_input/{case}_velocity.png 와 동일한 시각화 형식.
    단 *measured (PX4 실제)* 대신 *reconstructed (spline)* 로 비교 대상 변경.

    왼쪽:  East-North trajectory + velocity arrows (0.5s 간격)
    오른쪽: |Δv| (vector magnitude, 검정 solid) + vn/ve/vd component error (dashed)
            통계 (mean, std, max, end) 를 title 에 표시.
    """
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle(
        f"{case} — spline reconstruct vs RK4 predict\n"
        f"XY trajectory (with velocity arrows) | velocity error",
        fontsize=12, fontweight='bold')

    # ── 왼쪽: Top-down XY (East = pe, North = pn) ──
    ax_xy = axes[0]
    ax_xy.plot(pred['pe'],   pred['pn'],   'r--', label='predicted (RK4)',     linewidth=1.5)
    ax_xy.plot(spline['pe'], spline['pn'], 'b-',  label='reconstructed (spline)', linewidth=1.2)

    # 시작 / 종료 marker
    ax_xy.plot(pred['pe'][0],  pred['pn'][0],  'ko', markersize=8,  label='start')
    ax_xy.plot(pred['pe'][-1], pred['pn'][-1], 'r^', markersize=10, label='pred end')
    ax_xy.plot(spline['pe'][-1], spline['pn'][-1], 'bs', markersize=10, label='spline end')

    # ── velocity arrows (0.5s 간격 = index 5 마다) ──
    arrow_indices = list(range(0, len(t), 5))
    # arrow scale 자동: trajectory size 기반.
    # (pred 경로 polygon bbox 의 *짧은 변* 의 1/15 정도 — 화살표가 보이지만 path 안 가림)
    pe_range = max(1e-3, pred['pe'].max() - pred['pe'].min())
    pn_range = max(1e-3, pred['pn'].max() - pred['pn'].min())
    diag     = np.sqrt(pe_range**2 + pn_range**2)
    # vel m/s 를 화살표 길이 m 로 변환 — diag 의 1/10 / 평균 속도
    mean_v   = np.sqrt(pred['vn']**2 + pred['ve']**2).mean()
    arrow_scale_factor = (diag / 12.0) / max(mean_v, 1e-3)

    for k in arrow_indices:
        # pred 화살표 (빨강)
        ax_xy.arrow(pred['pe'][k], pred['pn'][k],
                    pred['ve'][k] * arrow_scale_factor,
                    pred['vn'][k] * arrow_scale_factor,
                    color='red', alpha=0.55,
                    head_width=diag*0.012, head_length=diag*0.018,
                    length_includes_head=True, linewidth=0.8)
        # spline 화살표 (파랑)
        ax_xy.arrow(spline['pe'][k], spline['pn'][k],
                    spline['ve'][k] * arrow_scale_factor,
                    spline['vn'][k] * arrow_scale_factor,
                    color='blue', alpha=0.55,
                    head_width=diag*0.012, head_length=diag*0.018,
                    length_includes_head=True, linewidth=0.8)

    ax_xy.set_xlabel('East [m]')
    ax_xy.set_ylabel('North [m]')
    ax_xy.set_title('Top-down XY trajectory  (arrows = velocity vectors, 0.5s 간격)')
    ax_xy.legend(loc='best', fontsize=8)
    ax_xy.grid(True, alpha=0.3)
    ax_xy.set_aspect('equal', adjustable='datalim')

    # ── 오른쪽: Velocity error (|Δv| + 3 축 성분) ──
    ax_ev = axes[1]
    err_vn = pred['vn'] - spline['vn']
    err_ve = pred['ve'] - spline['ve']
    err_vd = pred['vd'] - spline['vd']
    err_v_mag = np.sqrt(err_vn**2 + err_ve**2 + err_vd**2)

    ax_ev.plot(t, err_v_mag, 'k-',  linewidth=1.8,
               label='|Δv| (vector magnitude)')
    ax_ev.plot(t, err_vn,    'b--', linewidth=1.0,
               label='vn err (North, pred − spline)')
    ax_ev.plot(t, err_ve,    color='orange', linestyle='--', linewidth=1.0,
               label='ve err (East, pred − spline)')
    ax_ev.plot(t, err_vd,    'g--', linewidth=1.0,
               label='vd err (Down, pred − spline)')

    ax_ev.axhline(0, color='k', linewidth=0.5)
    ax_ev.set_xlabel('t [s]')
    ax_ev.set_ylabel('velocity error [m/s]')

    # 통계 (mean, std, max, end) — 기존 single_input PNG 와 동일 형식
    mean_err = err_v_mag.mean()
    std_err  = err_v_mag.std()
    max_err  = err_v_mag.max()
    end_err  = err_v_mag[-1]
    ax_ev.set_title(
        f'Velocity error  |Δv|: mean={mean_err:.4f}, std={std_err:.4f}, '
        f'max={max_err:.4f}, end={end_err:.4f}  [m/s]')
    ax_ev.legend(loc='best', fontsize=8)
    ax_ev.grid(True, alpha=0.3)

    plt.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(output_path, dpi=120, bbox_inches='tight')
    plt.close(fig)
    print(f"[info] XY+velocity PNG 저장: {output_path}")


# ────────────────────────────────────────────────────────────
# 4. Main
# ────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case',
                        default='R15P',
                        help='case ID (default: R15P)')
    parser.add_argument('--results-dir',
                        default='/home/leedonghyuck/ros2_ws/results',
                        type=Path,
                        help='results 루트 (default: ~/ros2_ws/results)')
    parser.add_argument('--output-dir',
                        default=None, type=Path,
                        help='PNG 출력 디렉터리 (default: results-dir/reconstruct_spline)')
    args = parser.parse_args()

    results_dir = args.results_dir
    cases_dir   = results_dir / 'cases'
    output_dir  = args.output_dir or (results_dir / 'reconstruct_spline')

    # CSV 짝 검색
    try:
        predict_csv, spline_csv = find_latest_csv_pair(cases_dir, args.case)
    except FileNotFoundError as e:
        print(f"[error] {e}", file=sys.stderr)
        sys.exit(1)
    print(f"[info] predict CSV: {predict_csv.name}")
    print(f"[info] spline  CSV: {spline_csv.name}")

    # merge + 자극 시점 검출
    df = load_and_merge(predict_csv, spline_csv)
    try:
        stim_idx = detect_stimulus_row(df)
    except RuntimeError as e:
        print(f"[error] {e}", file=sys.stderr)
        sys.exit(2)

    row = df.iloc[stim_idx]

    # 46점 추출
    pred   = extract_predict_46(row)
    spline = extract_spline_46(row)

    # spline 의 NaN 체크 — predict 호출 전 row 가 잡힌 경우 NaN 가능
    if np.isnan(spline['pn']).any():
        print(f"[error] spline 데이터에 NaN 포함 — 자극 시점이 predict 갱신 전?",
              file=sys.stderr)
        sys.exit(3)

    # t 축 = 0.0, 0.1, ..., 4.5
    t = np.arange(N_PREDICT) / PREDICT_RATE_HZ

    # ── PNG 생성 (두 종류) ──
    # 1) 8 panel — pn/pe/pd/vn/ve/vd 시계열 + 오차 norm + 통계 표
    output_path_8 = output_dir / f"{args.case}_spline_compare.png"
    plot_8panel(t, pred, spline, args.case, output_path_8)

    # 2) 2 panel — Top-down XY trajectory (velocity arrows) + velocity error
    output_path_xy = output_dir / f"{args.case}_spline_xy_velocity.png"
    plot_xy_velocity(t, pred, spline, args.case, output_path_xy)


if __name__ == '__main__':
    main()
