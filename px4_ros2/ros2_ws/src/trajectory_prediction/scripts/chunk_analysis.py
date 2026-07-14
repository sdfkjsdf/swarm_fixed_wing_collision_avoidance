#!/usr/bin/env python3
"""
chunk_analysis.py
=================
trajectory_prediction CSV (rolling-mode 50Hz 기록) 에서
chunk-mode (4.5초 간격 reset) 패턴을 사후추출해서 모델 정확도 분석.

추출 원리:
  rolling-mode 가 100ms 마다 매번 새 4.5초 예측을 만들어 모두 CSV 에 기록.
  여기서 4.5초 간격으로만 행을 골라보면 그게 chunk-mode 와 동일.

  chunk i 의 시작 행 = i × 230 (= N_PREDICT × TICKS_PER_PREDICT = 46 × 5)
  chunk i 의 예측    = 그 행의 p_*_0 ~ p_*_45  (46점, 0.1s 간격, endpoint-inclusive)
  chunk i 의 실측    = 행 (i×230 + k×5) 의 측정값  (k = 0..45)

출력:
  /tmp/chunk_error_curves.png   채널별 오차 곡선 (XY/h/V/psi × N chunk)
  /tmp/chunk_trajectories.png   XY top-view 궤적 overlay (chunk 별 subplot)
  RMSE 표 (stdout)

사용:
  ./chunk_analysis.py                       # /tmp/trajectory_*.csv 최신 자동
  ./chunk_analysis.py /path/to/foo.csv
  ./chunk_analysis.py --out-dir /tmp/analy  # plot 저장 디렉토리 변경
"""

import argparse
import glob
import os
import sys

import matplotlib
matplotlib.use('Agg')                # GUI 없는 환경 호환
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


# ─── 매크로 (replay_params.yaml + TrajectoryLogger.hpp 와 정합 ) ───
# ★ 2026-05-13 endpoint-inclusive 컨벤션:
#   N_PREDICT = round(endpoint × rate) + 1 (kPredictHorizon 과 동일 정의)
LOG_RATE_HZ               = 50               # CSV 기록 주기
PREDICT_RATE_HZ           = 10               # predict 내부 dt = 1/rate
PREDICT_HORIZON_ENDPOINT_S = 4.5              # 마지막 점의 시각 (endpoint-inclusive)

TICKS_PER_PREDICT  = LOG_RATE_HZ // PREDICT_RATE_HZ          # 5
N_PREDICT          = int(round(PREDICT_HORIZON_ENDPOINT_S * PREDICT_RATE_HZ)) + 1   # 46
CHUNK_TICKS        = N_PREDICT * TICKS_PER_PREDICT             # 230


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('csv', nargs='?',
                   help='CSV 경로 (생략 시 /tmp/trajectory_*.csv 최신)')
    p.add_argument('--out-dir', default='/tmp',
                   help='plot 저장 디렉토리 (기본 /tmp)')
    p.add_argument('--max-chunks', type=int, default=8,
                   help='plot 에 표시할 chunk 개수 (기본 8 = 35s 시퀀스 영역, '
                        '0 또는 음수 = 전부)')
    return p.parse_args()


def find_latest_csv():
    files = sorted(glob.glob('/tmp/trajectory_*.csv'), key=os.path.getmtime)
    if not files:
        sys.exit('ERROR: /tmp/trajectory_*.csv 없음. trajectory_replay_node 실행 후 시도.')
    return files[-1]


def angle_wrap(a):
    """ wrap to [-pi, pi] (벡터 OK) """
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def chunk_compare(df, chunk_idx):
    """
    chunk_idx 번째 4.5초 chunk 의 예측 / 실측 추출.
    return: dict 또는 None (데이터 부족 시)
    """
    row_start = chunk_idx * CHUNK_TICKS
    last_row_needed = row_start + (N_PREDICT - 1) * TICKS_PER_PREDICT
    if last_row_needed >= len(df):
        return None

    head = df.iloc[row_start]
    pred_pn  = np.array([head[f'p_pn_{k}']   for k in range(N_PREDICT)], dtype=float)
    pred_pe  = np.array([head[f'p_pe_{k}']   for k in range(N_PREDICT)], dtype=float)
    pred_h   = np.array([head[f'p_h_{k}']    for k in range(N_PREDICT)], dtype=float)
    pred_V   = np.array([head[f'p_V_{k}']    for k in range(N_PREDICT)], dtype=float)
    pred_psi = np.array([head[f'p_psi_{k}']  for k in range(N_PREDICT)], dtype=float)
    # ★ PATCH 호환: 새 CSV 는 p_phi_k, 구 CSV 는 p_alat_k. phi 가 있으면 사용.
    if f'p_phi_0' in df.columns:
        pred_phi = np.array([head[f'p_phi_{k}'] for k in range(N_PREDICT)], dtype=float)
    else:
        pred_phi = np.zeros(N_PREDICT)   # 구 CSV 에 없는 컬럼은 0 (분석 영향 없음)

    rows = [row_start + k * TICKS_PER_PREDICT for k in range(N_PREDICT)]
    sub  = df.iloc[rows]
    meas_pn  = sub['x'].to_numpy(dtype=float)
    meas_pe  = sub['y'].to_numpy(dtype=float)
    meas_h   = -sub['z'].to_numpy(dtype=float)              # NED z (down) → altitude (up)
    meas_V   = sub['true_airspeed'].to_numpy(dtype=float)
    meas_psi = sub['yaw'].to_numpy(dtype=float)
    # ★ A-1: roll 컬럼 (구 CSV 호환). 없으면 NaN 으로 채워서 chunk_compare 가 NaN-skip 처리.
    if 'roll' in df.columns:
        meas_phi = sub['roll'].to_numpy(dtype=float)
    else:
        meas_phi = np.full(N_PREDICT, np.nan)

    return dict(
        chunk_idx = chunk_idx,
        t_start   = chunk_idx * PREDICT_HORIZON_ENDPOINT_S,
        t_local   = np.arange(N_PREDICT) * (1.0 / PREDICT_RATE_HZ),
        pred_pn = pred_pn, pred_pe = pred_pe, pred_h = pred_h,
        pred_V  = pred_V,  pred_psi = pred_psi, pred_phi = pred_phi,
        meas_pn = meas_pn, meas_pe = meas_pe, meas_h = meas_h,
        meas_V  = meas_V,  meas_psi = meas_psi, meas_phi = meas_phi,
    )


def has_nan(c):
    """ chunk 안에 NaN 있는지 (Replay 활성화 직전 첫 행 같은 경우).
        ★ A-1: meas_phi 는 구 CSV 호환을 위해 NaN 일 수 있음 → has_nan 판정에서 제외.
                phi 채널은 별도로 valid_phi 로 처리. """
    arrays = [c['pred_pn'], c['pred_pe'], c['pred_h'], c['pred_V'], c['pred_psi'],
              c['meas_pn'], c['meas_pe'], c['meas_h'], c['meas_V'], c['meas_psi']]
    return any(np.any(np.isnan(a)) for a in arrays)


def has_phi(c):
    """ chunk 에 phi 비교 가능한 데이터가 있는지 (예측 pred_phi 와 실측 meas_phi 모두 finite). """
    return (not np.any(np.isnan(c['pred_phi']))) and (not np.any(np.isnan(c['meas_phi'])))


def print_rmse_table(chunks):
    print()
    print('=== Chunk-mode 예측 정확도 (chunk 시작점에서 measured 로 reset, 4.5초 free run) ===')
    print()
    h = ('chunk', 't_start', 'XY end', 'XY mean', 'h end', 'h mean',
         'V end', 'V mean', 'psi end', 'psi mean', 'phi end', 'phi mean')   # ★ A-1: phi 채널
    print(f'{h[0]:>5} {h[1]:>9} {h[2]:>9} {h[3]:>9} {h[4]:>8} {h[5]:>8} '
          f'{h[6]:>7} {h[7]:>7} {h[8]:>9} {h[9]:>9} {h[10]:>9} {h[11]:>9}')
    print('-' * 110)

    for c in chunks:
        err_xy  = np.sqrt((c['pred_pn'] - c['meas_pn'])**2 +
                          (c['pred_pe'] - c['meas_pe'])**2)
        err_h   = np.abs(c['pred_h']   - c['meas_h'])
        err_V   = np.abs(c['pred_V']   - c['meas_V'])
        err_psi = np.abs(angle_wrap(c['pred_psi'] - c['meas_psi']))

        # ★ A-1: phi 채널 (구 CSV 호환 — meas_phi NaN 이면 N/A 출력)
        if has_phi(c):
            err_phi  = np.abs(angle_wrap(c['pred_phi'] - c['meas_phi']))
            phi_end  = f"{np.degrees(err_phi[-1]):>8.2f}°"
            phi_mean = f"{np.degrees(err_phi.mean()):>8.2f}°"
        else:
            phi_end  = f"{'N/A':>9}"
            phi_mean = f"{'N/A':>9}"

        print(f"{c['chunk_idx']:>5} {c['t_start']:>8.1f}s "
              f"{err_xy[-1]:>8.2f}m {err_xy.mean():>8.2f}m "
              f"{err_h[-1]:>7.3f}m {err_h.mean():>7.3f}m "
              f"{err_V[-1]:>6.3f} {err_V.mean():>6.3f} "
              f"{np.degrees(err_psi[-1]):>8.2f}° {np.degrees(err_psi.mean()):>8.2f}° "
              f"{phi_end} {phi_mean}")


def plot_error_curves(chunks, out_path):
    # ★ A-1: 2×3 layout — XY/h/V/psi/phi (마지막 subplot 비어있을 수 있음, 5채널)
    fig, axes = plt.subplots(2, 3, figsize=(18, 9))
    fig.suptitle(f'Chunk-mode prediction error (time vs error)\n'
                 f'{len(chunks)} chunks, horizon = {PREDICT_HORIZON_ENDPOINT_S}s, '
                 f'each chunk reset to measured at start', fontsize=12)

    cmap = plt.get_cmap('tab10')
    for i, c in enumerate(chunks):
        err_xy  = np.sqrt((c['pred_pn'] - c['meas_pn'])**2 +
                          (c['pred_pe'] - c['meas_pe'])**2)
        err_h   = np.abs(c['pred_h']   - c['meas_h'])
        err_V   = np.abs(c['pred_V']   - c['meas_V'])
        err_psi = np.degrees(np.abs(angle_wrap(c['pred_psi'] - c['meas_psi'])))

        color = cmap(i % 10)
        label = f"chunk {c['chunk_idx']} (t={c['t_start']:.1f}s)"
        axes[0, 0].plot(c['t_local'], err_xy,  label=label, color=color, linewidth=2.0)
        axes[0, 1].plot(c['t_local'], err_h,   label=label, color=color, linewidth=2.0)
        axes[0, 2].plot(c['t_local'], err_V,   label=label, color=color, linewidth=2.0)
        axes[1, 0].plot(c['t_local'], err_psi, label=label, color=color, linewidth=2.0)

        # ★ A-1: phi channel (구 CSV 호환 — finite 데이터 있을 때만)
        if has_phi(c):
            err_phi = np.degrees(np.abs(angle_wrap(c['pred_phi'] - c['meas_phi'])))
            axes[1, 1].plot(c['t_local'], err_phi, label=label, color=color, linewidth=2.0)

    axes[0, 0].set(title='XY position error',  xlabel='time within chunk [s]', ylabel='[m]')
    axes[0, 1].set(title='Altitude error',     xlabel='time within chunk [s]', ylabel='[m]')
    axes[0, 2].set(title='Airspeed error',     xlabel='time within chunk [s]', ylabel='[m/s]')
    axes[1, 0].set(title='Heading (psi) error', xlabel='time within chunk [s]', ylabel='[deg]')
    axes[1, 1].set(title='Roll (phi) error  [A-1 added]', xlabel='time within chunk [s]', ylabel='[deg]')
    axes[1, 2].axis('off')   # 빈 subplot
    for ax in axes.flat[:5]:
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8, loc='upper left')

    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)
    print(f'\n[plot] {out_path}')


def plot_xy_trajectories(chunks, out_path):
    n = len(chunks)
    if n == 0:
        return
    cols = min(4, n)
    rows = (n + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(4 * cols, 4 * rows), squeeze=False)
    fig.suptitle('XY trajectories: predicted (red dashed) vs measured (green solid) — per chunk\n'
                 f'each chunk: model reset to measured at start, then free-run {PREDICT_HORIZON_ENDPOINT_S}s', fontsize=11)

    for idx, c in enumerate(chunks):
        ax = axes[idx // cols][idx % cols]
        # 측정: NED 의 East = y, North = x → x축은 East (y), y축은 North (x)
        ax.plot(c['meas_pe'], c['meas_pn'], 'g-',  linewidth=2.5, label='measured', alpha=0.7)
        ax.plot(c['pred_pe'], c['pred_pn'], 'r--', linewidth=1.5, label='predicted', alpha=0.9)
        ax.plot(c['meas_pe'][0],  c['meas_pn'][0],  'ko', markersize=8, label='start')
        ax.plot(c['meas_pe'][-1], c['meas_pn'][-1], 'gs', markersize=8, label='meas end')
        ax.plot(c['pred_pe'][-1], c['pred_pn'][-1], 'r^', markersize=8, label='pred end')

        ax.set_title(f"chunk {c['chunk_idx']}: t={c['t_start']:.1f}~"
                     f"{c['t_start'] + PREDICT_HORIZON_ENDPOINT_S:.1f}s")
        ax.set_xlabel('East (y) [m]')
        ax.set_ylabel('North (x) [m]')
        ax.axis('equal')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7)

    # 빈 subplot 숨김
    for idx in range(n, rows * cols):
        axes[idx // cols][idx % cols].axis('off')

    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)
    print(f'[plot] {out_path}')


def main():
    args = parse_args()
    csv_path = args.csv or find_latest_csv()
    print(f'[chunk_analysis] CSV: {csv_path}')

    df = pd.read_csv(csv_path)
    print(f'[chunk_analysis] {len(df)} rows × {len(df.columns)} columns')
    print(f'[chunk_analysis] CHUNK_TICKS={CHUNK_TICKS} (= {PREDICT_HORIZON_ENDPOINT_S}s × {LOG_RATE_HZ}Hz)')
    print(f'[chunk_analysis] N_PREDICT  ={N_PREDICT}  (= {PREDICT_HORIZON_ENDPOINT_S}s × {PREDICT_RATE_HZ}Hz)')

    n_chunks_max = len(df) // CHUNK_TICKS
    print(f'[chunk_analysis] chunks possible (전체/CHUNK_TICKS): {n_chunks_max}')

    chunks = []
    for i in range(n_chunks_max + 2):    # 약간 overshoot 했다가 끝에서 None 잡힘
        c = chunk_compare(df, i)
        if c is None:
            break
        if has_nan(c):
            print(f'[chunk_analysis] chunk {i} (t={c["t_start"]:.1f}s) NaN 포함 → skip')
            continue
        chunks.append(c)
    print(f'[chunk_analysis] valid chunks: {len(chunks)}')

    if not chunks:
        sys.exit('ERROR: 유효한 chunk 0 — CSV 가 너무 짧거나 NaN 만 가득.')

    print_rmse_table(chunks)

    if args.max_chunks > 0:
        chunks_for_plot = chunks[:args.max_chunks]
        print(f'\n[plot] 처음 {len(chunks_for_plot)} chunks 만 시각화 '
              f'(--max-chunks 0 으로 전체 표시 가능)')
    else:
        chunks_for_plot = chunks
        print(f'\n[plot] 전체 {len(chunks_for_plot)} chunks 시각화')

    plot_error_curves(chunks_for_plot,
                      os.path.join(args.out_dir, 'chunk_error_curves.png'))
    plot_xy_trajectories(chunks_for_plot,
                         os.path.join(args.out_dir, 'chunk_trajectories.png'))

    print(f'\n[chunk_analysis] 완료. 분석 {len(chunks)} chunks, '
          f'시각화 {len(chunks_for_plot)} chunks.')


if __name__ == '__main__':
    main()
