#!/usr/bin/env bash
# ════════════════════════════════════════════════════════════════════
# compare_b_h_grid.sh — B-2 grid 결과 비교
# ────────────────────────────────────────────────────────────────────
# B_H=0.0, 0.05, 0.1, 0.2 로 SITL 4번 돌린 후 각 CSV 를 chunk_analysis 로
# 분석하여 chunk 별 오차를 한 표로 비교.
#
# 전제: /tmp/trajectory_bH0p0_*.csv, bH0p05_*, bH0p1_*, bH0p2_* 4 개 CSV.
# ════════════════════════════════════════════════════════════════════

PY=/home/leedonghyuck/anaconda3/bin/python3
SCRIPT=/home/leedonghyuck/ros2_ws/src/trajectory_prediction/scripts/chunk_analysis.py

echo "═══════════════════════════════════════════════════════════════"
echo "  b_h grid 비교 (B-2)"
echo "═══════════════════════════════════════════════════════════════"

for B_H in 0p0 0p05 0p1 0p2; do
    CSV=$(ls -t /tmp/trajectory_bH${B_H}_*.csv 2>/dev/null | head -1)
    if [[ -z "${CSV}" ]]; then
        echo ""
        echo "[b_h=${B_H/p/.}] CSV 없음 — skip"
        continue
    fi
    echo ""
    echo "═══════════════════════════════════════════════════════════════"
    echo "  b_h = ${B_H/p/.}   (CSV: ${CSV})"
    echo "═══════════════════════════════════════════════════════════════"
    OUT_DIR=/tmp/bH_${B_H}
    mkdir -p "${OUT_DIR}"
    "${PY}" "${SCRIPT}" "${CSV}" --out-dir "${OUT_DIR}/" 2>&1 | \
        grep -E "^chunk |^[ ]+[0-9]+ +[0-9]+\." | head -10
done

echo ""
echo "═══════════════════════════════════════════════════════════════"
echo "  PNG 위치:"
echo "    b_h=0.0  : /tmp/bH_0p0/chunk_error_curves.png"
echo "    b_h=0.05 : /tmp/bH_0p05/"
echo "    b_h=0.1  : /tmp/bH_0p1/"
echo "    b_h=0.2  : /tmp/bH_0p2/"
echo "═══════════════════════════════════════════════════════════════"
