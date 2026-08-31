#!/usr/bin/env python3
"""Plot formation decisions and configured boundaries in the R-Vc plane.

The input CSV is the schema emitted by FormationDecisionCsvFormatter.  This
tool does not reclassify an encounter, so the C++ state machine remains the
single algorithmic source of truth.  It only overlays the exact limits logged
at each update, including entry/exit hysteresis boundaries.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Dict, List


REQUIRED_COLUMNS = {
    "time_s",
    "threat_id",
    "range_m",
    "closure_rate_mps",
    "new_state",
    "entry_closure_limit_mps",
    "exit_closure_limit_mps",
    "closure_lower_entry_mps",
    "closure_lower_exit_mps",
    "fdz_entry_limit_m",
    "fdz_exit_limit_m",
    "max_range_entry_m",
    "max_range_exit_m",
}


def load_rows(path: Path) -> List[Dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        missing = REQUIRED_COLUMNS.difference(reader.fieldnames or [])
        if missing:
            raise ValueError(f"missing CSV columns: {sorted(missing)}")
        return list(reader)


def numeric(row: Dict[str, str], key: str) -> float:
    return float(row[key])


def plot(rows: List[Dict[str, str]], output: Path, title: str) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError as error:
        raise RuntimeError(
            "matplotlib is required to render the formation calibration plot"
        ) from error
    if not rows:
        raise ValueError("input CSV contains no formation decision rows")

    figure, axis = plt.subplots(figsize=(11, 7))
    state_style = {
        "OUTSIDE_FORMATION": ("#d62728", "outside"),
        "FDZ": ("#2ca02c", "FDZ"),
        "STANDBY": ("#1f77b4", "standby"),
    }
    for state, (color, label) in state_style.items():
        selected = [row for row in rows if row["new_state"] == state]
        if selected:
            axis.scatter(
                [numeric(row, "range_m") for row in selected],
                [numeric(row, "closure_rate_mps") for row in selected],
                s=18,
                color=color,
                label=f"encounter: {label}",
                zorder=4,
            )

    ordered = sorted(rows, key=lambda row: numeric(row, "range_m"))
    ranges = [numeric(row, "range_m") for row in ordered]
    axis.plot(
        ranges,
        [numeric(row, "entry_closure_limit_mps") for row in ordered],
        color="#111111",
        linewidth=1.8,
        label="standby upper entry (logged)",
    )
    axis.plot(
        ranges,
        [numeric(row, "exit_closure_limit_mps") for row in ordered],
        color="#7f7f7f",
        linestyle="--",
        linewidth=1.8,
        label="standby upper exit (logged)",
    )

    first = rows[0]
    axis.axhline(
        numeric(first, "closure_lower_entry_mps"),
        color="#111111",
        linestyle=":",
        label="standby lower entry",
    )
    axis.axhline(
        numeric(first, "closure_lower_exit_mps"),
        color="#7f7f7f",
        linestyle=":",
        label="standby lower exit",
    )
    verticals = (
        ("fdz_entry_limit_m", "#2ca02c", "-", "FDZ entry"),
        ("fdz_exit_limit_m", "#98df8a", "--", "FDZ exit"),
        ("max_range_entry_m", "#1f77b4", "-", "max range entry"),
        ("max_range_exit_m", "#aec7e8", "--", "max range exit"),
    )
    for key, color, linestyle, label in verticals:
        axis.axvline(
            numeric(first, key), color=color, linestyle=linestyle, label=label
        )

    axis.set_xlabel("3-D slant range R [m]")
    axis.set_ylabel("closure Vc = -R_dot [m/s]")
    axis.set_title(title)
    axis.grid(True, alpha=0.3)
    axis.legend(loc="best", fontsize=8)
    figure.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=180)
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--title", default="Formation discrimination: range vs closure"
    )
    args = parser.parse_args()
    plot(load_rows(args.input), args.output, args.title)


if __name__ == "__main__":
    main()
