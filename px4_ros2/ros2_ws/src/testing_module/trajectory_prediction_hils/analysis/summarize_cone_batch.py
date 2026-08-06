#!/usr/bin/env python3
"""Aggregate per-run offline cone summaries into batch tables and plots."""

import argparse
import csv
import json
import re
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


RUN_PATTERN = re.compile(r"^(?P<case>.+)_r(?P<repetition>[0-9]+)$")
FIELDS = (
    "run_id", "case_id", "repetition", "analysis_ok", "missing_required_topics",
    "ground_truth_sample_time_valid", "common_contract_failures",
    "invalid_cones", "non_zoh_cones",
    "complete_ground_truth_cones", "partial_horizon_cones",
    "causal_partial_horizon_cones", "ground_truth_partial_horizon_cones",
    "causally_censored_points", "analyzed_points", "covariance_failures",
    "cone_rate_hz", "empirical_coverage_all_horizons",
    "empirical_coverage_at_4_5_s", "mean_position_error_m", "max_position_error_m",
)


def nullable_float(value):
    return None if value is None else float(value)


def read_runs(raw_root: Path):
    rows = []
    for summary_path in sorted(raw_root.glob("*/summary.json")):
        run_id = summary_path.parent.name
        match = RUN_PATTERN.match(run_id)
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        missing = summary.get("missing_required_topics", [])
        complete = int(summary.get("complete_ground_truth_cones", 0))
        covariance_failures = int(summary.get("covariance_failures", 0))
        invalid_cones = int(summary.get("invalid_cones", 0))
        non_zoh_cones = int(summary.get("non_zoh_cones", 0))
        sample_time_valid = bool(summary.get("ground_truth_sample_time_valid", False))
        common_contract_failures = int(summary.get("common_contract_failures", 0))
        rows.append({
            "run_id": run_id,
            "case_id": match.group("case") if match else run_id,
            "repetition": int(match.group("repetition")) if match else 0,
            "analysis_ok": int(
                not missing and sample_time_valid and complete > 0
                and covariance_failures == 0 and invalid_cones == 0
                and non_zoh_cones == 0 and common_contract_failures == 0),
            "missing_required_topics": ";".join(missing),
            "ground_truth_sample_time_valid": int(sample_time_valid),
            "common_contract_failures": common_contract_failures,
            "invalid_cones": invalid_cones,
            "non_zoh_cones": non_zoh_cones,
            "complete_ground_truth_cones": complete,
            "partial_horizon_cones": int(summary.get("partial_horizon_cones", 0)),
            "causal_partial_horizon_cones": int(
                summary.get("causal_partial_horizon_cones", 0)),
            "ground_truth_partial_horizon_cones": int(
                summary.get("ground_truth_partial_horizon_cones", 0)),
            "causally_censored_points": int(
                summary.get("causally_censored_points", 0)),
            "analyzed_points": int(summary.get("analyzed_points", 0)),
            "covariance_failures": covariance_failures,
            "cone_rate_hz": nullable_float(summary.get("cone_rate_hz")),
            "empirical_coverage_all_horizons": nullable_float(
                summary.get("empirical_coverage_all_horizons")),
            "empirical_coverage_at_4_5_s": nullable_float(
                summary.get("empirical_coverage_at_4_5_s")),
            "mean_position_error_m": nullable_float(summary.get("mean_position_error_m")),
            "max_position_error_m": nullable_float(summary.get("max_position_error_m")),
        })
    return rows


def write_case_table(rows, output_dir: Path):
    with (output_dir / "cases.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def grouped_metric(rows, field):
    grouped = defaultdict(list)
    for row in rows:
        value = row[field]
        if value is not None:
            grouped[row["case_id"]].append(value)
    labels = sorted(grouped)
    means = np.asarray([np.mean(grouped[label]) for label in labels])
    deviations = np.asarray([np.std(grouped[label]) for label in labels])
    return labels, means, deviations


def save_metric_plot(rows, output_dir: Path, field: str, filename: str,
                     ylabel: str, target=None):
    labels, means, deviations = grouped_metric(rows, field)
    if not labels:
        return False
    width = max(8.0, 0.48 * len(labels))
    fig, axis = plt.subplots(figsize=(width, 4.8))
    positions = np.arange(len(labels))
    axis.bar(positions, means, yerr=deviations, capsize=3, color="tab:blue", alpha=0.8)
    if target is not None:
        axis.axhline(target, color="tab:red", linestyle="--", label=f"target {target:g}")
        axis.legend()
    axis.set_xticks(positions, labels, rotation=45, ha="right")
    axis.set_ylabel(ylabel)
    axis.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(output_dir / filename, dpi=160)
    plt.close(fig)
    return True


def mean_or_none(values):
    values = [value for value in values if value is not None]
    return float(np.mean(values)) if values else None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--raw-root", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--expected-runs", type=int)
    args = parser.parse_args()

    raw_root = Path(args.raw_root)
    output_dir = Path(args.output)
    rows = read_runs(raw_root)
    if not rows:
        print(f"ERROR: no per-run summary.json found below {raw_root}")
        return 2
    output_dir.mkdir(parents=True, exist_ok=True)
    write_case_table(rows, output_dir)

    plots = []
    if save_metric_plot(
            rows, output_dir, "empirical_coverage_at_4_5_s",
            "endpoint_coverage_by_case.png", "4.5 s empirical coverage", 0.95):
        plots.append("endpoint_coverage_by_case.png")
    if save_metric_plot(
            rows, output_dir, "mean_position_error_m",
            "mean_position_error_by_case.png", "mean position error [m]"):
        plots.append("mean_position_error_by_case.png")

    expected = args.expected_runs if args.expected_runs is not None else len(rows)
    summary = {
        "raw_root": str(raw_root.resolve()),
        "expected_run_count": expected,
        "analyzed_run_count": len(rows),
        "structurally_valid_run_count": sum(row["analysis_ok"] for row in rows),
        "missing_analysis_run_count": max(expected - len(rows), 0),
        "case_count": len({row["case_id"] for row in rows}),
        "mean_empirical_coverage_all_horizons": mean_or_none(
            [row["empirical_coverage_all_horizons"] for row in rows]),
        "mean_empirical_coverage_at_4_5_s": mean_or_none(
            [row["empirical_coverage_at_4_5_s"] for row in rows]),
        "total_covariance_failures": sum(row["covariance_failures"] for row in rows),
        "total_invalid_cones": sum(row["invalid_cones"] for row in rows),
        "total_non_zoh_cones": sum(row["non_zoh_cones"] for row in rows),
        "total_common_contract_failures": sum(
            row["common_contract_failures"] for row in rows),
        "formal_coverage_pass": None,
        "formal_coverage_note": (
            "Coverage is reported for calibration; acceptance limits are not gated yet."),
        "plots": plots,
    }
    (output_dir / "batch_summary.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0 if summary["structurally_valid_run_count"] == len(rows) \
        and len(rows) == expected else 1


if __name__ == "__main__":
    raise SystemExit(main())
