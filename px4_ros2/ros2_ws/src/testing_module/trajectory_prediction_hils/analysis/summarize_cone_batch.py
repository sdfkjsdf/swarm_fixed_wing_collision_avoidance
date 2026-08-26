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
    "propagation_test_contract_present", "propagation_test_contract_pass",
    "trim_gate_required", "trim_gate_pass", "trim_max_ground_speed_error_mps",
    "trim_max_abs_vertical_speed_mps", "trim_max_abs_roll_deg",
    "alignment_gate_required", "alignment_gate_pass",
    "complete_ground_truth_cones", "partial_horizon_cones",
    "causal_partial_horizon_cones", "ground_truth_partial_horizon_cones",
    "causally_censored_points", "analyzed_points", "covariance_failures",
    "cone_rate_hz", "empirical_coverage_all_horizons",
    "empirical_coverage_at_4_5_s", "causal_segment_trajectory_containment_rate",
    "full_4_5s_trajectory_containment_rate", "first_exit_horizon_median_s",
    "fusion_horizon_error_median_m", "delay_compensation_error_median_m",
    "horizon_zero_error_median_m", "propagation_error_4_5_median_m",
    "propagation_error_4_5_p95_m", "propagation_error_4_5_median_n_m",
    "propagation_error_4_5_median_e_m", "propagation_error_4_5_median_d_m",
    "mean_position_error_m", "max_position_error_m",
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
        alignment = summary.get("initial_alignment", {})
        alignment_gate = summary.get("alignment_gate", {})
        alignment_required = bool(alignment_gate.get("required", False))
        alignment_pass = bool(alignment_gate.get("pass", False))
        propagation_contract = summary.get("propagation_test_contract", {})
        propagation_contract_present = bool(
            propagation_contract.get("present", False))
        propagation_contract_pass = bool(
            propagation_contract.get("pass", False))
        trim_gate = summary.get("trim_gate", {})
        trim_required = bool(trim_gate.get("required", False))
        trim_pass = bool(trim_gate.get("pass", False))
        propagation_error = summary.get("propagation_error_at_4_5_s", {})
        propagation_error_vector = propagation_error.get("median_vector_ned_m")
        rows.append({
            "run_id": run_id,
            "case_id": match.group("case") if match else run_id,
            "repetition": int(match.group("repetition")) if match else 0,
            "analysis_ok": int(
                not missing and sample_time_valid and complete > 0
                and covariance_failures == 0 and invalid_cones == 0
                and non_zoh_cones == 0 and common_contract_failures == 0
                and (not alignment_required or alignment_pass)
                and (not propagation_contract_present
                     or propagation_contract_pass)
                and (not trim_required or trim_pass)),
            "missing_required_topics": ";".join(missing),
            "ground_truth_sample_time_valid": int(sample_time_valid),
            "common_contract_failures": common_contract_failures,
            "invalid_cones": invalid_cones,
            "non_zoh_cones": non_zoh_cones,
            "propagation_test_contract_present": int(
                propagation_contract_present),
            "propagation_test_contract_pass": (
                int(propagation_contract_pass)
                if propagation_contract_present else ""),
            "trim_gate_required": int(trim_required),
            "trim_gate_pass": int(trim_pass) if trim_required else "",
            "trim_max_ground_speed_error_mps": nullable_float(
                trim_gate.get(
                    "max_ground_speed_error_mps",
                    trim_gate.get("max_airspeed_error_mps"))),
            "trim_max_abs_vertical_speed_mps": nullable_float(
                trim_gate.get("max_abs_vertical_speed_mps")),
            "trim_max_abs_roll_deg": nullable_float(
                trim_gate.get("max_abs_roll_deg")),
            "alignment_gate_required": int(alignment_required),
            "alignment_gate_pass": (
                int(alignment_pass) if alignment_required else ""),
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
            "causal_segment_trajectory_containment_rate": nullable_float(
                summary.get("causal_segment_trajectory_containment_rate")),
            "full_4_5s_trajectory_containment_rate": nullable_float(
                summary.get("full_4_5s_trajectory_containment_rate")),
            "first_exit_horizon_median_s": nullable_float(
                summary.get("first_exit_horizon_median_s")),
            "fusion_horizon_error_median_m": nullable_float(
                alignment.get("fusion_horizon_ekf_error", {}).get("median_norm_m")),
            "delay_compensation_error_median_m": nullable_float(
                alignment.get("delay_compensation_error", {}).get("median_norm_m")),
            "horizon_zero_error_median_m": nullable_float(
                alignment.get("horizon_zero_error", {}).get("median_norm_m")),
            "propagation_error_4_5_median_m": nullable_float(
                propagation_error.get("median_norm_m")),
            "propagation_error_4_5_p95_m": nullable_float(
                propagation_error.get("percentile_95_norm_m")),
            "propagation_error_4_5_median_n_m": (
                nullable_float(propagation_error_vector[0])
                if propagation_error_vector is not None else None),
            "propagation_error_4_5_median_e_m": (
                nullable_float(propagation_error_vector[1])
                if propagation_error_vector is not None else None),
            "propagation_error_4_5_median_d_m": (
                nullable_float(propagation_error_vector[2])
                if propagation_error_vector is not None else None),
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


def save_propagation_component_plot(rows, output_dir: Path):
    labels = sorted({row["case_id"] for row in rows})
    if not labels:
        return False
    fields = (
        "propagation_error_4_5_median_n_m",
        "propagation_error_4_5_median_e_m",
        "propagation_error_4_5_median_d_m",
    )
    component_labels = ("North", "East", "Down")
    positions = np.arange(len(labels), dtype=np.float64)
    width = 0.24
    fig, axis = plt.subplots(figsize=(max(8.0, 0.6 * len(labels)), 4.8))
    for component, (field, component_label) in enumerate(
            zip(fields, component_labels)):
        values = []
        for case_id in labels:
            samples = [
                row[field] for row in rows
                if row["case_id"] == case_id and row[field] is not None
            ]
            values.append(float(np.mean(samples)) if samples else np.nan)
        axis.bar(
            positions + (component - 1) * width, values, width,
            label=component_label)
    axis.axhline(0.0, color="black", linewidth=0.8)
    axis.set_xticks(positions, labels, rotation=45, ha="right")
    axis.set_ylabel("median start-aligned 4.5 s error [m]")
    axis.grid(axis="y", alpha=0.3)
    axis.legend()
    fig.tight_layout()
    fig.savefig(output_dir / "propagation_error_components_4_5_by_case.png", dpi=160)
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
    if save_metric_plot(
            rows, output_dir, "full_4_5s_trajectory_containment_rate",
            "full_trajectory_containment_by_case.png",
            "full 4.5 s trajectory containment", 0.95):
        plots.append("full_trajectory_containment_by_case.png")
    if save_metric_plot(
            rows, output_dir, "propagation_error_4_5_median_m",
            "propagation_error_4_5_by_case.png",
            "median start-aligned 4.5 s propagation error [m]"):
        plots.append("propagation_error_4_5_by_case.png")
    if save_propagation_component_plot(rows, output_dir):
        plots.append("propagation_error_components_4_5_by_case.png")

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
        "mean_causal_segment_trajectory_containment_rate": mean_or_none(
            [row["causal_segment_trajectory_containment_rate"] for row in rows]),
        "mean_full_4_5s_trajectory_containment_rate": mean_or_none(
            [row["full_4_5s_trajectory_containment_rate"] for row in rows]),
        "mean_fusion_horizon_error_median_m": mean_or_none(
            [row["fusion_horizon_error_median_m"] for row in rows]),
        "mean_delay_compensation_error_median_m": mean_or_none(
            [row["delay_compensation_error_median_m"] for row in rows]),
        "mean_horizon_zero_error_median_m": mean_or_none(
            [row["horizon_zero_error_median_m"] for row in rows]),
        "mean_propagation_error_4_5_median_m": mean_or_none(
            [row["propagation_error_4_5_median_m"] for row in rows]),
        "mean_propagation_error_4_5_p95_m": mean_or_none(
            [row["propagation_error_4_5_p95_m"] for row in rows]),
        "propagation_contract_run_count": sum(
            row["propagation_test_contract_present"] for row in rows),
        "propagation_contract_failure_count": sum(
            row["propagation_test_contract_present"]
            and row["propagation_test_contract_pass"] != 1
            for row in rows),
        "alignment_gate_failure_count": sum(
            row["alignment_gate_required"]
            and row["alignment_gate_pass"] != 1
            for row in rows),
        "trim_gate_failure_count": sum(
            row["trim_gate_required"] and row["trim_gate_pass"] != 1
            for row in rows),
        "total_covariance_failures": sum(row["covariance_failures"] for row in rows),
        "total_invalid_cones": sum(row["invalid_cones"] for row in rows),
        "total_non_zoh_cones": sum(row["non_zoh_cones"] for row in rows),
        "total_common_contract_failures": sum(
            row["common_contract_failures"] for row in rows),
        "formal_coverage_pass": None,
        "formal_coverage_note": (
            "Preliminary calibration only; independent holdout acceptance is not gated."),
        "plots": plots,
    }
    (output_dir / "batch_summary.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0 if summary["structurally_valid_run_count"] == len(rows) \
        and len(rows) == expected else 1


if __name__ == "__main__":
    raise SystemExit(main())
