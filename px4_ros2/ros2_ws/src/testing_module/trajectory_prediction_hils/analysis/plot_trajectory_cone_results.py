#!/usr/bin/env python3
"""Create headless PNG diagnostics from one offline cone-bag analysis."""

import argparse
import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Ellipse


CHI_SQUARE_95_DF3 = 7.8147279


def load_rows(path: Path):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    numeric_fields = (
        "horizon_s", "error_norm_m", "propagation_error_norm_m",
        "mahalanobis_sq", "inside_95",
    )
    for row in rows:
        for field in numeric_fields:
            row[field] = float(row[field])
    return rows


def grouped(rows, field):
    horizons = sorted({row["horizon_s"] for row in rows})
    values = [
        np.asarray([row[field] for row in rows if row["horizon_s"] == horizon])
        for horizon in horizons
    ]
    return np.asarray(horizons), values


def save_coverage(rows, output: Path):
    horizons, values = grouped(rows, "inside_95")
    coverage = np.asarray([100.0 * np.mean(item) for item in values])
    fig, axis = plt.subplots(figsize=(8, 4.5))
    axis.plot(horizons, coverage, marker="o", markersize=3, label="empirical coverage")
    axis.axhline(95.0, color="tab:red", linestyle="--", label="nominal 95%")
    axis.set(xlabel="prediction horizon [s]", ylabel="inside cone [%]", ylim=(0, 102))
    axis.grid(alpha=0.3)
    axis.legend()
    fig.tight_layout()
    fig.savefig(output / "coverage_by_horizon.png", dpi=160)
    plt.close(fig)


def save_error(rows, output: Path):
    horizons, values = grouped(rows, "error_norm_m")
    median = np.asarray([np.median(item) for item in values])
    percentile_95 = np.asarray([np.percentile(item, 95) for item in values])
    maximum = np.asarray([np.max(item) for item in values])
    fig, axis = plt.subplots(figsize=(8, 4.5))
    axis.plot(horizons, median, label="median")
    axis.plot(horizons, percentile_95, label="95th percentile")
    axis.plot(horizons, maximum, alpha=0.65, label="maximum")
    axis.set(xlabel="prediction horizon [s]", ylabel="position error [m]")
    axis.grid(alpha=0.3)
    axis.legend()
    fig.tight_layout()
    fig.savefig(output / "position_error_by_horizon.png", dpi=160)
    plt.close(fig)


def save_propagation_error(rows, output: Path):
    horizons, values = grouped(rows, "propagation_error_norm_m")
    median = np.asarray([np.median(item) for item in values])
    percentile_95 = np.asarray([np.percentile(item, 95) for item in values])
    maximum = np.asarray([np.max(item) for item in values])
    fig, axis = plt.subplots(figsize=(8, 4.5))
    axis.plot(horizons, median, label="median")
    axis.plot(horizons, percentile_95, label="95th percentile")
    axis.plot(horizons, maximum, alpha=0.65, label="maximum")
    axis.set(
        xlabel="prediction horizon [s]",
        ylabel="start-aligned propagation error [m]")
    axis.grid(alpha=0.3)
    axis.legend()
    fig.tight_layout()
    fig.savefig(output / "propagation_error_by_horizon.png", dpi=160)
    plt.close(fig)


def save_mahalanobis(rows, output: Path):
    horizons, values = grouped(rows, "mahalanobis_sq")
    median = np.asarray([np.median(item) for item in values])
    percentile_95 = np.asarray([np.percentile(item, 95) for item in values])
    fig, axis = plt.subplots(figsize=(8, 4.5))
    axis.plot(horizons, median, label="median")
    axis.plot(horizons, percentile_95, label="95th percentile")
    axis.axhline(
        CHI_SQUARE_95_DF3, color="tab:red", linestyle="--", label="95% threshold")
    axis.set(xlabel="prediction horizon [s]", ylabel="squared Mahalanobis distance")
    axis.grid(alpha=0.3)
    axis.legend()
    fig.tight_layout()
    fig.savefig(output / "mahalanobis_by_horizon.png", dpi=160)
    plt.close(fig)


def save_trajectory_survival(rows, output: Path):
    """Plot the fraction of evaluable cones that have not exited by each horizon."""
    by_cone = {}
    for row in rows:
        by_cone.setdefault(row["source_timestamp_us"], []).append(row)
    horizons = sorted({row["horizon_s"] for row in rows})
    survival = []
    at_risk = []
    for horizon in horizons:
        eligible = [
            cone_rows for cone_rows in by_cone.values()
            if max(row["horizon_s"] for row in cone_rows) >= horizon
        ]
        survived = sum(
            all(row["inside_95"] >= 0.5 for row in cone_rows
                if row["horizon_s"] <= horizon)
            for cone_rows in eligible
        )
        at_risk.append(len(eligible))
        survival.append(100.0 * survived / len(eligible) if eligible else np.nan)
    fig, axis = plt.subplots(figsize=(8, 4.5))
    axis.step(horizons, survival, where="post", label="no 95% cone exit yet")
    axis.axhline(95.0, color="tab:red", linestyle="--", label="nominal reference")
    axis.set(
        xlabel="prediction horizon [s]",
        ylabel="trajectory survival inside cone [%]",
        ylim=(0, 102),
    )
    axis.grid(alpha=0.3)
    axis.legend()
    axis.text(
        0.99, 0.03,
        f"at risk: {at_risk[0]} → {at_risk[-1]}",
        transform=axis.transAxes, ha="right", va="bottom", fontsize=9)
    fig.tight_layout()
    fig.savefig(output / "trajectory_survival_by_horizon.png", dpi=160)
    plt.close(fig)


def save_initial_alignment(input_dir: Path, output: Path):
    path = input_dir / "initial_alignment.csv"
    if not path.is_file():
        return False
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        return False
    fields = (
        "fusion_error_norm_m",
        "delay_compensation_error_norm_m",
        "horizon_zero_error_norm_m",
    )
    values = [np.asarray([float(row[field]) for row in rows]) for field in fields]
    fig, axis = plt.subplots(figsize=(8, 4.5))
    axis.boxplot(values, tick_labels=(
        "fusion-horizon\nEKF error",
        "delay-compensation\nerror",
        "horizon-zero\nerror",
    ), showfliers=False)
    axis.set_ylabel("position error norm [m]")
    axis.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(output / "initial_alignment_decomposition.png", dpi=160)
    plt.close(fig)
    return True


def add_horizontal_ellipse(axis, center, covariance):
    covariance_2d = 0.5 * (covariance[:2, :2] + covariance[:2, :2].T)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance_2d)
    eigenvalues = np.maximum(eigenvalues, 0.0)
    order = np.argsort(eigenvalues)[::-1]
    eigenvalues = eigenvalues[order]
    eigenvectors = eigenvectors[:, order]
    scale = np.sqrt(CHI_SQUARE_95_DF3)
    width, height = 2.0 * scale * np.sqrt(eigenvalues)
    angle = np.degrees(np.arctan2(eigenvectors[1, 0], eigenvectors[0, 0]))
    axis.add_patch(Ellipse(
        xy=center[:2], width=width, height=height, angle=angle,
        fill=False, edgecolor="tab:blue", alpha=0.35, linewidth=1.0))


def matched_sample_indices(point_count: int):
    """Return four common array indices spanning the evaluated horizon."""
    return np.unique(np.rint(np.linspace(0, point_count - 1, 4)).astype(int))


def add_same_index_connectors(axis, mean, truth, horizons, indices):
    """Make it explicit that prediction[i] is compared with truth[i]."""
    for order, point in enumerate(indices):
        axis.plot(
            [mean[point, 1], truth[point, 1]],
            [mean[point, 0], truth[point, 0]],
            color="0.35", linestyle=":", linewidth=1.0, alpha=0.85,
            zorder=2,
            label="same-index error" if order == 0 else None)
        midpoint_e = 0.5 * (mean[point, 1] + truth[point, 1])
        midpoint_n = 0.5 * (mean[point, 0] + truth[point, 0])
        if point != 0:
            axis.annotate(
                f"i={point}, t={horizons[point]:.1f}s",
                xy=(midpoint_e, midpoint_n), xytext=(4, 4),
                textcoords="offset points", fontsize=7, color="0.3")


def save_example(arrays_path: Path, output: Path):
    arrays = np.load(arrays_path)
    predicted = arrays["predicted_mean"]
    covariance = arrays["predicted_position_covariance"]
    ground_truth = arrays["ground_truth"]
    if predicted.shape[0] == 0:
        return None
    point_counts = arrays["causal_point_count"] \
        if "causal_point_count" in arrays else np.full(predicted.shape[0], predicted.shape[1])
    candidates = arrays["candidate_inputs"] \
        if "candidate_inputs" in arrays else np.zeros((predicted.shape[0], 4))
    timestamps = arrays["source_timestamp_us"] \
        if "source_timestamp_us" in arrays else np.zeros(predicted.shape[0], dtype=np.int64)
    time_offsets = arrays["time_offsets_s"] \
        if "time_offsets_s" in arrays else None

    if predicted.shape != ground_truth.shape:
        raise ValueError(
            "predicted_mean and ground_truth must have identical shapes")
    if covariance.shape[:2] != predicted.shape[:2]:
        raise ValueError(
            "predicted covariance and trajectory sample dimensions do not match")

    # A maneuver case should display a maneuver-time cone, not the arbitrary
    # middle cone. Prefer lateral excitation, then vertical excitation.
    lateral = np.abs(candidates[:, 3])
    vertical = np.abs(candidates[:, 2])
    if np.max(lateral) > 0.05:
        representative = np.flatnonzero(lateral >= np.max(lateral) - 1.0e-6)
    elif np.max(vertical) > 0.05:
        representative = np.flatnonzero(vertical >= np.max(vertical) - 1.0e-6)
    else:
        representative = np.arange(predicted.shape[0])
    index = int(representative[len(representative) // 2])
    point_count = int(point_counts[index])
    if not 1 <= point_count <= predicted.shape[1]:
        raise ValueError(f"invalid causal_point_count: {point_count}")
    mean = predicted[index, :point_count]
    truth = ground_truth[index, :point_count]
    horizons = (
        np.asarray(time_offsets[index, :point_count], dtype=np.float64)
        if time_offsets is not None
        else np.arange(point_count, dtype=np.float64) * 0.1
    )
    if not np.isfinite(mean).all() or not np.isfinite(truth).all():
        raise ValueError("representative trajectory contains non-finite samples")
    if not np.isfinite(horizons).all() or np.any(np.diff(horizons) <= 0.0):
        raise ValueError("trajectory time offsets must be finite and strictly increasing")

    alignment_error_m = float(np.linalg.norm(truth[0] - mean[0]))
    terminal_error = truth[-1] - mean[-1]
    terminal_absolute_error_m = float(np.linalg.norm(terminal_error))
    terminal_horizontal_absolute_error_m = float(
        np.linalg.norm(terminal_error[:2]))
    relative_mean = mean - mean[0]
    relative_truth = truth - truth[0]
    terminal_propagation_error = relative_truth[-1] - relative_mean[-1]
    terminal_propagation_error_m = float(
        np.linalg.norm(terminal_propagation_error))
    terminal_horizontal_propagation_error_m = float(
        np.linalg.norm(terminal_propagation_error[:2]))
    matched_indices = matched_sample_indices(point_count)

    fig, axis = plt.subplots(figsize=(7, 7))
    axis.plot(
        mean[:, 1], mean[:, 0], "-", marker="o", markersize=3.0,
        color="tab:blue", label=f"predicted mean (i=0..{point_count - 1})")
    axis.plot(
        truth[:, 1], truth[:, 0], "-", marker="x", markersize=3.5,
        markeredgewidth=0.9, color="tab:orange",
        label="ground truth (same i)")
    add_same_index_connectors(
        axis, mean, truth, horizons, matched_indices)
    axis.scatter(
        mean[0, 1], mean[0, 0], marker="s", s=45, color="tab:blue",
        zorder=5, label="predicted t=0")
    axis.scatter(
        truth[0, 1], truth[0, 0], marker="x", s=55, color="tab:orange",
        linewidths=2.0, zorder=6, label="truth t=0")
    axis.scatter(
        mean[-1, 1], mean[-1, 0], marker="^", s=65, color="tab:blue",
        edgecolors="white", linewidths=0.6, zorder=7,
        label=f"predicted t={horizons[-1]:.1f}s")
    axis.scatter(
        truth[-1, 1], truth[-1, 0], marker="v", s=65,
        color="tab:orange", edgecolors="white", linewidths=0.6, zorder=7,
        label=f"truth t={horizons[-1]:.1f}s")
    for point in range(0, mean.shape[0], 5):
        # Plot axes are East, North, so reorder both center and covariance.
        reordered_mean = mean[point, [1, 0, 2]]
        reordered_covariance = covariance[index, point][np.ix_([1, 0, 2], [1, 0, 2])]
        add_horizontal_ellipse(axis, reordered_mean, reordered_covariance)
    candidate = candidates[index]
    horizon_s = float(horizons[-1])
    axis.set(
        xlabel="East [m]",
        ylabel="North [m]",
        title=(
            "Representative causal 95% trajectory cone\n"
            f"ZOH: V={candidate[0]:.1f}, h_dot={candidate[2]:.1f}, "
            f"a_lat={candidate[3]:.1f}; evaluated horizon={horizon_s:.1f}s"
        ),
    )
    axis.axis("equal")
    axis.grid(alpha=0.3)
    axis.text(
        0.98, 0.02,
        f"matched samples: {point_count} (prediction[i] ↔ truth[i])\n"
        f"terminal error: XY={terminal_horizontal_absolute_error_m:.3f} m, "
        f"3D={terminal_absolute_error_m:.3f} m",
        transform=axis.transAxes, ha="right", va="bottom", fontsize=8.5,
        bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.85})
    axis.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(output / "trajectory_cone_example.png", dpi=160)
    plt.close(fig)
    fig, axis = plt.subplots(figsize=(7, 7))
    axis.plot(
        relative_mean[:, 1], relative_mean[:, 0], "-", marker="o",
        markersize=3.0, color="tab:blue",
        label=f"predicted displacement (i=0..{point_count - 1})")
    axis.plot(
        relative_truth[:, 1], relative_truth[:, 0], "-", marker="x",
        markersize=3.5, markeredgewidth=0.9,
        color="tab:orange", label="ground-truth displacement (same i)")
    add_same_index_connectors(
        axis, relative_mean, relative_truth, horizons, matched_indices)
    axis.scatter(0.0, 0.0, marker="s", s=45, color="black", zorder=5)
    axis.scatter(
        relative_mean[-1, 1], relative_mean[-1, 0], marker="^", s=65,
        color="tab:blue", edgecolors="white", linewidths=0.6, zorder=7,
        label=f"predicted t={horizon_s:.1f}s")
    axis.scatter(
        relative_truth[-1, 1], relative_truth[-1, 0], marker="v", s=65,
        color="tab:orange", edgecolors="white", linewidths=0.6, zorder=7,
        label=f"truth t={horizon_s:.1f}s")
    axis.set(
        xlabel="relative East [m]",
        ylabel="relative North [m]",
        title=(
            "Start-aligned propagation diagnostic\n"
            f"removed horizon-zero offset={alignment_error_m:.3f}m; "
            f"terminal 3D propagation error={terminal_propagation_error_m:.3f}m"),
    )
    axis.axis("equal")
    axis.grid(alpha=0.3)
    axis.text(
        0.98, 0.02,
        f"matched samples: {point_count} (prediction[i] ↔ truth[i])\n"
        f"terminal propagation error: "
        f"XY={terminal_horizontal_propagation_error_m:.3f} m, "
        f"3D={terminal_propagation_error_m:.3f} m",
        transform=axis.transAxes, ha="right", va="bottom", fontsize=8.5,
        bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.85})
    axis.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(output / "trajectory_start_aligned.png", dpi=160)
    plt.close(fig)

    relative_mean_altitude = -relative_mean[:, 2]
    relative_truth_altitude = -relative_truth[:, 2]
    fig, axis = plt.subplots(figsize=(8, 4.8))
    axis.plot(
        horizons, relative_mean_altitude, "-", marker="o", markersize=3.0,
        label=f"predicted relative altitude (i=0..{point_count - 1})")
    axis.plot(
        horizons, relative_truth_altitude, "-", marker="x", markersize=3.5,
        markeredgewidth=0.9, color="tab:orange",
        label="ground-truth relative altitude (same i)")
    axis.plot(
        [horizons[-1], horizons[-1]],
        [relative_mean_altitude[-1], relative_truth_altitude[-1]],
        color="0.35", linestyle=":", linewidth=1.0,
        label="same-index terminal error")
    axis.set(
        xlabel="prediction horizon [s]",
        ylabel="relative altitude [m]",
        title=(
            "Start-aligned vertical propagation diagnostic\n"
            f"ZOH h_dot={candidate[2]:.1f}m/s; "
            f"terminal truth−prediction={terminal_propagation_error[2] * -1.0:.3f}m"),
    )
    axis.grid(alpha=0.3)
    axis.legend()
    fig.tight_layout()
    fig.savefig(output / "trajectory_vertical_start_aligned.png", dpi=160)
    plt.close(fig)
    return {
        "array_index": index,
        "source_timestamp_us": int(timestamps[index]),
        "causal_point_count": point_count,
        "evaluated_horizon_s": horizon_s,
        "horizon_zero_alignment_error_m": alignment_error_m,
        "sample_index_contract": (
            f"prediction[i] and ground_truth[i] share time_offsets_s[i]; "
            f"i=0..{point_count - 1}"),
        "terminal_horizontal_absolute_error_m": (
            terminal_horizontal_absolute_error_m),
        "terminal_absolute_error_m": terminal_absolute_error_m,
        "terminal_horizontal_propagation_error_m": (
            terminal_horizontal_propagation_error_m),
        "terminal_propagation_error_m": terminal_propagation_error_m,
        "terminal_vertical_propagation_error_m": float(
            relative_truth_altitude[-1] - relative_mean_altitude[-1]),
        "candidate_input": {
            "V_cmd": float(candidate[0]),
            "h_cmd": float(candidate[1]),
            "h_dot_cmd": float(candidate[2]),
            "a_lat_cmd": float(candidate[3]),
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="offline analysis directory")
    parser.add_argument("--output", required=True, help="PNG output directory")
    parser.add_argument("--title", default="")
    args = parser.parse_args()

    input_dir = Path(args.input)
    output_dir = Path(args.output)
    samples_path = input_dir / "cone_samples.csv"
    arrays_path = input_dir / "cone_arrays.npz"
    summary_path = input_dir / "summary.json"
    missing = [path.name for path in (samples_path, arrays_path, summary_path) if not path.is_file()]
    if missing:
        print(f"ERROR: missing analysis files in {input_dir}: {', '.join(missing)}")
        return 2

    rows = load_rows(samples_path)
    if not rows:
        print(f"ERROR: no analyzed cone samples in {samples_path}")
        return 2
    output_dir.mkdir(parents=True, exist_ok=True)
    save_coverage(rows, output_dir)
    save_error(rows, output_dir)
    save_propagation_error(rows, output_dir)
    save_mahalanobis(rows, output_dir)
    save_trajectory_survival(rows, output_dir)
    alignment_plot = save_initial_alignment(input_dir, output_dir)
    example = save_example(arrays_path, output_dir)

    manifest = {
        "source": str(input_dir.resolve()),
        "title": args.title,
        "sample_count": len(rows),
        "plots": [
            "coverage_by_horizon.png",
            "position_error_by_horizon.png",
            "propagation_error_by_horizon.png",
            "mahalanobis_by_horizon.png",
            "trajectory_survival_by_horizon.png",
        ] + (["initial_alignment_decomposition.png"] if alignment_plot else []) \
          + (["trajectory_cone_example.png", "trajectory_start_aligned.png",
              "trajectory_vertical_start_aligned.png"]
             if example else []),
        "representative_cone": example,
    }
    (output_dir / "plot_manifest.json").write_text(
        json.dumps(manifest, indent=2), encoding="utf-8")
    print(json.dumps(manifest, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
