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
        "horizon_s", "error_norm_m", "mahalanobis_sq", "inside_95",
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


def save_example(arrays_path: Path, output: Path):
    arrays = np.load(arrays_path)
    predicted = arrays["predicted_mean"]
    covariance = arrays["predicted_position_covariance"]
    ground_truth = arrays["ground_truth"]
    if predicted.shape[0] == 0:
        return False
    index = predicted.shape[0] // 2
    mean = predicted[index]
    truth = ground_truth[index]
    fig, axis = plt.subplots(figsize=(7, 7))
    axis.plot(mean[:, 1], mean[:, 0], "o-", markersize=2.5, label="predicted mean")
    axis.plot(truth[:, 1], truth[:, 0], "-", color="tab:orange", label="ground truth")
    for point in range(0, mean.shape[0], 5):
        # Plot axes are East, North, so reorder both center and covariance.
        reordered_mean = mean[point, [1, 0, 2]]
        reordered_covariance = covariance[index, point][np.ix_([1, 0, 2], [1, 0, 2])]
        add_horizontal_ellipse(axis, reordered_mean, reordered_covariance)
    axis.set(xlabel="East [m]", ylabel="North [m]", title="Representative 95% trajectory cone")
    axis.axis("equal")
    axis.grid(alpha=0.3)
    axis.legend()
    fig.tight_layout()
    fig.savefig(output / "trajectory_cone_example.png", dpi=160)
    plt.close(fig)
    return True


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
    save_mahalanobis(rows, output_dir)
    example_written = save_example(arrays_path, output_dir)

    manifest = {
        "source": str(input_dir.resolve()),
        "title": args.title,
        "sample_count": len(rows),
        "plots": [
            "coverage_by_horizon.png",
            "position_error_by_horizon.png",
            "mahalanobis_by_horizon.png",
        ] + (["trajectory_cone_example.png"] if example_written else []),
    }
    (output_dir / "plot_manifest.json").write_text(
        json.dumps(manifest, indent=2), encoding="utf-8")
    print(json.dumps(manifest, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
