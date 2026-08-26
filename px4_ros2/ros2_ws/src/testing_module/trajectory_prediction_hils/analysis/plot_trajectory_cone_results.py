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
from matplotlib.lines import Line2D
from matplotlib.patches import Ellipse


CHI_SQUARE_95_DF3 = 7.8147279
NED_TO_EAST_NORTH_ALTITUDE = np.asarray([
    [0.0, 1.0, 0.0],
    [1.0, 0.0, 0.0],
    [0.0, 0.0, -1.0],
])


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
                xy=(midpoint_e, midpoint_n), xytext=(0, -14),
                textcoords="offset points", ha="center", va="top",
                fontsize=7, color="0.3", annotation_clip=False,
                bbox={
                    "boxstyle": "round,pad=0.15",
                    "facecolor": "white",
                    "edgecolor": "none",
                    "alpha": 0.82,
                })


def validate_integration_contract(time_offsets, point_counts):
    """Validate the shared mean/covariance horizon time axis in one NPZ."""
    if time_offsets is None:
        return {
            "verified_from_recorded_time_offsets": False,
            "shared_mean_covariance_time_axis": True,
            "trajectory_mean_integration_step_s": 0.1,
            "cone_covariance_integration_step_s": 0.1,
            "maximum_step_deviation_s": None,
            "pre_horizon_delay_compensation_is_separate": True,
            "note": "legacy NPZ without time_offsets_s; 0.1 s fallback used",
        }

    all_steps = []
    for row, raw_count in zip(time_offsets, point_counts):
        count = int(raw_count)
        if not 1 <= count <= row.shape[0]:
            raise ValueError(f"invalid causal_point_count: {count}")
        offsets = np.asarray(row[:count], dtype=np.float64)
        if not np.isfinite(offsets).all() or abs(offsets[0]) > 1.0e-6:
            raise ValueError("recorded time offsets must be finite and start at zero")
        steps = np.diff(offsets)
        if np.any(steps <= 0.0):
            raise ValueError("recorded time offsets must be strictly increasing")
        if steps.size:
            all_steps.append(steps)

    if not all_steps:
        raise ValueError("recorded trajectories do not contain an integration interval")
    steps = np.concatenate(all_steps)
    integration_step_s = float(np.median(steps))
    maximum_deviation_s = float(np.max(np.abs(steps - integration_step_s)))
    tolerance_s = max(1.0e-6, abs(integration_step_s) * 1.0e-5)
    if maximum_deviation_s > tolerance_s:
        raise ValueError(
            "non-uniform mean/covariance horizon time axis: "
            f"median={integration_step_s:.9f}s, "
            f"maximum deviation={maximum_deviation_s:.9f}s")

    return {
        "verified_from_recorded_time_offsets": True,
        "shared_mean_covariance_time_axis": True,
        "trajectory_mean_integration_step_s": integration_step_s,
        "cone_covariance_integration_step_s": integration_step_s,
        "maximum_step_deviation_s": maximum_deviation_s,
        "recorded_interval_count": int(steps.size),
        "pre_horizon_delay_compensation_is_separate": True,
        "note": (
            "predicted_mean and predicted_position_covariance share each "
            "time_offsets_s index; cone publication rate and pre-horizon "
            "fusion-delay compensation are separate from the horizon step"),
    }


def covariance_ellipsoid_geometry(
        mean_ned, covariance_ned, polar_samples=13, azimuth_samples=25,
        temporal_generator_count=12):
    """Build exact pointwise 3-D covariance ellipsoids in the plot frame."""
    if polar_samples < 3:
        raise ValueError("ellipsoid mesh requires at least three polar samples")
    if azimuth_samples < 4:
        raise ValueError("ellipsoid mesh requires at least four azimuth samples")
    if temporal_generator_count < 1:
        raise ValueError("ellipsoid mesh requires at least one generator")
    if mean_ned.ndim != 2 or mean_ned.shape[1] != 3:
        raise ValueError("ellipsoid means must have shape (sample, 3)")
    if covariance_ned.shape != (mean_ned.shape[0], 3, 3):
        raise ValueError("ellipsoid covariances must have shape (sample, 3, 3)")
    if not np.isfinite(mean_ned).all() or not np.isfinite(covariance_ned).all():
        raise ValueError("ellipsoid means and covariances must be finite")

    centers = mean_ned @ NED_TO_EAST_NORTH_ALTITUDE.T
    covariance_plot = np.einsum(
        "ab,nbc,dc->nad",
        NED_TO_EAST_NORTH_ALTITUDE,
        covariance_ned,
        NED_TO_EAST_NORTH_ALTITUDE,
    )

    polar = np.linspace(0.0, np.pi, polar_samples)
    azimuth = np.linspace(0.0, 2.0 * np.pi, azimuth_samples)
    sin_polar = np.sin(polar)[:, np.newaxis]
    unit_surface = np.stack((
        sin_polar * np.cos(azimuth)[np.newaxis, :],
        sin_polar * np.sin(azimuth)[np.newaxis, :],
        np.cos(polar)[:, np.newaxis] * np.ones_like(azimuth)[np.newaxis, :],
    ), axis=-1)

    # Fixed global unit-sphere directions make the temporal rendering lines
    # deterministic. The symmetric PSD square root below avoids eigenvector
    # sign/swap discontinuities while preserving each exact ellipsoid surface.
    generator_order = np.arange(temporal_generator_count, dtype=np.float64) + 0.5
    generator_z = 1.0 - 2.0 * generator_order / temporal_generator_count
    generator_radius = np.sqrt(np.maximum(1.0 - generator_z ** 2, 0.0))
    golden_angle = np.pi * (3.0 - np.sqrt(5.0))
    generator_azimuth = golden_angle * generator_order
    unit_generators = np.column_stack((
        generator_radius * np.cos(generator_azimuth),
        generator_radius * np.sin(generator_azimuth),
        generator_z,
    ))

    scale = np.sqrt(CHI_SQUARE_95_DF3)
    surfaces = []
    temporal_generators = []
    for point, center in enumerate(centers):
        symmetric_covariance = 0.5 * (
            covariance_plot[point] + covariance_plot[point].T)
        eigenvalues, eigenvectors = np.linalg.eigh(symmetric_covariance)
        psd_tolerance = max(
            1.0e-9, 1.0e-8 * max(float(np.max(np.abs(eigenvalues))), 1.0))
        if np.min(eigenvalues) < -psd_tolerance:
            raise ValueError("position covariance is not positive semidefinite")
        eigenvalues = np.maximum(eigenvalues, 0.0)
        symmetric_covariance_root = (
            eigenvectors @ np.diag(np.sqrt(eigenvalues)) @ eigenvectors.T)
        surface_offsets = scale * np.einsum(
            "ij,abj->abi", symmetric_covariance_root, unit_surface)
        generator_offsets = scale * np.einsum(
            "ij,nj->ni", symmetric_covariance_root, unit_generators)
        surfaces.append(center[np.newaxis, np.newaxis, :] + surface_offsets)
        temporal_generators.append(center[np.newaxis, :] + generator_offsets)

    return centers, np.asarray(surfaces), np.asarray(temporal_generators)


def covariance_ellipsoid_mesh_indices(
        sample_count, polar_sample_count, azimuth_sample_count,
        target_time_sections=10, target_latitudes=6, target_meridians=12):
    """Choose deterministic time sections and ellipsoid wireframe lines."""
    if sample_count < 1:
        raise ValueError("trajectory mesh requires at least one time section")
    if polar_sample_count < 3:
        raise ValueError("ellipsoid mesh requires at least three polar samples")
    unique_azimuth_count = azimuth_sample_count - 1
    if unique_azimuth_count < 3:
        raise ValueError("ellipsoid mesh requires at least three azimuths")

    section_count = min(target_time_sections, sample_count)
    section_indices = np.unique(np.rint(
        np.linspace(0, sample_count - 1, section_count)).astype(int))
    latitude_count = min(target_latitudes, polar_sample_count - 2)
    latitude_indices = np.unique(np.rint(np.linspace(
        1, polar_sample_count - 2, latitude_count)).astype(int))
    meridian_count = min(target_meridians, unique_azimuth_count)
    meridian_indices = np.unique(np.floor(np.linspace(
        0, unique_azimuth_count, meridian_count,
        endpoint=False)).astype(int))
    return section_indices, latitude_indices, meridian_indices


def add_covariance_ellipsoid_wireframes(
        axis, surfaces, section_indices, latitude_indices,
        meridian_indices, dimensions):
    """Draw latitude and meridian lines on selected full ellipsoid surfaces."""
    for point in section_indices:
        for latitude in latitude_indices:
            coordinates = [
                surfaces[point, latitude, :, dimension]
                for dimension in dimensions]
            axis.plot(
                *coordinates, color="tab:blue", alpha=0.34, linewidth=0.65)
        for meridian in meridian_indices:
            coordinates = [
                surfaces[point, :, meridian, dimension]
                for dimension in dimensions]
            axis.plot(
                *coordinates, color="tab:blue", alpha=0.34, linewidth=0.65)


def add_covariance_temporal_generators(axis, generators, dimensions):
    """Connect identical unit-sphere directions across prediction times."""
    if generators.shape[0] < 2:
        return 0
    for generator in range(generators.shape[1]):
        coordinates = [
            generators[:, generator, dimension] for dimension in dimensions]
        axis.plot(
            *coordinates, color="tab:blue", alpha=0.48, linewidth=0.75)
    return generators.shape[1]


def save_plan_view_ellipsoid_mesh(
        mean_plot, truth_plot, surfaces, temporal_generators, horizons,
        matched_indices, section_indices, latitude_indices,
        meridian_indices, output):
    """Render the plan-view projection of the full 3-D ellipsoid wireframe."""
    fig, axis = plt.subplots(figsize=(8.5, 7.0))
    add_covariance_ellipsoid_wireframes(
        axis, surfaces, section_indices, latitude_indices,
        meridian_indices, dimensions=(0, 1))
    temporal_generator_count = add_covariance_temporal_generators(
        axis, temporal_generators, dimensions=(0, 1))
    axis.plot(
        mean_plot[:, 0], mean_plot[:, 1], "-o", color="navy",
        markersize=2.8, linewidth=1.9, label="predicted mean")
    axis.plot(
        truth_plot[:, 0], truth_plot[:, 1], "-x", color="tab:orange",
        markersize=3.2, linewidth=1.5, label="ground truth (same i)")
    axis.scatter(
        mean_plot[0, 0], mean_plot[0, 1], marker="s", s=42,
        color="navy", zorder=5)
    axis.scatter(
        truth_plot[0, 0], truth_plot[0, 1], marker="x", s=55,
        color="tab:orange", zorder=6)
    axis.scatter(
        mean_plot[-1, 0], mean_plot[-1, 1], marker="^", s=58,
        color="navy", zorder=5)
    axis.scatter(
        truth_plot[-1, 0], truth_plot[-1, 1], marker="v", s=58,
        color="tab:orange", zorder=6)
    for point in matched_indices[1:]:
        axis.annotate(
            f"t={horizons[point]:.1f}s",
            (mean_plot[point, 0], mean_plot[point, 1]),
            xytext=(0, -13), textcoords="offset points",
            ha="center", va="top", fontsize=7, color="0.3",
            bbox={
                "boxstyle": "round,pad=0.15", "facecolor": "white",
                "edgecolor": "none", "alpha": 0.80,
            })

    handles, labels = axis.get_legend_handles_labels()
    handles.append(Line2D(
        [0], [0], color="tab:blue", alpha=0.55, linewidth=1.0,
        label="full pointwise nominal 95% covariance ellipsoids"))
    labels.append("full pointwise nominal 95% covariance ellipsoids")
    if temporal_generator_count:
        handles.append(Line2D(
            [0], [0], color="tab:blue", alpha=0.48, linewidth=0.75,
            label="same sphere-direction rendering lines"))
        labels.append("same sphere-direction rendering lines")
    axis.legend(handles, labels, loc="upper left", fontsize=8)
    axis.set(
        xlabel="East [m]", ylabel="North [m]",
        title=(
            "Plan-view projection of the full 3-D covariance-ellipsoid mesh\n"
            "Ellipsoid orientation and radii come from each full 3x3 covariance"),
    )
    axis.axis("equal")
    axis.grid(alpha=0.25)
    fig.text(
        0.5, 0.015,
        "Every grid lies on its pointwise 3-D covariance ellipsoid; temporal "
        "connectors are rendering lines, not a simultaneous confidence tube.",
        ha="center", fontsize=8, color="0.35")
    # A near-vertical plan-view path can otherwise push the two-line title
    # outside the canvas when equal data aspect is enforced.
    fig.tight_layout(rect=(0.0, 0.035, 1.0, 0.90))
    fig.savefig(output / "trajectory_cone_mesh.png", dpi=180)
    plt.close(fig)


def set_3d_data_aspect(axis, points):
    """Use physical data ranges without silently exaggerating altitude."""
    minima = np.min(points, axis=0)
    maxima = np.max(points, axis=0)
    ranges = np.maximum(maxima - minima, 1.0e-3)
    padding = np.maximum(0.05 * ranges, 0.1)
    axis.set_xlim(minima[0] - padding[0], maxima[0] + padding[0])
    axis.set_ylim(minima[1] - padding[1], maxima[1] + padding[1])
    axis.set_zlim(minima[2] - padding[2], maxima[2] + padding[2])
    axis.set_box_aspect(ranges + 2.0 * padding)


def save_connected_3d_envelope(
        mean, truth, covariance, horizons, matched_indices,
        integration_contract, output):
    """Render full pointwise 3-D covariance ellipsoids as a wireframe mesh."""
    mean_plot, surfaces, temporal_generators = covariance_ellipsoid_geometry(
        mean, covariance)
    truth_plot = truth @ NED_TO_EAST_NORTH_ALTITUDE.T
    section_indices, latitude_indices, meridian_indices = (
        covariance_ellipsoid_mesh_indices(
            surfaces.shape[0], surfaces.shape[1], surfaces.shape[2]))
    save_plan_view_ellipsoid_mesh(
        mean_plot, truth_plot, surfaces, temporal_generators, horizons,
        matched_indices, section_indices, latitude_indices,
        meridian_indices, output)

    fig = plt.figure(figsize=(10, 8))
    axis = fig.add_subplot(111, projection="3d")
    add_covariance_ellipsoid_wireframes(
        axis, surfaces, section_indices, latitude_indices,
        meridian_indices, dimensions=(0, 1, 2))
    temporal_generator_count = add_covariance_temporal_generators(
        axis, temporal_generators, dimensions=(0, 1, 2))

    axis.plot(
        mean_plot[:, 0], mean_plot[:, 1], mean_plot[:, 2],
        "-o", color="navy", markersize=2.8, linewidth=1.9,
        label="predicted mean")
    axis.plot(
        truth_plot[:, 0], truth_plot[:, 1], truth_plot[:, 2],
        "-x", color="tab:orange", markersize=3.2, linewidth=1.6,
        label="ground truth (same i)")
    for order, point in enumerate(matched_indices):
        axis.plot(
            [mean_plot[point, 0], truth_plot[point, 0]],
            [mean_plot[point, 1], truth_plot[point, 1]],
            [mean_plot[point, 2], truth_plot[point, 2]],
            color="0.3", linestyle=":", linewidth=1.0,
            label="same-index error" if order == 0 else None)
    axis.scatter(*mean_plot[0], marker="s", s=42, color="navy")
    axis.scatter(*truth_plot[0], marker="x", s=55, color="tab:orange")
    axis.scatter(*mean_plot[-1], marker="^", s=58, color="navy")
    axis.scatter(*truth_plot[-1], marker="v", s=58, color="tab:orange")

    dt = integration_contract["trajectory_mean_integration_step_s"]
    axis.set(
        xlabel="East [m]", ylabel="North [m]", zlabel="Altitude [m]",
    )
    fig.suptitle(
        "Full 3-D covariance-ellipsoid trajectory mesh\n"
        "Pointwise nominal 95% ellipsoids from each full covariance; "
        f"shared mean/cov dt={dt:.3f}s",
        y=0.965)
    axis.view_init(elev=27, azim=-58)
    set_3d_data_aspect(
        axis,
        np.vstack((surfaces.reshape(-1, 3), mean_plot, truth_plot)),
    )
    handles, labels = axis.get_legend_handles_labels()
    handles.append(Line2D(
        [0], [0], color="tab:blue", alpha=0.55, linewidth=1.0,
        label="full pointwise nominal 95% covariance ellipsoids"))
    labels.append("full pointwise nominal 95% covariance ellipsoids")
    if temporal_generator_count:
        handles.append(Line2D(
            [0], [0], color="tab:blue", alpha=0.48, linewidth=0.75,
            label="same sphere-direction rendering lines"))
        labels.append("same sphere-direction rendering lines")
    fig.legend(
        handles, labels, loc="upper center", bbox_to_anchor=(0.5, 0.89),
        ncol=2, fontsize=8)
    fig.text(
        0.5, 0.015,
        "Full covariance determines every ellipsoid surface; temporal "
        "connectors are rendering lines and physical axes are not exaggerated.",
        ha="center", fontsize=8, color="0.35")
    # Keep the figure-owned title and legend independent of the 3-D axes box;
    # equal physical scaling changes that box substantially across track shapes.
    fig.tight_layout(rect=(0.0, 0.035, 1.0, 0.78))
    fig.savefig(output / "trajectory_cone_3d.png", dpi=180)
    plt.close(fig)
    return {
        "geometry": (
            "sampled wireframe surfaces of full nominal pointwise 3-D "
            "position-covariance ellipsoids centered on the predicted mean"),
        "render_style": "wireframe mesh",
        "time_section_count": int(section_indices.size),
        "generator_line_count": int(temporal_generator_count),
        "temporal_generator_line_count": int(temporal_generator_count),
        "time_section_indices": section_indices.tolist(),
        "rendered_latitude_indices": latitude_indices.tolist(),
        "rendered_meridian_indices": meridian_indices.tolist(),
        "section_selection_policy": (
            "uniform rounded indices including the first and last causal "
            "sample"),
        "ellipsoid_surface_definition": (
            "p = mean + sqrt(chi2_3(0.95)) * "
            "principal_symmetric_sqrt(P_plot) * u, ||u|| = 1"),
        "parameterization": (
            "fixed East-North-altitude unit-sphere grid reused at every "
            "horizon; eigenvector signs and ordering do not define mesh "
            "correspondence"),
        "polar_samples_including_poles": int(surfaces.shape[1]),
        "azimuth_samples_including_closed_seam": int(surfaces.shape[2]),
        "pointwise_position_ellipsoid_nominal_confidence_level": 0.95,
        "chi_square_degrees_of_freedom": 3,
        "chi_square_quantile_df3": CHI_SQUARE_95_DF3,
        "full_rank_required_for_exact_95_percent_probability": True,
        "path_normal_projection": False,
        "plan_view_semantics": (
            "East-North orthogonal projection of the pointwise 3-D df=3 "
            "ellipsoid mesh; not a 95% 2-D marginal contour"),
        "temporal_connector_semantics": (
            "fixed global unit-sphere directions mapped by each symmetric "
            "covariance square root and connected for visualization only; "
            "no inter-time probability coupling is implied"),
        "confidence_calibration_status": (
            "nominal covariance confidence; rendering alone does not "
            "demonstrate holdout calibration"),
        "simultaneous_trajectory_confidence_tube": False,
        "auto_acas_masd_reconstruction": False,
        "uncertainty_scope": (
            "project EKF position covariance plus propagated model/process "
            "uncertainty; not the complete Figure 8 uncertainty roll-up"),
        "coordinate_axes": "East, North, altitude",
        "covariance_transform": "P_plot = T * P_NED * T^T",
        "vertical_exaggeration": False,
        "visual_reference": (
            "project-owned wireframe inspired by Figure 8's time-growing "
            "uncertainty-envelope concept; ellipsoid geometry and mesh "
            "topology are not source-defined"),
    }


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
    if np.asarray(point_counts).shape != (predicted.shape[0],):
        raise ValueError("causal_point_count must contain one value per cone")
    if time_offsets is not None and time_offsets.shape != predicted.shape[:2]:
        raise ValueError(
            "time_offsets_s and mean/covariance sample dimensions do not match")
    integration_contract = validate_integration_contract(
        time_offsets, point_counts)

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
    covariance_for_example = covariance[index, :point_count]
    if not np.isfinite(covariance_for_example).all():
        raise ValueError("representative covariance contains non-finite samples")

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

    predicted_altitude = -mean[:, 2]
    truth_altitude = -truth[:, 2]
    absolute_vertical_error = truth_altitude - predicted_altitude
    propagation_vertical_error = (
        relative_truth_altitude - relative_mean_altitude)
    fig, axis = plt.subplots(figsize=(8, 4.8))
    axis.axhline(0.0, color="0.35", linestyle="--", linewidth=0.9)
    axis.plot(
        horizons, absolute_vertical_error, "-o", markersize=3.0,
        color="tab:purple", label="absolute altitude error")
    axis.plot(
        horizons, propagation_vertical_error, "-x", markersize=3.5,
        color="tab:green", label="start-aligned propagation error")
    axis.scatter(
        horizons[-1], absolute_vertical_error[-1], marker="o", s=48,
        color="tab:purple", zorder=5)
    axis.scatter(
        horizons[-1], propagation_vertical_error[-1], marker="x", s=55,
        color="tab:green", linewidths=1.8, zorder=5)
    axis.set(
        xlabel="prediction horizon [s]",
        ylabel="truth − prediction altitude error [m]",
        title=(
            "Vertical (NED-Z / altitude) error at matched sample indices\n"
            "positive value means ground truth is higher than prediction"),
    )
    axis.text(
        0.98, 0.03,
        f"terminal absolute={absolute_vertical_error[-1]:.3f} m\n"
        f"terminal start-aligned={propagation_vertical_error[-1]:.3f} m",
        transform=axis.transAxes, ha="right", va="bottom", fontsize=8.5,
        bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.85})
    axis.grid(alpha=0.3)
    axis.legend()
    fig.tight_layout()
    fig.savefig(output / "trajectory_vertical_error.png", dpi=160)
    plt.close(fig)

    mesh_contract = save_connected_3d_envelope(
        mean, truth, covariance_for_example, horizons, matched_indices,
        integration_contract, output)
    return {
        "array_index": index,
        "source_timestamp_us": int(timestamps[index]),
        "causal_point_count": point_count,
        "evaluated_horizon_s": horizon_s,
        "horizon_zero_alignment_error_m": alignment_error_m,
        "sample_index_contract": (
            f"prediction[i] and ground_truth[i] share time_offsets_s[i]; "
            f"i=0..{point_count - 1}"),
        "integration_contract": integration_contract,
        "connected_3d_envelope_contract": mesh_contract,
        "vertical_error_sign_contract": (
            "truth_altitude - prediction_altitude; positive means truth higher"),
        "terminal_horizontal_absolute_error_m": (
            terminal_horizontal_absolute_error_m),
        "terminal_absolute_error_m": terminal_absolute_error_m,
        "terminal_horizontal_propagation_error_m": (
            terminal_horizontal_propagation_error_m),
        "terminal_propagation_error_m": terminal_propagation_error_m,
        "terminal_vertical_propagation_error_m": float(
            relative_truth_altitude[-1] - relative_mean_altitude[-1]),
        "terminal_vertical_absolute_error_m": float(
            absolute_vertical_error[-1]),
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
              "trajectory_vertical_start_aligned.png",
              "trajectory_vertical_error.png", "trajectory_cone_mesh.png",
              "trajectory_cone_3d.png"]
             if example else []),
        "representative_cone": example,
    }
    (output_dir / "plot_manifest.json").write_text(
        json.dumps(manifest, indent=2), encoding="utf-8")
    print(json.dumps(manifest, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
