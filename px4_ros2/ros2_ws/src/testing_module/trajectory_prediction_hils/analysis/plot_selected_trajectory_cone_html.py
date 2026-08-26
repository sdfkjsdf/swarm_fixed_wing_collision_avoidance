#!/usr/bin/env python3
"""Create one interactive 3-D cone view from a selected analyzed HILS run."""

import argparse
import html
import json
from pathlib import Path

import numpy as np


CHI_SQUARE_95_DF3 = 7.8147279
NED_TO_EAST_NORTH_ALTITUDE = np.asarray([
    [0.0, 1.0, 0.0],
    [1.0, 0.0, 0.0],
    [0.0, 0.0, -1.0],
])
PLOTLY_CDN = "https://cdn.jsdelivr.net/npm/plotly.js-dist-min@2.35.2/plotly.min.js"


def choose_representative(candidate_inputs: np.ndarray) -> int:
    """Match the representative-cone policy used by the PNG plotter."""
    lateral = np.abs(candidate_inputs[:, 3])
    vertical = np.abs(candidate_inputs[:, 2])
    if np.max(lateral) > 0.05:
        candidates = np.flatnonzero(lateral >= np.max(lateral) - 1.0e-6)
    elif np.max(vertical) > 0.05:
        candidates = np.flatnonzero(vertical >= np.max(vertical) - 1.0e-6)
    else:
        candidates = np.arange(candidate_inputs.shape[0])
    return int(candidates[len(candidates) // 2])


def trajectory_tangents(centers: np.ndarray) -> np.ndarray:
    """Estimate a unit path tangent at every prediction horizon."""
    if centers.shape[0] < 2:
        raise ValueError("interactive cone requires at least two trajectory points")
    tangents = []
    for index in range(centers.shape[0]):
        if index == 0:
            delta = centers[1] - centers[0]
        elif index == centers.shape[0] - 1:
            delta = centers[-1] - centers[-2]
        else:
            delta = centers[index + 1] - centers[index - 1]
        norm = np.linalg.norm(delta)
        if norm < 1.0e-9:
            raise ValueError(
                f"predicted mean contains a degenerate tangent at index {index}")
        tangents.append(delta / norm)
    return np.asarray(tangents)


def transported_normal_frames(tangents: np.ndarray) -> np.ndarray:
    """Parallel-transport a stable 2-D normal frame along the mean path."""
    frames = []
    previous_axis = None
    for tangent in tangents:
        if previous_axis is None:
            reference = np.asarray([0.0, 0.0, 1.0])
            if abs(np.dot(reference, tangent)) > 0.95:
                references = np.eye(3)
                reference = references[np.argmin(np.abs(references @ tangent))]
            axis_u = reference - np.dot(reference, tangent) * tangent
        else:
            axis_u = previous_axis - np.dot(previous_axis, tangent) * tangent
            if np.linalg.norm(axis_u) < 1.0e-9:
                references = np.eye(3)
                reference = references[np.argmin(np.abs(references @ tangent))]
                axis_u = reference - np.dot(reference, tangent) * tangent
        axis_u /= np.linalg.norm(axis_u)
        if previous_axis is not None and np.dot(axis_u, previous_axis) < 0.0:
            axis_u = -axis_u
        axis_v = np.cross(tangent, axis_u)
        axis_v /= np.linalg.norm(axis_v)
        frames.append(np.column_stack((axis_u, axis_v)))
        previous_axis = axis_u
    return np.asarray(frames)


def covariance_support_rings(
        mean_ned: np.ndarray, covariance_ned: np.ndarray,
        angular_samples: int = 37):
    """Sample path-transverse support points of every full 3-D ellipsoid."""
    if angular_samples < 4:
        raise ValueError("support ring requires at least four angular samples")
    if mean_ned.ndim != 2 or mean_ned.shape[1] != 3:
        raise ValueError("predicted mean must have shape (point, 3)")
    if covariance_ned.shape != (mean_ned.shape[0], 3, 3):
        raise ValueError("position covariance must have shape (point, 3, 3)")
    if not np.isfinite(mean_ned).all() or not np.isfinite(covariance_ned).all():
        raise ValueError("mean and covariance values must be finite")

    centers = mean_ned @ NED_TO_EAST_NORTH_ALTITUDE.T
    covariance_plot = np.einsum(
        "ab,nbc,dc->nad",
        NED_TO_EAST_NORTH_ALTITUDE,
        covariance_ned,
        NED_TO_EAST_NORTH_ALTITUDE,
    )
    tangents = trajectory_tangents(centers)
    normal_frames = transported_normal_frames(tangents)
    angles = np.linspace(0.0, 2.0 * np.pi, angular_samples)
    scale = np.sqrt(CHI_SQUARE_95_DF3)
    rings = []
    support_normals = []

    for center, covariance, tangent, frame in zip(
            centers, covariance_plot, tangents, normal_frames):
        covariance = 0.5 * (covariance + covariance.T)
        eigenvalues = np.linalg.eigvalsh(covariance)
        if eigenvalues[0] <= 1.0e-10:
            raise ValueError("position covariance must be positive definite")
        directions = (
            np.cos(angles)[:, np.newaxis] * frame[:, 0] +
            np.sin(angles)[:, np.newaxis] * frame[:, 1])
        denominators = np.sqrt(np.einsum(
            "ni,ij,nj->n", directions, covariance, directions))
        offsets = (
            scale * (directions @ covariance.T) /
            denominators[:, np.newaxis])
        ring = center[np.newaxis, :] + offsets

        inverse_covariance = np.linalg.inv(covariance)
        mahalanobis = np.einsum(
            "ni,ij,nj->n", offsets, inverse_covariance, offsets)
        if np.max(np.abs(mahalanobis - CHI_SQUARE_95_DF3)) > 1.0e-6:
            raise ValueError("support ring is not on the pointwise 3-D ellipsoid")
        surface_normals = offsets @ inverse_covariance.T
        if np.max(np.abs(surface_normals @ tangent)) > 1.0e-7:
            raise ValueError("support-ring normals are not path-transverse")
        if not np.allclose(ring[0], ring[-1], atol=1.0e-9):
            raise ValueError("support-ring seam is open")
        rings.append(ring)
        support_normals.append(directions)

    return (
        centers, np.asarray(rings), covariance_plot, tangents,
        np.asarray(support_normals))


def terminal_display_cap(
        center: np.ndarray, covariance: np.ndarray,
        terminal_tangent: np.ndarray, boundary_ring: np.ndarray,
        boundary_normals: np.ndarray, polar_samples: int = 13) -> np.ndarray:
    """Close the display shell with the outward-normal half final ellipsoid."""
    if polar_samples < 3:
        raise ValueError("terminal cap requires at least three polar samples")
    covariance = 0.5 * (covariance + covariance.T)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    if eigenvalues[0] <= 1.0e-10:
        raise ValueError("terminal position covariance must be positive definite")
    covariance_root = (
        eigenvectors @ np.diag(np.sqrt(eigenvalues)) @ eigenvectors.T)
    inverse_root = (
        eigenvectors @ np.diag(1.0 / np.sqrt(eigenvalues)) @ eigenvectors.T)

    terminal_tangent = np.asarray(terminal_tangent, dtype=float)
    terminal_tangent /= np.linalg.norm(terminal_tangent)
    if np.max(np.abs(boundary_normals @ terminal_tangent)) > 1.0e-9:
        raise ValueError("terminal support normals are not path-transverse")
    sphere_axis = inverse_root @ terminal_tangent
    axis_norm = np.linalg.norm(sphere_axis)
    if axis_norm < 1.0e-12:
        raise ValueError("terminal covariance has no forward support")
    sphere_axis /= axis_norm

    scale = np.sqrt(CHI_SQUARE_95_DF3)
    boundary_offsets = boundary_ring - center
    boundary_sphere = np.einsum(
        "ij,nj->ni", inverse_root, boundary_offsets / scale)
    boundary_sphere /= np.linalg.norm(
        boundary_sphere, axis=1, keepdims=True)
    if np.max(np.abs(boundary_sphere @ sphere_axis)) > 1.0e-7:
        raise ValueError("terminal support ring is not the cap boundary")

    beta = np.linspace(0.0, 0.5 * np.pi, polar_samples)
    unit_sphere = (
        np.cos(beta)[:, np.newaxis, np.newaxis] *
        boundary_sphere[np.newaxis, :, :] +
        np.sin(beta)[:, np.newaxis, np.newaxis] *
        sphere_axis[np.newaxis, np.newaxis, :])
    offsets = scale * np.einsum(
        "ij,abj->abi", covariance_root, unit_sphere)
    surface = center[np.newaxis, np.newaxis, :] + offsets
    surface[0] = boundary_ring
    surface[-1] = surface[-1, 0]
    offsets = surface - center

    inverse_covariance = np.linalg.inv(covariance)
    mahalanobis = np.einsum(
        "abi,ij,abj->ab", offsets, inverse_covariance, offsets)
    if np.max(np.abs(mahalanobis - CHI_SQUARE_95_DF3)) > 1.0e-6:
        raise ValueError("terminal cap is not on the final 3-D ellipsoid")
    surface_normals = np.einsum(
        "ij,abj->abi", inverse_covariance, offsets)
    outward_support = np.einsum(
        "abi,i->ab", surface_normals, terminal_tangent)
    if np.min(outward_support) < -1.0e-8:
        raise ValueError("terminal cap contains inward-normal points")
    if not np.array_equal(surface[0], boundary_ring):
        raise ValueError("terminal cap and display shell do not share nodes")
    if not np.allclose(surface[:, 0], surface[:, -1], atol=1.0e-9):
        raise ValueError("terminal-cap seam is open")
    return surface


def line_trace(
        xyz: np.ndarray, name: str, series: str, width: float = 2.0,
        show_legend: bool = False):
    return {
        "type": "scatter3d",
        "mode": "lines",
        "x": xyz[:, 0].tolist(),
        "y": xyz[:, 1].tolist(),
        "z": xyz[:, 2].tolist(),
        "name": name,
        "legendgroup": series,
        "showlegend": show_legend,
        "hoverinfo": "skip",
        "line": {"color": f"__{series}", "width": width},
    }


def trajectory_trace(
        xyz: np.ndarray, horizons: np.ndarray, name: str, series: str,
        symbol: str):
    return {
        "type": "scatter3d",
        "mode": "lines+markers",
        "x": xyz[:, 0].tolist(),
        "y": xyz[:, 1].tolist(),
        "z": xyz[:, 2].tolist(),
        "customdata": horizons.tolist(),
        "name": name,
        "legendgroup": series,
        "showlegend": True,
        "line": {"color": f"__{series}", "width": 5.0},
        "marker": {
            "color": f"__{series}", "size": 3.5, "symbol": symbol,
        },
        "hovertemplate": (
            f"{name}<br>t=%{{customdata:.1f}} s<br>"
            "East=%{x:.2f} m<br>North=%{y:.2f} m<br>Altitude=%{z:.2f} m"
            "<extra></extra>"),
    }


def swept_shell_mesh_trace(
        rings: np.ndarray, ring_normals: np.ndarray, cap: np.ndarray,
        terminal_covariance: np.ndarray, terminal_center: np.ndarray):
    """Build one outside-facing shell with a shared terminal-cap seam."""
    time_count, closed_columns, _ = rings.shape
    cap_rows = cap.shape[0]
    unique_columns = closed_columns - 1
    if cap.shape[1] != closed_columns:
        raise ValueError("shell and cap azimuth samples do not match")

    shell_vertices = rings[:, :unique_columns].reshape(-1, 3)
    cap_interior = cap[1:-1, :unique_columns].reshape(-1, 3)
    apex = cap[-1, 0][np.newaxis, :]
    vertices = np.concatenate((shell_vertices, cap_interior, apex), axis=0)
    terminal_inverse = np.linalg.inv(
        0.5 * (terminal_covariance + terminal_covariance.T))
    triangles = []
    face_groups = []

    def append_oriented(indices, expected_normal, group):
        first, second, third = indices
        face_normal = np.cross(
            vertices[second] - vertices[first],
            vertices[third] - vertices[first])
        if np.dot(face_normal, expected_normal) < 0.0:
            second, third = third, second
            face_normal = -face_normal
        if np.linalg.norm(face_normal) <= 1.0e-12:
            raise ValueError("display shell contains a degenerate triangle")
        triangles.append((first, second, third))
        face_groups.append(group)

    for time_index in range(time_count - 1):
        for column in range(unique_columns):
            next_column = (column + 1) % unique_columns
            current = time_index * unique_columns + column
            current_next = time_index * unique_columns + next_column
            following = (time_index + 1) * unique_columns + column
            following_next = (
                (time_index + 1) * unique_columns + next_column)
            expected = (
                ring_normals[time_index, column] +
                ring_normals[time_index, next_column] +
                ring_normals[time_index + 1, column] +
                ring_normals[time_index + 1, next_column])
            expected /= np.linalg.norm(expected)
            append_oriented((current, following, current_next), expected, "shell")
            append_oriented(
                (current_next, following, following_next), expected, "shell")

    boundary = (time_count - 1) * unique_columns + np.arange(unique_columns)
    cap_rows_indices = [boundary]
    interior_base = time_count * unique_columns
    for row in range(cap_rows - 2):
        cap_rows_indices.append(
            interior_base + row * unique_columns + np.arange(unique_columns))
    apex_index = vertices.shape[0] - 1

    def cap_normal(vertex_indices):
        offset = np.mean(vertices[list(vertex_indices)], axis=0) - terminal_center
        normal = terminal_inverse @ offset
        normal_norm = np.linalg.norm(normal)
        if normal_norm <= 1.0e-12:
            raise ValueError("terminal cap has an undefined face normal")
        return normal / normal_norm

    for row in range(len(cap_rows_indices) - 1):
        current_row = cap_rows_indices[row]
        following_row = cap_rows_indices[row + 1]
        for column in range(unique_columns):
            next_column = (column + 1) % unique_columns
            first_triangle = (
                int(current_row[column]), int(following_row[column]),
                int(current_row[next_column]))
            second_triangle = (
                int(current_row[next_column]), int(following_row[column]),
                int(following_row[next_column]))
            append_oriented(
                first_triangle, cap_normal(first_triangle), "cap")
            append_oriented(
                second_triangle, cap_normal(second_triangle), "cap")

    last_row = cap_rows_indices[-1]
    for column in range(unique_columns):
        next_column = (column + 1) % unique_columns
        triangle = (
            int(last_row[column]), apex_index, int(last_row[next_column]))
        append_oriented(triangle, cap_normal(triangle), "cap")

    triangles = np.asarray(triangles, dtype=int)
    return {
        "type": "mesh3d",
        "x": vertices[:, 0].tolist(),
        "y": vertices[:, 1].tolist(),
        "z": vertices[:, 2].tolist(),
        "i": triangles[:, 0].tolist(),
        "j": triangles[:, 1].tolist(),
        "k": triangles[:, 2].tolist(),
        "facecolor": [f"__{group}" for group in face_groups],
        "name": "display-only swept shell",
        "legendgroup": "shell",
        "showlegend": True,
        "hoverinfo": "skip",
        "opacity": 0.16,
        "flatshading": False,
    }


def load_selected_cone(input_dir: Path, requested_index):
    arrays_path = input_dir / "cone_arrays.npz"
    if not arrays_path.is_file():
        raise FileNotFoundError(f"missing analyzed cone arrays: {arrays_path}")
    arrays = np.load(arrays_path, allow_pickle=False)
    required = (
        "predicted_mean", "predicted_position_covariance", "ground_truth")
    missing = [name for name in required if name not in arrays.files]
    if missing:
        raise ValueError(f"cone_arrays.npz is missing: {', '.join(missing)}")

    predicted = arrays["predicted_mean"]
    covariance = arrays["predicted_position_covariance"]
    truth = arrays["ground_truth"]
    if predicted.ndim != 3 or predicted.shape[2] != 3:
        raise ValueError("predicted_mean must have shape (cone, point, 3)")
    if truth.shape != predicted.shape:
        raise ValueError("ground_truth must match predicted_mean")
    if covariance.shape != predicted.shape[:2] + (3, 3):
        raise ValueError("predicted covariance dimensions do not match trajectories")
    if predicted.shape[0] == 0:
        raise ValueError("cone_arrays.npz contains no cones")

    point_counts = (
        arrays["causal_point_count"]
        if "causal_point_count" in arrays.files
        else np.full(predicted.shape[0], predicted.shape[1]))
    candidates = (
        arrays["candidate_inputs"]
        if "candidate_inputs" in arrays.files
        else np.zeros((predicted.shape[0], 4)))
    if candidates.shape != (predicted.shape[0], 4):
        raise ValueError("candidate_inputs must have shape (cone, 4)")
    if np.asarray(point_counts).shape != (predicted.shape[0],):
        raise ValueError("causal_point_count must contain one value per cone")
    if "time_offsets_s" in arrays.files:
        if arrays["time_offsets_s"].shape != predicted.shape[:2]:
            raise ValueError("time_offsets_s dimensions do not match trajectories")
    if "source_timestamp_us" in arrays.files:
        if arrays["source_timestamp_us"].shape != (predicted.shape[0],):
            raise ValueError("source_timestamp_us must contain one value per cone")
    if requested_index is None:
        index = choose_representative(candidates)
    else:
        index = int(requested_index)
        if not 0 <= index < predicted.shape[0]:
            raise ValueError(
                f"--array-index must be in [0, {predicted.shape[0] - 1}]")

    point_count = int(point_counts[index])
    if not 2 <= point_count <= predicted.shape[1]:
        raise ValueError(f"invalid causal_point_count: {point_count}")
    horizons = (
        np.asarray(arrays["time_offsets_s"][index, :point_count], dtype=float)
        if "time_offsets_s" in arrays.files
        else np.arange(point_count, dtype=float) * 0.1)
    if not np.isfinite(horizons).all() or np.any(np.diff(horizons) <= 0.0):
        raise ValueError("time offsets must be finite and strictly increasing")
    if not np.isfinite(truth[index, :point_count]).all():
        raise ValueError("ground-truth trajectory must be finite")
    timestamp = (
        int(arrays["source_timestamp_us"][index])
        if "source_timestamp_us" in arrays.files else 0)
    return {
        "index": index,
        "timestamp": timestamp,
        "point_count": point_count,
        "mean_ned": predicted[index, :point_count],
        "truth_ned": truth[index, :point_count],
        "covariance_ned": covariance[index, :point_count],
        "horizons": horizons,
        "candidate": candidates[index],
    }


def build_plot(cone):
    mean, rings, covariance_plot, tangents, ring_normals = (
        covariance_support_rings(cone["mean_ned"], cone["covariance_ned"]))
    truth = cone["truth_ned"] @ NED_TO_EAST_NORTH_ALTITUDE.T
    cap = terminal_display_cap(
        mean[-1], covariance_plot[-1], tangents[-1], rings[-1],
        ring_normals[-1])

    point_count = cone["point_count"]
    section_indices = np.arange(0, point_count, 5, dtype=int)
    if section_indices[-1] != point_count - 1:
        section_indices = np.append(section_indices, point_count - 1)
    generator_indices = np.arange(0, rings.shape[1] - 1, 3, dtype=int)
    traces = [
        swept_shell_mesh_trace(
            rings, ring_normals, cap, covariance_plot[-1], mean[-1]),
        trajectory_trace(
            mean, cone["horizons"], "predicted mean", "predicted", "circle"),
        trajectory_trace(
            truth, cone["horizons"], "ground truth", "truth", "x"),
    ]
    for order, point in enumerate(section_indices):
        traces.append(line_trace(
            rings[point],
            "pointwise nominal 95% ellipsoid support samples", "ring",
            width=2.2, show_legend=(order == 0)))
    for angle in generator_indices:
        traces.append(line_trace(
            rings[:, angle], "display-only swept-shell generators",
            "connector", width=1.4))
    for order, row in enumerate(range(0, cap.shape[0], 2)):
        traces.append(line_trace(
            cap[row], "terminal display cap", "cap", width=2.0,
            show_legend=(order == 0)))
    for angle in range(0, cap.shape[1] - 1, 2):
        traces.append(line_trace(
            cap[:, angle], "terminal display cap", "cap", width=1.7))

    points = np.concatenate((
        mean, truth, rings.reshape(-1, 3), cap.reshape(-1, 3)), axis=0)
    spans = np.maximum(np.ptp(points, axis=0), 1.0)
    aspect_ratio = {
        "x": 1.0,
        "y": float(spans[1] / spans[0]),
        "z": float(3.0 * spans[2] / spans[0]),
    }
    mesh = traces[0]
    metadata = {
        "array_index": cone["index"],
        "source_timestamp_us": cone["timestamp"],
        "causal_point_count": point_count,
        "evaluated_horizon_s": float(cone["horizons"][-1]),
        "mesh_vertex_count": len(mesh["x"]),
        "mesh_face_count": len(mesh["i"]),
    }
    return traces, aspect_ratio, metadata


def standalone_document(traces, aspect_ratio, title: str) -> str:
    data_json = json.dumps(traces, separators=(",", ":"), allow_nan=False)
    aspect_json = json.dumps(aspect_ratio, separators=(",", ":"))
    document_title = html.escape(title, quote=True)
    return f'''<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<meta name="referrer" content="no-referrer">
<meta http-equiv="Content-Security-Policy" content="default-src 'none'; script-src 'unsafe-inline' 'unsafe-eval' 'wasm-unsafe-eval' blob: data: https://cdn.jsdelivr.net; style-src 'unsafe-inline'; img-src blob: data:; worker-src blob:; connect-src blob: data:; object-src 'none'; base-uri 'none'; form-action 'none'">
<title>{document_title}</title>
<style>
:root {{ color-scheme: light dark; }}
* {{ box-sizing: border-box; }}
html, body {{ margin: 0; color: CanvasText; background: Canvas; font-family: system-ui, sans-serif; }}
#trajectory-cone-interactive {{ width: 100%; height: 720px; color: CanvasText; }}
@media (max-width: 480px) {{ #trajectory-cone-interactive {{ height: 520px; }} }}
</style>
</head>
<body>
<div id="trajectory-cone-interactive" role="img" aria-label="Interactive three-dimensional display-only shell built from pointwise position-covariance ellipsoid support samples, with predicted and ground-truth paths and a terminal cap; it is not an exact ellipsoid-union boundary, simultaneous confidence tube, or Auto ACAS MASD reconstruction"></div>
<script src="{PLOTLY_CDN}"></script>
<script>
(() => {{
  const root = document.getElementById("trajectory-cone-interactive");
  const resolveColor = (value) => {{
    const probe = document.createElement("span");
    probe.style.color = value;
    root.appendChild(probe);
    const resolved = getComputedStyle(probe).color;
    probe.remove();
    return resolved;
  }};
  const colors = {{
    predicted: resolveColor("LinkText"),
    truth: resolveColor("CanvasText"),
    ring: resolveColor("AccentColor"),
    connector: resolveColor("GrayText"),
    cap: resolveColor("VisitedText"),
    shell: resolveColor("AccentColor"),
    foreground: resolveColor("CanvasText"),
    muted: resolveColor("GrayText"),
    grid: resolveColor("GrayText")
  }};
  const traces = {data_json};
  traces.forEach((trace) => {{
    if (trace.line?.color?.startsWith("__")) {{
      trace.line.color = colors[trace.line.color.slice(2)];
    }}
    if (trace.marker?.color?.startsWith("__")) {{
      trace.marker.color = colors[trace.marker.color.slice(2)];
    }}
    if (Array.isArray(trace.facecolor)) {{
      trace.facecolor = trace.facecolor.map((entry) =>
        entry.startsWith("__") ? colors[entry.slice(2)] : entry);
    }}
  }});
  const axis = (titleText) => ({{
    title: {{text: titleText, font: {{color: colors.foreground}}}},
    tickfont: {{color: colors.muted}},
    gridcolor: colors.grid,
    zerolinecolor: colors.grid,
    showbackground: false
  }});
  const layout = {{
    autosize: true,
    margin: {{l: 0, r: 0, t: 52, b: 0}},
    paper_bgcolor: "rgba(0,0,0,0)",
    plot_bgcolor: "rgba(0,0,0,0)",
    font: {{color: colors.foreground}},
    hoverlabel: {{font: {{color: colors.foreground}}}},
    legend: {{
      orientation: "h", x: 0.5, xanchor: "center", y: 1.02,
      yanchor: "bottom", font: {{color: colors.foreground}}
    }},
    scene: {{
      xaxis: axis("East [m]"),
      yaxis: axis("North [m]"),
      zaxis: axis("Altitude [m]"),
      aspectmode: "manual",
      aspectratio: {aspect_json},
      dragmode: "orbit",
      camera: {{
        eye: {{x: 0.58, y: -1.28, z: 0.62}},
        center: {{x: 0, y: 0, z: 0}},
        up: {{x: 0, y: 0, z: 1}}
      }}
    }},
    annotations: [{{
      xref: "paper", yref: "paper", x: 0.01, y: 0.01,
      xanchor: "left", yanchor: "bottom", showarrow: false,
      text: "Pointwise nominal 95% (χ², 3 DoF) · display-only swept shell and terminal display cap<br>Not an exact ellipsoid-union boundary, simultaneous confidence tube, or Auto ACAS/MASD reconstruction · Figure 8 visual inspiration only · Altitude display ×3 (values unchanged)",
      font: {{color: colors.muted}}
    }}],
    uirevision: "trajectory-cone-camera"
  }};
  const config = {{
    responsive: true,
    scrollZoom: true,
    displaylogo: false,
    displayModeBar: true,
    modeBarButtonsToRemove: ["select2d", "lasso2d"]
  }};
  Plotly.newPlot(root, traces, layout, config);
}})();
</script>
</body>
</html>
'''


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Create one on-demand interactive trajectory-cone HTML from an "
            "already analyzed HILS run. This command is not used by batch processing."))
    parser.add_argument(
        "--input", required=True,
        help="selected raw run directory containing cone_arrays.npz")
    parser.add_argument(
        "--output", required=True, help="standalone .html output path")
    parser.add_argument(
        "--array-index", type=int,
        help="optional cone array index; default uses the PNG representative policy")
    parser.add_argument(
        "--title", default="Interactive trajectory-cone diagnostic",
        help="browser document title")
    args = parser.parse_args()

    input_dir = Path(args.input)
    output_path = Path(args.output)
    if output_path.suffix.lower() != ".html":
        print("ERROR: --output must end with .html")
        return 2
    try:
        cone = load_selected_cone(input_dir, args.array_index)
        traces, aspect_ratio, metadata = build_plot(cone)
        document = standalone_document(traces, aspect_ratio, args.title)
    except (FileNotFoundError, ValueError, np.linalg.LinAlgError) as error:
        print(f"ERROR: {error}")
        return 2

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(document, encoding="utf-8")
    metadata["source"] = str(input_dir.resolve())
    metadata["output"] = str(output_path.resolve())
    metadata["probability_contract"] = (
        "pointwise nominal 95% position-covariance ellipsoids; the connected "
        "shell and cap are display-only and are not an exact ellipsoid-union "
        "boundary, simultaneous confidence tube, or Auto ACAS/MASD reconstruction")
    print(json.dumps(metadata, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
