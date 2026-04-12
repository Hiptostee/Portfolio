#!/usr/bin/env python3
"""
Analyze Paesano trajectory-following experiment bags.

Expected bag layout:
    Trajectory Following Accuracy Experiment/bags/
        traj_straight_t1/
        traj_curve_t1/
        traj_complex_t1/
        ...

Outputs:
    Trajectory Following Accuracy Experiment/trajectory_results/
        summary.csv
        straight_overlay.png
        curve_overlay.png
        complex_overlay.png

Dependencies:
    pip install rosbags matplotlib numpy
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

import matplotlib.pyplot as plt
import numpy as np
from rosbags.highlevel import AnyReader


ROOT = Path(__file__).resolve().parent
BAGS_DIR = ROOT / "bags"
OUT_DIR = ROOT / "trajectory_results"


@dataclass
class TrialResult:
    name: str
    route: str
    completion_time_s: float | None
    final_position_error_m: float | None
    final_heading_error_rad: float | None
    final_heading_error_deg: float | None
    mean_cross_track_error_m: float | None
    max_cross_track_error_m: float | None
    path_length_m: float | None
    success: int


def route_name(name: str) -> str:
    if "straight" in name:
        return "straight"
    if "curve" in name:
        return "curve"
    if "complex" in name:
        return "complex"
    raise ValueError(f"Unrecognized bag name: {name}")


def yaw_from_orientation(orientation) -> float:
    siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def read_trial(bag_dir: Path) -> Dict[str, object]:
    mcap_files = sorted(bag_dir.glob("*.mcap"))
    if not mcap_files:
        raise FileNotFoundError(f"No mcap found in {bag_dir}")

    pose_records: List[Tuple[float, object]] = []
    path_records: List[Tuple[float, object]] = []
    cmd_records: List[Tuple[float, object]] = []

    with AnyReader([mcap_files[0]]) as reader:
        connections = [
            x for x in reader.connections
            if x.topic in {"/estimated_pose", "/path", "/cmd_vel"}
        ]
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            t = timestamp / 1e9
            if connection.topic == "/estimated_pose":
                pose_records.append((t, msg))
            elif connection.topic == "/path":
                path_records.append((t, msg))
            elif connection.topic == "/cmd_vel":
                cmd_records.append((t, msg))

    if not pose_records:
        raise RuntimeError(f"No /estimated_pose data in {bag_dir}")
    if not path_records:
        raise RuntimeError(f"No /path data in {bag_dir}")

    start_time = min(
        pose_records[0][0],
        path_records[0][0],
        cmd_records[0][0] if cmd_records else pose_records[0][0],
    )

    # Use the last non-empty published path.
    path_msg = None
    for _, msg in path_records:
        if getattr(msg, "poses", None):
            if len(msg.poses) > 0:
                path_msg = msg
    if path_msg is None:
        raise RuntimeError(f"No non-empty /path message in {bag_dir}")

    path_xy = np.array(
        [[float(p.pose.position.x), float(p.pose.position.y)] for p in path_msg.poses],
        dtype=float,
    )
    path_theta = np.array(
        [yaw_from_orientation(p.pose.orientation) for p in path_msg.poses],
        dtype=float,
    )

    pose_t = np.array([t - start_time for t, _ in pose_records], dtype=float)
    pose_xy = np.array(
        [[float(msg.pose.position.x), float(msg.pose.position.y)] for _, msg in pose_records],
        dtype=float,
    )
    pose_theta = np.array([yaw_from_orientation(msg.pose.orientation) for _, msg in pose_records], dtype=float)

    cmd_t = np.array([t - start_time for t, _ in cmd_records], dtype=float)
    cmd_mag = np.array(
        [
            math.sqrt(
                float(msg.linear.x) ** 2 +
                float(msg.linear.y) ** 2 +
                float(msg.angular.z) ** 2
            )
            for _, msg in cmd_records
        ],
        dtype=float,
    ) if cmd_records else np.array([], dtype=float)

    return {
        "pose_t": pose_t,
        "pose_xy": pose_xy,
        "pose_theta": pose_theta,
        "path_xy": path_xy,
        "path_theta": path_theta,
        "cmd_t": cmd_t,
        "cmd_mag": cmd_mag,
    }


def point_to_segment_distance(point: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    ab = b - a
    denom = float(np.dot(ab, ab))
    if denom <= 1e-12:
        return float(np.linalg.norm(point - a))
    t = float(np.dot(point - a, ab) / denom)
    t = min(1.0, max(0.0, t))
    projection = a + t * ab
    return float(np.linalg.norm(point - projection))


def min_distance_to_polyline(point: np.ndarray, polyline: np.ndarray) -> float:
    if len(polyline) == 1:
        return float(np.linalg.norm(point - polyline[0]))
    return min(
        point_to_segment_distance(point, polyline[i], polyline[i + 1])
        for i in range(len(polyline) - 1)
    )


def path_length(polyline: np.ndarray) -> float:
    if len(polyline) < 2:
        return 0.0
    diffs = np.diff(polyline, axis=0)
    return float(np.sum(np.linalg.norm(diffs, axis=1)))


def compute_completion_time(cmd_t: np.ndarray, cmd_mag: np.ndarray, threshold: float = 1e-3) -> float | None:
    if len(cmd_t) == 0 or len(cmd_mag) == 0:
        return None
    active = np.where(cmd_mag > threshold)[0]
    if len(active) == 0:
        return None
    return float(cmd_t[int(active[-1])] - cmd_t[int(active[0])])


def compute_metrics(name: str, route: str, data: Dict[str, object]) -> TrialResult:
    pose_xy = data["pose_xy"]
    pose_theta = data["pose_theta"]
    path_xy = data["path_xy"]
    path_theta = data["path_theta"]
    cmd_t = data["cmd_t"]
    cmd_mag = data["cmd_mag"]

    cross_track = np.array([min_distance_to_polyline(point, path_xy) for point in pose_xy], dtype=float)
    goal = path_xy[-1]
    final_position_error = float(np.linalg.norm(pose_xy[-1] - goal))
    final_heading_error_rad = float(abs(wrap_angle(float(pose_theta[-1] - path_theta[-1]))))
    final_heading_error_deg = float(math.degrees(final_heading_error_rad))
    completion_time = compute_completion_time(cmd_t, cmd_mag)
    length = path_length(path_xy)
    success = int(final_position_error <= 0.10)

    return TrialResult(
        name=name,
        route=route,
        completion_time_s=completion_time,
        final_position_error_m=final_position_error,
        final_heading_error_rad=final_heading_error_rad,
        final_heading_error_deg=final_heading_error_deg,
        mean_cross_track_error_m=float(np.mean(cross_track)),
        max_cross_track_error_m=float(np.max(cross_track)),
        path_length_m=length,
        success=success,
    )


def plot_overlay(route: str, trials: Sequence[Tuple[str, Dict[str, object]]]) -> None:
    plt.figure(figsize=(7, 7))
    colors = plt.rcParams["axes.prop_cycle"].by_key().get("color", [])
    for idx, (name, data) in enumerate(trials):
        path_xy = data["path_xy"]
        path_theta = data["path_theta"]
        pose_xy = data["pose_xy"]
        pose_theta = data["pose_theta"]
        color = colors[idx % len(colors)] if colors else None
        final_heading_error_deg = abs(math.degrees(wrap_angle(float(pose_theta[-1] - path_theta[-1]))))
        plt.plot(
            path_xy[:, 0],
            path_xy[:, 1],
            linestyle="--",
            linewidth=1.8,
            color=color,
            alpha=0.8,
            label=f"{name} planned",
        )
        plt.plot(
            pose_xy[:, 0],
            pose_xy[:, 1],
            linewidth=1.8,
            color=color,
            label=f"{name} actual (final hdg err={final_heading_error_deg:.2f} deg)",
        )
        plt.scatter([pose_xy[0, 0]], [pose_xy[0, 1]], s=30, marker="o")
        plt.scatter([pose_xy[-1, 0]], [pose_xy[-1, 1]], s=30, marker="x")

    plt.title(f"{route.capitalize()} path overlay")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.axis("equal")
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=8)
    plt.tight_layout()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    plt.savefig(OUT_DIR / f"{route}_overlay.png", dpi=180)
    plt.close()


def plot_cross_track_error(route: str, trials: Sequence[Tuple[str, Dict[str, object]]]) -> None:
    plt.figure(figsize=(9, 4.8))
    colors = plt.rcParams["axes.prop_cycle"].by_key().get("color", [])
    for idx, (name, data) in enumerate(trials):
        pose_t = data["pose_t"]
        pose_xy = data["pose_xy"]
        pose_theta = data["pose_theta"]
        path_xy = data["path_xy"]
        path_theta = data["path_theta"]
        color = colors[idx % len(colors)] if colors else None
        cross_track = np.array([min_distance_to_polyline(point, path_xy) for point in pose_xy], dtype=float)
        mean_error = float(np.mean(cross_track))
        final_heading_error_deg = abs(math.degrees(wrap_angle(float(pose_theta[-1] - path_theta[-1]))))
        plt.plot(
            pose_t,
            cross_track,
            linewidth=1.8,
            color=color,
            label=f"{name} (mean={mean_error:.3f} m, final hdg={final_heading_error_deg:.2f} deg)",
        )
        plt.axhline(mean_error, linestyle="--", linewidth=1.0, color=color, alpha=0.6)

    plt.title(f"{route.capitalize()} cross-track error over time")
    plt.xlabel("Time [s]")
    plt.ylabel("Cross-Track Error [m]")
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=8)
    plt.tight_layout()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    plt.savefig(OUT_DIR / f"{route}_cross_track_error.png", dpi=180)
    plt.close()


def write_summary(results: Iterable[TrialResult]) -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    out_csv = OUT_DIR / "summary.csv"
    with out_csv.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "name",
            "route",
            "completion_time_s",
            "final_position_error_m",
            "final_heading_error_rad",
            "final_heading_error_deg",
            "mean_cross_track_error_m",
            "max_cross_track_error_m",
            "path_length_m",
            "success",
        ])
        for r in results:
            writer.writerow([
                r.name,
                r.route,
                "" if r.completion_time_s is None else f"{r.completion_time_s:.6f}",
                "" if r.final_position_error_m is None else f"{r.final_position_error_m:.6f}",
                "" if r.final_heading_error_rad is None else f"{r.final_heading_error_rad:.6f}",
                "" if r.final_heading_error_deg is None else f"{r.final_heading_error_deg:.6f}",
                "" if r.mean_cross_track_error_m is None else f"{r.mean_cross_track_error_m:.6f}",
                "" if r.max_cross_track_error_m is None else f"{r.max_cross_track_error_m:.6f}",
                "" if r.path_length_m is None else f"{r.path_length_m:.6f}",
                r.success,
            ])


def main() -> None:
    if not BAGS_DIR.exists():
        raise SystemExit(f"Bag directory not found: {BAGS_DIR}")

    bag_dirs = sorted([p for p in BAGS_DIR.iterdir() if p.is_dir() and p.name.startswith("traj_")])
    if not bag_dirs:
        raise SystemExit(f"No trajectory bag folders found in {BAGS_DIR}")

    grouped: Dict[str, List[Tuple[str, Dict[str, object]]]] = {
        "straight": [],
        "curve": [],
        "complex": [],
    }
    results: List[TrialResult] = []

    for bag_dir in bag_dirs:
        route = route_name(bag_dir.name)
        data = read_trial(bag_dir)
        grouped[route].append((bag_dir.name, data))
        results.append(compute_metrics(bag_dir.name, route, data))

    for route, trials in grouped.items():
        if trials:
            plot_overlay(route, trials)
            plot_cross_track_error(route, trials)

    write_summary(results)
    print(f"Wrote results to {OUT_DIR}")


if __name__ == "__main__":
    main()
