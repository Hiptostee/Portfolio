#!/usr/bin/env python3
"""
Analyze Paesano velocity-control rosbag trials.

Expected bag layout:
    Paesano/documentation/bags/
        vel_forward_t1/
        vel_forward_t2/
        vel_forward_t3/
        vel_lateral_t1/
        ...

Outputs:
    Paesano/documentation/velocity_results/
        summary.csv
        forward_overlay.png
        lateral_overlay.png
        rotate_overlay.png

Dependencies:
    pip install rosbags matplotlib numpy
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

import matplotlib.pyplot as plt
import numpy as np
from rosbags.highlevel import AnyReader


ROOT = Path(__file__).resolve().parent
BAGS_DIR = ROOT / "bags"
OUT_DIR = ROOT / "velocity_results"


@dataclass
class TrialResult:
    name: str
    movement: str
    target: float
    rise_time_s: float | None
    settling_time_s: float | None
    steady_state_error: float | None
    overshoot_pct: float | None
    stop_decay_time_s: float | None
    stop_drift_m: float | None
    stop_encoder_error_ticks: float | None
    stop_encoder_error_m: float | None


def movement_axis_and_target(name: str) -> Tuple[str, str]:
    if "forward" in name:
        return "forward", "linear_x"
    if "lateral" in name:
        return "lateral", "linear_y"
    if "rotate" in name:
        return "rotate", "angular_z"
    raise ValueError(f"Unrecognized bag name: {name}")


def command_cap_for_movement(movement: str) -> float:
    if movement in {"forward", "lateral"}:
        return 0.35
    if movement == "rotate":
        return 1.5
    raise ValueError(f"Unrecognized movement: {movement}")


def extract_signal_from_msg(msg, signal: str) -> float:
    if signal == "linear_x":
        return float(msg.linear.x)
    if signal == "linear_y":
        return float(msg.linear.y)
    if signal == "angular_z":
        return float(msg.angular.z)
    raise ValueError(signal)


def extract_odom_signal(msg, signal: str) -> float:
    twist = msg.twist.twist
    if signal == "linear_x":
        return float(twist.linear.x)
    if signal == "linear_y":
        return float(twist.linear.y)
    if signal == "angular_z":
        return float(twist.angular.z)
    raise ValueError(signal)


def extract_position_signal(msg, movement: str) -> float:
    pose = msg.pose.pose.position
    if movement == "forward":
        return float(pose.x)
    if movement == "lateral":
        return float(pose.y)
    return float("nan")


def read_trial(bag_dir: Path) -> Dict[str, np.ndarray]:
    mcap_files = sorted(bag_dir.glob("*.mcap"))
    if not mcap_files:
        raise FileNotFoundError(f"No mcap found in {bag_dir}")

    cmd_records: List[Tuple[float, object]] = []
    odom_records: List[Tuple[float, object]] = []
    odom_filtered_records: List[Tuple[float, object]] = []
    encoder_records: List[Tuple[float, object]] = []

    with AnyReader([mcap_files[0]]) as reader:
        connections = [
            x for x in reader.connections
            if x.topic in {"/cmd_vel", "/odom", "/odom/filtered", "/wheel_encoders"}
        ]
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            t = timestamp / 1e9
            if connection.topic == "/cmd_vel":
                cmd_records.append((t, msg))
            elif connection.topic == "/odom":
                odom_records.append((t, msg))
            elif connection.topic == "/odom/filtered":
                odom_filtered_records.append((t, msg))
            elif connection.topic == "/wheel_encoders":
                encoder_records.append((t, msg))

    if not odom_records:
        raise RuntimeError(f"No /odom data in {bag_dir}")
    if not cmd_records:
        raise RuntimeError(f"No /cmd_vel data in {bag_dir}")

    start_time = min(
        odom_records[0][0],
        odom_filtered_records[0][0] if odom_filtered_records else odom_records[0][0],
        cmd_records[0][0],
    )

    return {
        "cmd_t": np.array([t - start_time for t, _ in cmd_records]),
        "cmd_msg": np.array([m for _, m in cmd_records], dtype=object),
        "odom_t": np.array([t - start_time for t, _ in odom_records]),
        "odom_msg": np.array([m for _, m in odom_records], dtype=object),
        "odom_filtered_t": np.array([t - start_time for t, _ in odom_filtered_records]),
        "odom_filtered_msg": np.array([m for _, m in odom_filtered_records], dtype=object),
        "enc_t": np.array([t - start_time for t, _ in encoder_records]),
        "enc_msg": np.array([m for _, m in encoder_records], dtype=object),
    }


def build_piecewise_command(sample_t: np.ndarray, cmd_t: np.ndarray, cmd_values: np.ndarray) -> np.ndarray:
    out = np.zeros_like(sample_t)
    idx = 0
    current = 0.0
    for i, t in enumerate(sample_t):
        while idx < len(cmd_t) and cmd_t[idx] <= t:
            current = cmd_values[idx]
            idx += 1
        out[i] = current
    return out


def align_sign(measured: np.ndarray, command: np.ndarray, cmd_values: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float]:
    signed_target = float(cmd_values[np.argmax(np.abs(cmd_values))])
    sign = np.sign(signed_target if signed_target != 0.0 else 1.0)
    return measured * sign, command * sign, abs(signed_target)


def step_interval(times: np.ndarray, command_aligned: np.ndarray) -> Tuple[int, int]:
    step_indices = np.where(command_aligned > 1e-6)[0]
    if len(step_indices) == 0:
        raise RuntimeError("No step interval found")
    step_start_idx = int(step_indices[0])

    step_end_idx = len(command_aligned) - 1
    for idx in range(step_start_idx + 1, len(command_aligned)):
        if command_aligned[idx] <= 1e-6:
            step_end_idx = idx
            break
    return step_start_idx, step_end_idx


def nearest_index(times: np.ndarray, target_t: float) -> int | None:
    if len(times) == 0:
        return None
    return int(np.argmin(np.abs(times - target_t)))


def first_time_at_or_above(times: np.ndarray, values: np.ndarray, threshold: float) -> float | None:
    indices = np.where(values >= threshold)[0]
    if len(indices) == 0:
        return None
    return float(times[indices[0]])


def last_time_outside_band(times: np.ndarray, values: np.ndarray, lower: float, upper: float) -> float | None:
    indices = np.where((values < lower) | (values > upper))[0]
    if len(indices) == 0:
        return 0.0
    last_index = int(indices[-1])
    if last_index >= len(times) - 1:
        return None
    return float(times[last_index + 1])


def compute_metrics(name: str, movement: str, signal: str, data: Dict[str, np.ndarray]) -> TrialResult:
    odom_t = data["odom_t"]
    odom_msg = data["odom_msg"]
    cmd_t = data["cmd_t"]
    cmd_msg = data["cmd_msg"]

    measured = np.array([extract_odom_signal(msg, signal) for msg in odom_msg])
    cmd_values = np.array([extract_signal_from_msg(msg, signal) for msg in cmd_msg])
    command = build_piecewise_command(odom_t, cmd_t, cmd_values)

    raw_target = float(np.max(np.abs(command)))
    if raw_target <= 1e-6:
        raise RuntimeError(f"No nonzero target command reconstructed for {name}")

    measured_aligned, command_aligned, _ = align_sign(measured, command, cmd_values)
    effective_command_aligned = np.clip(command_aligned, 0.0, command_cap_for_movement(movement))
    target = float(np.max(effective_command_aligned))
    step_start_idx, step_end_idx = step_interval(odom_t, effective_command_aligned)

    step_times = odom_t[step_start_idx:step_end_idx]
    step_values = measured_aligned[step_start_idx:step_end_idx]

    rise_time = None
    t10 = first_time_at_or_above(step_times, step_values, 0.1 * target)
    t90 = first_time_at_or_above(step_times, step_values, 0.9 * target)
    if t10 is not None and t90 is not None:
        rise_time = t90 - t10

    settling_time = None
    settle = last_time_outside_band(step_times, step_values, 0.95 * target, 1.05 * target)
    if settle is not None:
        settling_time = settle - float(step_times[0])

    steady_window_start = float(step_times[0]) + 0.6 * max(float(step_times[-1] - step_times[0]), 1e-6)
    steady_indices = np.where(step_times >= steady_window_start)[0]
    steady_state_error = None
    if len(steady_indices) > 0:
        steady_state_error = float(target - np.mean(step_values[steady_indices]))

    peak = float(np.max(step_values)) if len(step_values) else 0.0
    overshoot_pct = max(0.0, (peak - target) / target * 100.0)

    stop_decay_time = None
    stop_drift = None
    stop_encoder_error_ticks = None
    stop_encoder_error_m = None
    if step_end_idx < len(odom_t):
        stop_times = odom_t[step_end_idx:]
        stop_values = np.abs(measured_aligned[step_end_idx:])
        below = np.where(stop_values <= 0.05 * target)[0]
        if len(below) > 0:
            stop_decay_time = float(stop_times[below[0]] - odom_t[step_end_idx])

        if movement in {"forward", "lateral"}:
            positions = np.array([extract_position_signal(msg, movement) for msg in odom_msg])
            stop_positions = positions[step_end_idx:]
            if len(stop_positions) > 1:
                stop_drift = float(abs(stop_positions[-1] - stop_positions[0]))

            enc_t = data["enc_t"]
            enc_msg = data["enc_msg"]
            enc_idx = nearest_index(enc_t, odom_t[step_end_idx])
            if enc_idx is not None and len(enc_msg) > enc_idx:
                encoder_vectors = [np.array(msg.data[:4], dtype=float) for msg in enc_msg]
                ref = encoder_vectors[enc_idx]
                post_stop = encoder_vectors[enc_idx:]
                if post_stop:
                    mean_abs_deltas = [float(np.mean(np.abs(vec - ref))) for vec in post_stop]
                    stop_encoder_error_ticks = max(mean_abs_deltas)
                    distance_per_tick = (2.0 * math.pi * 0.0485) / 2882.0
                    stop_encoder_error_m = stop_encoder_error_ticks * distance_per_tick

    return TrialResult(
        name=name,
        movement=movement,
        target=target,
        rise_time_s=rise_time,
        settling_time_s=settling_time,
        steady_state_error=steady_state_error,
        overshoot_pct=overshoot_pct,
        stop_decay_time_s=stop_decay_time,
        stop_drift_m=stop_drift,
        stop_encoder_error_ticks=stop_encoder_error_ticks,
        stop_encoder_error_m=stop_encoder_error_m,
    )


def plot_overlay(movement: str, signal: str, trial_data: List[Tuple[str, Dict[str, np.ndarray]]]) -> None:
    plt.figure(figsize=(10, 5))
    command_cap = command_cap_for_movement(movement)
    for name, data in trial_data:
        odom_t = data["odom_t"]
        measured = np.array([extract_odom_signal(msg, signal) for msg in data["odom_msg"]])
        cmd_values = np.array([extract_signal_from_msg(msg, signal) for msg in data["cmd_msg"]])
        command = build_piecewise_command(odom_t, data["cmd_t"], cmd_values)
        measured_aligned, command_aligned, _ = align_sign(measured, command, cmd_values)
        command_for_plot = np.clip(command_aligned, 0.0, command_cap)
        target = float(np.max(command_for_plot))

        step_start_idx, step_end_idx = step_interval(odom_t, command_for_plot)
        step_times = odom_t[step_start_idx:step_end_idx]
        step_values = measured_aligned[step_start_idx:step_end_idx]
        steady_window_start = float(step_times[0]) + 0.6 * max(float(step_times[-1] - step_times[0]), 1e-6)
        steady_indices = np.where(step_times >= steady_window_start)[0]
        steady_state_error = None
        steady_state_mean = None
        if len(steady_indices) > 0:
            steady_state_mean = float(np.mean(step_values[steady_indices]))
            steady_state_error = float(target - steady_state_mean)

        measured_label = f"{name} measured"
        if steady_state_error is not None:
            unit = "m/s" if signal != "angular_z" else "rad/s"
            measured_label += f" (ss err={steady_state_error:.3f} {unit})"

        measured_line, = plt.plot(odom_t, measured_aligned, alpha=0.8, label=measured_label)
        cmd_line, = plt.step(odom_t, command_for_plot, where="post", linestyle="--", alpha=0.7, label=f"{name} cmd")

        if steady_state_mean is not None:
            plt.hlines(
                steady_state_mean,
                steady_window_start,
                float(step_times[-1]),
                colors=[measured_line.get_color()],
                linestyles=":",
                linewidth=1.5,
                alpha=0.9,
            )

    handles, labels = plt.gca().get_legend_handles_labels()
    cmd_handles = [h for h, label in zip(handles, labels) if label.endswith(" cmd")]
    cmd_labels = [label for label in labels if label.endswith(" cmd")]
    measured_handles = [h for h, label in zip(handles, labels) if not label.endswith(" cmd")]
    measured_labels = [label for label in labels if not label.endswith(" cmd")]

    paired_handles = []
    paired_labels = []
    for cmd_handle, cmd_label, measured_handle, measured_label in zip(
        cmd_handles, cmd_labels, measured_handles, measured_labels
    ):
        paired_handles.extend([cmd_handle, measured_handle])
        paired_labels.extend([cmd_label, measured_label])

    plt.title(f"{movement.capitalize()} velocity tracking")
    plt.xlabel("Time [s]")
    plt.ylabel({
        "linear_x": "Velocity [m/s]",
        "linear_y": "Velocity [m/s]",
        "angular_z": "Angular velocity [rad/s]",
    }[signal])
    plt.grid(True, alpha=0.3)
    plt.legend(paired_handles, paired_labels, fontsize=8, ncol=2)
    plt.tight_layout()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    plt.savefig(OUT_DIR / f"{movement}_overlay.png", dpi=180)
    plt.close()


def plot_stop_error_over_time(movement: str, signal: str, trial_data: List[Tuple[str, Dict[str, np.ndarray]]]) -> None:
    plt.figure(figsize=(10, 5))
    for name, data in trial_data:
        odom_t = data["odom_t"]
        odom_msg = data["odom_msg"]
        cmd_values = np.array([extract_signal_from_msg(msg, signal) for msg in data["cmd_msg"]])
        command = build_piecewise_command(odom_t, data["cmd_t"], cmd_values)
        measured = np.array([extract_odom_signal(msg, signal) for msg in odom_msg])
        measured_aligned, command_aligned, _ = align_sign(measured, command, cmd_values)
        step_start_idx, step_end_idx = step_interval(odom_t, command_aligned)

        if movement in {"forward", "lateral"}:
            positions = np.array([extract_position_signal(msg, movement) for msg in odom_msg])
            stop_positions = positions[step_end_idx:]
            if len(stop_positions) > 1:
                error = stop_positions - stop_positions[0]
                stop_time = odom_t[step_end_idx:] - odom_t[step_end_idx]
                plt.plot(stop_time, error, alpha=0.85, label=name)
        else:
            stop_values = measured_aligned[step_end_idx:]
            if len(stop_values) > 1:
                stop_time = odom_t[step_end_idx:] - odom_t[step_end_idx]
                plt.plot(stop_time, stop_values, alpha=0.85, label=name)

    plt.title(
        f"{movement.capitalize()} stop error over time"
        if movement in {"forward", "lateral"}
        else "Rotate stop angular velocity over time"
    )
    plt.xlabel("Time From Zero Command [s]")
    plt.ylabel(
        "Position Error [m]" if movement in {"forward", "lateral"} else "Angular Velocity [rad/s]"
    )
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=8)
    plt.tight_layout()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    filename = (
        f"{movement}_stop_error_time.png"
        if movement in {"forward", "lateral"}
        else "rotate_stop_velocity_time.png"
    )
    plt.savefig(OUT_DIR / filename, dpi=180)
    plt.close()


def write_summary(results: Iterable[TrialResult]) -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    out_csv = OUT_DIR / "summary.csv"
    with out_csv.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "name",
            "movement",
            "target",
            "rise_time_s",
            "settling_time_s",
            "steady_state_error",
            "overshoot_pct",
            "stop_decay_time_s",
            "stop_drift_m",
            "stop_encoder_error_ticks",
            "stop_encoder_error_m",
        ])
        for r in results:
            writer.writerow([
                r.name,
                r.movement,
                f"{r.target:.6f}",
                "" if r.rise_time_s is None else f"{r.rise_time_s:.6f}",
                "" if r.settling_time_s is None else f"{r.settling_time_s:.6f}",
                "" if r.steady_state_error is None else f"{r.steady_state_error:.6f}",
                "" if r.overshoot_pct is None else f"{r.overshoot_pct:.6f}",
                "" if r.stop_decay_time_s is None else f"{r.stop_decay_time_s:.6f}",
                "" if r.stop_drift_m is None or math.isnan(r.stop_drift_m) else f"{r.stop_drift_m:.6f}",
                "" if r.stop_encoder_error_ticks is None else f"{r.stop_encoder_error_ticks:.6f}",
                "" if r.stop_encoder_error_m is None else f"{r.stop_encoder_error_m:.6f}",
            ])


def main() -> None:
    if not BAGS_DIR.exists():
        raise SystemExit(f"Bag directory not found: {BAGS_DIR}")

    bag_dirs = sorted([p for p in BAGS_DIR.iterdir() if p.is_dir() and p.name.startswith("vel_")])
    if not bag_dirs:
        raise SystemExit(f"No velocity bag folders found in {BAGS_DIR}")

    grouped: Dict[str, List[Tuple[str, Dict[str, np.ndarray]]]] = {
        "forward": [],
        "lateral": [],
        "rotate": [],
    }
    results: List[TrialResult] = []
    skipped: List[Tuple[str, str]] = []

    for bag_dir in bag_dirs:
        movement, signal = movement_axis_and_target(bag_dir.name)
        data = read_trial(bag_dir)
        try:
            results.append(compute_metrics(bag_dir.name, movement, signal, data))
            grouped[movement].append((bag_dir.name, data))
        except RuntimeError as exc:
            skipped.append((bag_dir.name, str(exc)))

    if grouped["forward"]:
        plot_overlay("forward", "linear_x", grouped["forward"])
    if grouped["lateral"]:
        plot_overlay("lateral", "linear_y", grouped["lateral"])
    if grouped["rotate"]:
        plot_overlay("rotate", "angular_z", grouped["rotate"])

    if grouped["forward"]:
        plot_stop_error_over_time("forward", "linear_x", grouped["forward"])
    if grouped["lateral"]:
        plot_stop_error_over_time("lateral", "linear_y", grouped["lateral"])
    if grouped["rotate"]:
        plot_stop_error_over_time("rotate", "angular_z", grouped["rotate"])

    write_summary(results)

    print(f"Wrote results to {OUT_DIR}")
    if skipped:
        print("\nSkipped trials:")
        for name, reason in skipped:
            print(f"  - {name}: {reason}")


if __name__ == "__main__":
    main()
