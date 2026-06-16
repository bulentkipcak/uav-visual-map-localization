#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, Optional, Sequence

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

from sau_sift_nav.vision_filter import VisualPositionKalman2D


MEAS_NOISE_CANDIDATES = [2.0, 3.0, 4.0, 5.0, 6.0]
ACCEL_NOISE_CANDIDATES = [0.2, 0.5, 1.0, 2.0]
MAX_INNOVATION_CANDIDATES = [2.0, 3.0, 4.0, 5.0]
MAX_PREDICT_AGE_CANDIDATES = [0.30, 0.40, 0.50]
MIN_INLIERS_CANDIDATES = [30, 40, 60, 80]


@dataclass
class Fix:
    timestamp: float
    receive_time: Optional[float]
    north: float
    east: float
    inliers: int
    truth_north: Optional[float]
    truth_east: Optional[float]
    source_file: str
    row_index: int


@dataclass
class TruthSample:
    timestamp: float
    north: float
    east: float


@dataclass
class ReplayConfig:
    meas_noise_pos_m: float
    process_accel_noise_mps2: float
    max_innovation_m: float
    max_predict_age_s: float
    min_inliers: int
    max_jump_m: float
    max_fix_age_s: float
    initial_pos_std_m: float
    initial_vel_std_mps: float


def parse_float(value: Any) -> Optional[float]:
    if value is None or value == "":
        return None
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(out):
        return None
    return out


def parse_int(value: Any) -> Optional[int]:
    parsed = parse_float(value)
    if parsed is None:
        return None
    return int(round(parsed))


def first_float(row: Dict[str, Any], names: Sequence[str]) -> Optional[float]:
    for name in names:
        value = parse_float(row.get(name))
        if value is not None:
            return value
    return None


def first_int(row: Dict[str, Any], names: Sequence[str]) -> Optional[int]:
    for name in names:
        value = parse_int(row.get(name))
        if value is not None:
            return value
    return None


def load_fixes(paths: Sequence[Path]) -> tuple[list[Fix], list[TruthSample]]:
    fixes: list[Fix] = []
    truths: list[TruthSample] = []
    seen_fixes: set[tuple[str, float, float, float, int]] = set()
    for path in paths:
        with path.open(newline="", encoding="utf-8", errors="replace") as f:
            rows = csv.DictReader(line for line in f if not line.lstrip().startswith("#"))
            for index, row in enumerate(rows, start=2):
                truth_timestamp = first_float(row, ("time", "timestamp", "kf_capture_time", "capture_time", "frame_capture_time"))
                truth_north = first_float(row, ("true_north_m", "gazebo_truth_N", "truth_N", "truth_north_m"))
                truth_east = first_float(row, ("true_east_m", "gazebo_truth_E", "truth_E", "truth_east_m"))
                if truth_timestamp is not None and truth_north is not None and truth_east is not None:
                    truths.append(TruthSample(timestamp=truth_timestamp, north=truth_north, east=truth_east))

                timestamp = first_float(row, ("sift_capture_time", "kf_capture_time", "capture_time", "frame_capture_time"))
                if timestamp is None:
                    timestamp = first_float(row, ("sift_receive_time", "kf_receive_time", "receive_time"))
                north = first_float(
                    row,
                    ("aligned_sift_north_m", "kf_aligned_north_m", "sift_aligned_N", "aligned_N", "pred_north_m"),
                )
                east = first_float(
                    row,
                    ("aligned_sift_east_m", "kf_aligned_east_m", "sift_aligned_E", "aligned_E", "pred_east_m"),
                )
                if timestamp is None or north is None or east is None:
                    continue
                receive_time = first_float(row, ("sift_receive_time", "kf_receive_time", "receive_time"))
                inliers = first_int(row, ("sift_inliers", "kf_inliers", "inliers", "MIN_INLIERS")) or 0
                dedupe_key = (str(path), round(timestamp, 6), round(north, 6), round(east, 6), inliers)
                if dedupe_key in seen_fixes:
                    continue
                seen_fixes.add(dedupe_key)
                fixes.append(
                    Fix(
                        timestamp=timestamp,
                        receive_time=receive_time,
                        north=north,
                        east=east,
                        inliers=inliers,
                        truth_north=truth_north,
                        truth_east=truth_east,
                        source_file=str(path),
                        row_index=index,
                    )
                )
    fixes.sort(key=lambda item: item.timestamp)
    truths.sort(key=lambda item: item.timestamp)
    return fixes, truths


def interpolate_truth(truths: Sequence[TruthSample], timestamp: float) -> Optional[tuple[float, float]]:
    if not truths:
        return None
    if timestamp < truths[0].timestamp or timestamp > truths[-1].timestamp:
        return None
    lo = 0
    hi = len(truths) - 1
    while lo <= hi:
        mid = (lo + hi) // 2
        if truths[mid].timestamp < timestamp:
            lo = mid + 1
        else:
            hi = mid - 1
    right = min(lo, len(truths) - 1)
    left = max(0, right - 1)
    a = truths[left]
    b = truths[right]
    if abs(b.timestamp - a.timestamp) < 1.0e-9:
        return a.north, a.east
    ratio = (timestamp - a.timestamp) / (b.timestamp - a.timestamp)
    north = a.north + ratio * (b.north - a.north)
    east = a.east + ratio * (b.east - a.east)
    return north, east


def quantile(values: Sequence[float], p: float) -> float:
    vals = sorted(v for v in values if math.isfinite(v))
    if not vals:
        return float("nan")
    idx = (len(vals) - 1) * p
    lo = math.floor(idx)
    hi = math.ceil(idx)
    if lo == hi:
        return vals[int(lo)]
    return vals[int(lo)] * (hi - idx) + vals[int(hi)] * (idx - lo)


def rms(values: Sequence[float]) -> float:
    vals = [v for v in values if math.isfinite(v)]
    if not vals:
        return float("nan")
    return math.sqrt(sum(v * v for v in vals) / len(vals))


def path_length(points: Sequence[tuple[float, float]]) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))


def make_publish_times(start: float, end: float, rate_hz: float) -> list[float]:
    if end < start:
        return []
    period = 1.0 / rate_hz
    count = int(math.floor((end - start) / period)) + 1
    return [start + i * period for i in range(count)]


def trajectory_metrics(
    timestamps: Sequence[float],
    points: Sequence[tuple[float, float]],
    truths: Sequence[TruthSample],
) -> Dict[str, float]:
    errors: list[float] = []
    north_errors: list[float] = []
    east_errors: list[float] = []
    aligned_points: list[tuple[float, float]] = []
    for timestamp, point in zip(timestamps, points):
        truth = interpolate_truth(truths, timestamp)
        if truth is None:
            continue
        dn = point[0] - truth[0]
        de = point[1] - truth[1]
        north_errors.append(dn)
        east_errors.append(de)
        errors.append(math.hypot(dn, de))
        aligned_points.append(point)
    steps = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(aligned_points, aligned_points[1:])]
    bias_north = sum(north_errors) / len(north_errors) if north_errors else float("nan")
    bias_east = sum(east_errors) / len(east_errors) if east_errors else float("nan")
    return {
        "median_error_m": quantile(errors, 0.5),
        "p95_error_m": quantile(errors, 0.95),
        "max_error_m": max(errors) if errors else float("nan"),
        "rms_error_m": rms(errors),
        "bias_north_m": bias_north,
        "bias_east_m": bias_east,
        "bias_norm_m": math.hypot(bias_north, bias_east)
        if math.isfinite(bias_north) and math.isfinite(bias_east)
        else float("nan"),
        "path_length_m": path_length(aligned_points),
        "jitter_median_m": quantile(steps, 0.5),
        "jitter_p95_m": quantile(steps, 0.95),
        "sample_count": float(len(errors)),
    }


def raw_baseline(fixes: Sequence[Fix], truths: Sequence[TruthSample]) -> Dict[str, float]:
    timestamps = [fix.timestamp for fix in fixes]
    points = [(fix.north, fix.east) for fix in fixes]
    metrics = trajectory_metrics(timestamps, points, truths)
    metrics["accepted_fix_count"] = float(len(fixes))
    metrics["rejected_fix_count"] = 0.0
    return metrics


def raw_hold_last_baseline(
    fixes: Sequence[Fix],
    truths: Sequence[TruthSample],
    publish_rate_hz: float,
    max_age_s: float,
) -> Dict[str, float]:
    if not fixes:
        return {}
    publish_times = make_publish_times(fixes[0].timestamp, fixes[-1].timestamp, publish_rate_hz)
    fix_index = 0
    last_fix: Optional[Fix] = None
    timestamps: list[float] = []
    points: list[tuple[float, float]] = []
    for publish_time in publish_times:
        while fix_index < len(fixes) and fixes[fix_index].timestamp <= publish_time:
            last_fix = fixes[fix_index]
            fix_index += 1
        if last_fix is None:
            continue
        if max_age_s > 0 and publish_time - last_fix.timestamp > max_age_s:
            continue
        timestamps.append(publish_time)
        points.append((last_fix.north, last_fix.east))
    metrics = trajectory_metrics(timestamps, points, truths)
    metrics["accepted_fix_count"] = float(len(fixes))
    metrics["rejected_fix_count"] = 0.0
    metrics["publish_count"] = float(len(points))
    return metrics


def load_validation_publish_dt(paths: Sequence[Path]) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for path in paths:
        with path.open(newline="", encoding="utf-8", errors="replace") as f:
            rows = csv.DictReader(line for line in f if not line.lstrip().startswith("#"))
            for row in rows:
                timestamp = first_float(row, ("time", "send_time", "vpe_publish_time"))
                publish_dt = first_float(row, ("vpe_publish_dt_s", "publish_dt_s"))
                if timestamp is None or publish_dt is None:
                    continue
                points.append((timestamp, publish_dt))
    points.sort(key=lambda item: item[0])
    return points


def replay_config(
    fixes: Sequence[Fix],
    truths: Sequence[TruthSample],
    config: ReplayConfig,
    publish_rate_hz: float,
    mode: str,
) -> tuple[Dict[str, float], list[Dict[str, Any]]]:
    if not fixes:
        return {}, []
    available_fixes = sorted(fixes, key=lambda fix: fix.receive_time if fix.receive_time is not None else fix.timestamp)
    kf = VisualPositionKalman2D(
        process_accel_noise=config.process_accel_noise_mps2,
        meas_noise_pos=config.meas_noise_pos_m,
        initial_pos_std=config.initial_pos_std_m,
        initial_vel_std=config.initial_vel_std_mps,
    )
    start_time = min(fix.receive_time if fix.receive_time is not None else fix.timestamp for fix in available_fixes)
    end_time = max(fix.receive_time if fix.receive_time is not None else fix.timestamp for fix in available_fixes)
    publish_times = make_publish_times(start_time, end_time, publish_rate_hz)
    fix_index = 0
    last_accepted: Optional[Fix] = None
    accepted = 0
    rejected = 0
    innovations: list[float] = []
    last_innovation: Optional[float] = None
    outputs: list[Dict[str, Any]] = []
    out_timestamps: list[float] = []
    out_points: list[tuple[float, float]] = []
    out_last_fix_ages: list[float] = []
    out_publish_dts: list[float] = []
    previous_output_time: Optional[float] = None

    for publish_time in publish_times:
        while (
            fix_index < len(available_fixes)
            and (available_fixes[fix_index].receive_time or available_fixes[fix_index].timestamp) <= publish_time
        ):
            fix = available_fixes[fix_index]
            fix_index += 1
            reason = ""
            innovation: Optional[float] = None
            if fix.inliers < config.min_inliers:
                reason = "low_inliers"
            elif fix.receive_time is not None and config.max_fix_age_s > 0 and fix.receive_time - fix.timestamp > config.max_fix_age_s:
                reason = "stale_fix"
            elif kf.initialized and kf.timestamp is not None and fix.timestamp < float(kf.timestamp) - 1.0e-3:
                reason = "out_of_order_fix"
            elif last_accepted is not None and config.max_jump_m > 0:
                jump = math.hypot(fix.north - last_accepted.north, fix.east - last_accepted.east)
                if jump > config.max_jump_m:
                    reason = "jump_gate"
            if not reason and kf.initialized:
                innovation = kf.innovation(fix.north, fix.east, fix.timestamp)
                if innovation is not None and config.max_innovation_m > 0 and innovation > config.max_innovation_m:
                    reason = "innovation_gate"
            if reason:
                rejected += 1
                continue
            update = kf.update(fix.north, fix.east, fix.timestamp)
            innovation = float(update.get("innovation_m") or 0.0)
            last_innovation = innovation
            innovations.append(innovation)
            accepted += 1
            last_accepted = fix

        if not kf.initialized or last_accepted is None:
            continue
        last_age = publish_time - last_accepted.timestamp
        if config.max_predict_age_s > 0 and last_age > config.max_predict_age_s:
            continue
        pred = kf.predict_at(publish_time)
        if pred is None:
            continue
        truth = interpolate_truth(truths, publish_time)
        error = None if truth is None else math.hypot(pred.north - truth[0], pred.east - truth[1])
        publish_dt = None if previous_output_time is None else publish_time - previous_output_time
        previous_output_time = publish_time
        if publish_dt is not None:
            out_publish_dts.append(publish_dt)
        outputs.append(
            {
                "timestamp": publish_time,
                "north": pred.north,
                "east": pred.east,
                "vn": pred.vn,
                "ve": pred.ve,
                "last_fix_age_s": last_age,
                "last_innovation_m": last_innovation,
                "publish_dt_s": publish_dt,
                "truth_north": None if truth is None else truth[0],
                "truth_east": None if truth is None else truth[1],
                "error_m": error,
            }
        )
        out_timestamps.append(publish_time)
        out_points.append((pred.north, pred.east))
        out_last_fix_ages.append(last_age)

    metrics = trajectory_metrics(out_timestamps, out_points, truths)
    total_fixes = accepted + rejected
    period = 1.0 / publish_rate_hz if publish_rate_hz > 0 else float("nan")
    publish_jitters = [abs(dt - period) for dt in out_publish_dts if math.isfinite(dt) and math.isfinite(period)]
    replay_duration_s = max(1.0e-6, publish_times[-1] - publish_times[0]) if len(publish_times) >= 2 else float("nan")
    effective_publish_rate_hz = float(len(outputs) / replay_duration_s) if math.isfinite(replay_duration_s) else float("nan")
    metrics.update(
        {
            "accepted_fix_count": float(accepted),
            "rejected_fix_count": float(rejected),
            "reject_ratio": float(rejected / total_fixes) if total_fixes else float("nan"),
            "innovation_median_m": quantile(innovations, 0.5),
            "innovation_p95_m": quantile(innovations, 0.95),
            "last_fix_age_median_s": quantile(out_last_fix_ages, 0.5),
            "last_fix_age_p95_s": quantile(out_last_fix_ages, 0.95),
            "publish_dt_median_s": quantile(out_publish_dts, 0.5),
            "publish_dt_p95_s": quantile(out_publish_dts, 0.95),
            "publish_jitter_p95_s": quantile(publish_jitters, 0.95),
            "publish_count": float(len(outputs)),
            "replay_duration_s": replay_duration_s,
            "effective_publish_rate_hz": effective_publish_rate_hz,
            "score": score(metrics, mode),
        }
    )
    return metrics, outputs


def score(metrics: Dict[str, float], mode: str) -> float:
    median_error = metrics.get("median_error_m", float("nan"))
    p95_error = metrics.get("p95_error_m", float("nan"))
    max_error = metrics.get("max_error_m", float("nan"))
    path = metrics.get("path_length_m", float("nan"))
    jitter_p95 = metrics.get("jitter_p95_m", float("nan"))
    if any(math.isnan(v) for v in (median_error, p95_error, max_error, path, jitter_p95)):
        return float("inf")
    if mode == "motion_mode":
        return median_error * 1.0 + p95_error * 3.0 + max_error * 0.25 + path * 0.005 + jitter_p95 * 0.5
    return median_error * 1.0 + p95_error * 2.0 + max_error * 0.1 + path * 0.05 + jitter_p95 * 1.0


def write_csv(path: Path, rows: Sequence[Dict[str, Any]], fieldnames: Sequence[str]) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in fieldnames})


def write_best_config(path: Path, row: Dict[str, Any]) -> None:
    config = {
        "KF_MEAS_NOISE_POS_M": float(row["meas_noise_pos_m"]),
        "KF_PROCESS_ACCEL_NOISE_MPS2": float(row["process_accel_noise_mps2"]),
        "KF_INITIAL_POS_STD_M": float(row["initial_pos_std_m"]),
        "KF_INITIAL_VEL_STD_MPS": float(row["initial_vel_std_mps"]),
        "MAX_INNOVATION_M": float(row["max_innovation_m"]),
        "MAX_JUMP_M": float(row["max_jump_m"]),
        "MAX_PREDICT_AGE_S": float(row["max_predict_age_s"]),
        "MAX_FIX_AGE_S": float(row["max_fix_age_s"]),
        "MIN_INLIERS": int(float(row["min_inliers"])),
    }
    path.write_text(json.dumps(config, indent=2), encoding="utf-8")


def row_to_live_config(row: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "KF_MEAS_NOISE_POS_M": float(row["meas_noise_pos_m"]),
        "KF_PROCESS_ACCEL_NOISE_MPS2": float(row["process_accel_noise_mps2"]),
        "KF_INITIAL_POS_STD_M": float(row["initial_pos_std_m"]),
        "KF_INITIAL_VEL_STD_MPS": float(row["initial_vel_std_mps"]),
        "MAX_INNOVATION_M": float(row["max_innovation_m"]),
        "MAX_JUMP_M": float(row["max_jump_m"]),
        "MAX_PREDICT_AGE_S": float(row["max_predict_age_s"]),
        "MAX_FIX_AGE_S": float(row["max_fix_age_s"]),
        "MIN_INLIERS": int(float(row["min_inliers"])),
        "score": float(row["score"]),
    }


def select_live_test_top3(rows: Sequence[Dict[str, Any]]) -> list[Dict[str, Any]]:
    selected: list[Dict[str, Any]] = []
    seen_core: set[tuple[float, float, float, int]] = set()
    for row in rows:
        core = (
            float(row["meas_noise_pos_m"]),
            float(row["process_accel_noise_mps2"]),
            float(row["max_predict_age_s"]),
            int(float(row["min_inliers"])),
        )
        if core in seen_core:
            continue
        seen_core.add(core)
        selected.append(row_to_live_config(row))
        if len(selected) >= 3:
            return selected
    for row in rows:
        config = row_to_live_config(row)
        if config not in selected:
            selected.append(config)
        if len(selected) >= 3:
            break
    return selected


def maybe_plot(
    output_dir: Path,
    fixes: Sequence[Fix],
    truths: Sequence[TruthSample],
    outputs: Sequence[Dict[str, Any]],
    validation_publish_dt: Sequence[tuple[float, float]] = (),
) -> None:
    try:
        import matplotlib.pyplot as plt
    except Exception:
        print("matplotlib not available; skipping plots")
        return
    output_dir.mkdir(parents=True, exist_ok=True)
    truth_n = [t.north for t in truths]
    truth_e = [t.east for t in truths]
    raw_n = [f.north for f in fixes]
    raw_e = [f.east for f in fixes]
    kf_n = [float(o["north"]) for o in outputs]
    kf_e = [float(o["east"]) for o in outputs]
    times = [float(o["timestamp"]) - float(outputs[0]["timestamp"]) for o in outputs] if outputs else []
    errors = [float(o["error_m"]) for o in outputs if o.get("error_m") is not None]
    error_times = [
        float(o["timestamp"]) - float(outputs[0]["timestamp"])
        for o in outputs
        if o.get("error_m") is not None
    ] if outputs else []
    innovation_points = [
        (float(o["timestamp"]) - float(outputs[0]["timestamp"]), float(o["last_innovation_m"]))
        for o in outputs
        if o.get("last_innovation_m") is not None
    ] if outputs else []
    last_fix_age_points = [
        (float(o["timestamp"]) - float(outputs[0]["timestamp"]), float(o["last_fix_age_s"]))
        for o in outputs
        if o.get("last_fix_age_s") is not None
    ] if outputs else []
    replay_publish_dt_points = [
        (float(o["timestamp"]) - float(outputs[0]["timestamp"]), float(o["publish_dt_s"]))
        for o in outputs
        if o.get("publish_dt_s") is not None
    ] if outputs else []
    if validation_publish_dt:
        base_time = validation_publish_dt[0][0]
        publish_dt_points = [(timestamp - base_time, publish_dt) for timestamp, publish_dt in validation_publish_dt]
    else:
        publish_dt_points = replay_publish_dt_points

    plt.figure(figsize=(8, 8))
    plt.plot(truth_e, truth_n, label="Gazebo truth")
    plt.scatter(raw_e, raw_n, s=5, label="Raw SIFT")
    plt.plot(kf_e, kf_n, label="Kalman predicted")
    plt.axis("equal")
    plt.xlabel("East [m]")
    plt.ylabel("North [m]")
    plt.legend()
    plt.tight_layout()
    plt.savefig(output_dir / "trajectory.png")
    plt.close()

    plt.figure(figsize=(8, 8))
    plt.plot(truth_e, truth_n, label="Gazebo truth")
    plt.axis("equal")
    plt.xlabel("East [m]")
    plt.ylabel("North [m]")
    plt.legend()
    plt.tight_layout()
    plt.savefig(output_dir / "truth_trajectory.png")
    plt.close()

    plt.figure(figsize=(8, 8))
    plt.scatter(raw_e, raw_n, s=5, label="Raw SIFT")
    plt.axis("equal")
    plt.xlabel("East [m]")
    plt.ylabel("North [m]")
    plt.legend()
    plt.tight_layout()
    plt.savefig(output_dir / "raw_sift_trajectory.png")
    plt.close()

    plt.figure(figsize=(8, 8))
    plt.plot(kf_e, kf_n, label="Kalman predicted")
    plt.axis("equal")
    plt.xlabel("East [m]")
    plt.ylabel("North [m]")
    plt.legend()
    plt.tight_layout()
    plt.savefig(output_dir / "kalman_trajectory.png")
    plt.close()

    plt.figure(figsize=(9, 4))
    plt.plot(error_times, errors)
    plt.xlabel("Replay time [s]")
    plt.ylabel("Error [m]")
    plt.tight_layout()
    plt.savefig(output_dir / "error_vs_time.png")
    plt.close()

    plt.figure(figsize=(9, 4))
    if len(kf_n) >= 2:
        steps = [math.hypot(kf_n[i] - kf_n[i - 1], kf_e[i] - kf_e[i - 1]) for i in range(1, len(kf_n))]
        plt.plot(times[1:], steps)
    plt.xlabel("Replay time [s]")
    plt.ylabel("VPE step [m]")
    plt.tight_layout()
    plt.savefig(output_dir / "jitter_vs_time.png")
    plt.close()

    plt.figure(figsize=(9, 4))
    if innovation_points:
        plt.plot([p[0] for p in innovation_points], [p[1] for p in innovation_points])
    plt.xlabel("Replay time [s]")
    plt.ylabel("Innovation [m]")
    plt.tight_layout()
    plt.savefig(output_dir / "innovation_vs_time.png")
    plt.close()

    plt.figure(figsize=(9, 4))
    if last_fix_age_points:
        plt.plot([p[0] for p in last_fix_age_points], [p[1] for p in last_fix_age_points])
    plt.xlabel("Replay time [s]")
    plt.ylabel("Last fix age [s]")
    plt.tight_layout()
    plt.savefig(output_dir / "last_fix_age_vs_time.png")
    plt.close()

    plt.figure(figsize=(9, 4))
    if publish_dt_points:
        plt.plot([p[0] for p in publish_dt_points], [p[1] for p in publish_dt_points])
    plt.xlabel("Replay time [s]")
    plt.ylabel("VPE publish dt [s]")
    plt.tight_layout()
    plt.savefig(output_dir / "publish_dt_vs_time.png")
    plt.close()


def parse_float_list(value: str) -> list[float]:
    return [float(part.strip()) for part in value.split(",") if part.strip()]


def parse_int_list(value: str) -> list[int]:
    return [int(part.strip()) for part in value.split(",") if part.strip()]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Offline grid-search tuning for SIFT visual Kalman VPE")
    parser.add_argument("csv", nargs="+", type=Path, help="Input live_sift*.csv logs")
    parser.add_argument("--mode", choices=["hover_mode", "motion_mode"], default="hover_mode")
    parser.add_argument("--publish-rate-hz", type=float, default=10.0)
    parser.add_argument("--meas-noise", default=",".join(str(v) for v in MEAS_NOISE_CANDIDATES))
    parser.add_argument("--accel-noise", default=",".join(str(v) for v in ACCEL_NOISE_CANDIDATES))
    parser.add_argument("--max-innovation", default=",".join(str(v) for v in MAX_INNOVATION_CANDIDATES))
    parser.add_argument("--max-predict-age", default=",".join(str(v) for v in MAX_PREDICT_AGE_CANDIDATES))
    parser.add_argument("--min-inliers", default=",".join(str(v) for v in MIN_INLIERS_CANDIDATES))
    parser.add_argument("--max-jump-m", type=float, default=3.0)
    parser.add_argument("--max-fix-age-s", type=float, default=0.5)
    parser.add_argument("--initial-pos-std-m", type=float, default=2.0)
    parser.add_argument("--initial-vel-std-mps", type=float, default=1.0)
    parser.add_argument(
        "--min-effective-publish-rate-hz",
        type=float,
        default=0.0,
        help="If >0, configs below this replay output rate are excluded from best/top results",
    )
    parser.add_argument("--results-csv", type=Path, default=Path("kalman_tuning_results.csv"))
    parser.add_argument("--raw-baseline-csv", type=Path, default=Path("raw_baseline_metrics.csv"))
    parser.add_argument("--best-config-json", type=Path, default=Path("best_visual_kalman_config.json"))
    parser.add_argument("--top3-json", type=Path, default=Path("top3_visual_kalman_configs.json"))
    parser.add_argument("--best-replay-csv", type=Path, default=Path("best_kalman_replay.csv"))
    parser.add_argument("--plot-dir", type=Path, default=Path("tuning_plots"))
    parser.add_argument("--no-plots", action="store_true")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    fixes, truths = load_fixes(args.csv)
    if not fixes:
        raise SystemExit("No usable SIFT fixes found in input CSV(s)")
    if len(truths) < 2:
        raise SystemExit("Need at least two truth samples for interpolation")

    meas_candidates = parse_float_list(args.meas_noise)
    accel_candidates = parse_float_list(args.accel_noise)
    innovation_candidates = parse_float_list(args.max_innovation)
    predict_age_candidates = parse_float_list(args.max_predict_age)
    inlier_candidates = parse_int_list(args.min_inliers)
    raw = raw_baseline(fixes, truths)
    raw_hold = raw_hold_last_baseline(
        fixes,
        truths,
        publish_rate_hz=args.publish_rate_hz,
        max_age_s=max(predict_age_candidates) if predict_age_candidates else 0.0,
    )
    input_files = ";".join(str(p) for p in args.csv)
    raw_rows = [
        {
            "baseline": "raw_fix_times",
            "mode": args.mode,
            "input_files": input_files,
            "publish_rate_hz": "",
            "max_age_s": "",
            **raw,
        },
        {
            "baseline": "raw_hold_last_publish_times",
            "mode": args.mode,
            "input_files": input_files,
            "publish_rate_hz": args.publish_rate_hz,
            "max_age_s": max(predict_age_candidates) if predict_age_candidates else "",
            **raw_hold,
        },
    ]
    raw_fields = [
        "baseline",
        "mode",
        "input_files",
        "publish_rate_hz",
        "max_age_s",
        "median_error_m",
        "p95_error_m",
        "max_error_m",
        "rms_error_m",
        "bias_north_m",
        "bias_east_m",
        "bias_norm_m",
        "path_length_m",
        "jitter_median_m",
        "jitter_p95_m",
        "accepted_fix_count",
        "rejected_fix_count",
        "publish_count",
        "sample_count",
    ]
    write_csv(args.raw_baseline_csv, raw_rows, raw_fields)

    result_rows: list[Dict[str, Any]] = []
    best_outputs: list[Dict[str, Any]] = []

    total = (
        len(meas_candidates)
        * len(accel_candidates)
        * len(innovation_candidates)
        * len(predict_age_candidates)
        * len(inlier_candidates)
    )
    print(f"Loaded fixes={len(fixes)} truth_samples={len(truths)} configs={total}")

    for meas_noise in meas_candidates:
        for accel_noise in accel_candidates:
            for max_innovation in innovation_candidates:
                for max_predict_age in predict_age_candidates:
                    for min_inliers in inlier_candidates:
                        config = ReplayConfig(
                            meas_noise_pos_m=meas_noise,
                            process_accel_noise_mps2=accel_noise,
                            max_innovation_m=max_innovation,
                            max_predict_age_s=max_predict_age,
                            min_inliers=min_inliers,
                            max_jump_m=args.max_jump_m,
                            max_fix_age_s=args.max_fix_age_s,
                            initial_pos_std_m=args.initial_pos_std_m,
                            initial_vel_std_mps=args.initial_vel_std_mps,
                        )
                        metrics, outputs = replay_config(fixes, truths, config, args.publish_rate_hz, args.mode)
                        score_reason = ""
                        if (
                            args.min_effective_publish_rate_hz > 0.0
                            and float(metrics.get("effective_publish_rate_hz", 0.0)) < args.min_effective_publish_rate_hz
                        ):
                            metrics["score"] = float("inf")
                            score_reason = "effective_publish_rate_below_min"
                        row = {
                            "meas_noise_pos_m": meas_noise,
                            "process_accel_noise_mps2": accel_noise,
                            "max_innovation_m": max_innovation,
                            "max_predict_age_s": max_predict_age,
                            "min_inliers": min_inliers,
                            "max_jump_m": args.max_jump_m,
                            "max_fix_age_s": args.max_fix_age_s,
                            "initial_pos_std_m": args.initial_pos_std_m,
                            "initial_vel_std_mps": args.initial_vel_std_mps,
                            **metrics,
                            "score_reason": score_reason,
                            "raw_median_error_m": raw.get("median_error_m"),
                            "raw_p95_error_m": raw.get("p95_error_m"),
                            "raw_bias_north_m": raw.get("bias_north_m"),
                            "raw_bias_east_m": raw.get("bias_east_m"),
                            "raw_bias_norm_m": raw.get("bias_norm_m"),
                            "raw_path_length_m": raw.get("path_length_m"),
                            "raw_jitter_p95_m": raw.get("jitter_p95_m"),
                            "raw_hold_median_error_m": raw_hold.get("median_error_m"),
                            "raw_hold_p95_error_m": raw_hold.get("p95_error_m"),
                            "raw_hold_bias_north_m": raw_hold.get("bias_north_m"),
                            "raw_hold_bias_east_m": raw_hold.get("bias_east_m"),
                            "raw_hold_bias_norm_m": raw_hold.get("bias_norm_m"),
                            "raw_hold_path_length_m": raw_hold.get("path_length_m"),
                            "raw_hold_jitter_p95_m": raw_hold.get("jitter_p95_m"),
                        }
                        result_rows.append(row)
                        if not best_outputs or row["score"] < min(r["score"] for r in result_rows[:-1]):
                            best_outputs = outputs

    result_rows.sort(key=lambda row: float(row.get("score", float("inf"))))
    fieldnames = [
        "meas_noise_pos_m",
        "process_accel_noise_mps2",
        "max_innovation_m",
        "max_predict_age_s",
        "min_inliers",
        "max_jump_m",
        "max_fix_age_s",
        "initial_pos_std_m",
        "initial_vel_std_mps",
        "median_error_m",
        "p95_error_m",
        "max_error_m",
        "rms_error_m",
        "bias_north_m",
        "bias_east_m",
        "bias_norm_m",
        "path_length_m",
        "jitter_median_m",
        "jitter_p95_m",
        "accepted_fix_count",
        "rejected_fix_count",
        "reject_ratio",
        "innovation_median_m",
        "innovation_p95_m",
        "last_fix_age_median_s",
        "last_fix_age_p95_s",
        "publish_dt_median_s",
        "publish_dt_p95_s",
        "publish_jitter_p95_s",
        "publish_count",
        "replay_duration_s",
        "effective_publish_rate_hz",
        "score",
        "score_reason",
        "raw_median_error_m",
        "raw_p95_error_m",
        "raw_bias_north_m",
        "raw_bias_east_m",
        "raw_bias_norm_m",
        "raw_path_length_m",
        "raw_jitter_p95_m",
        "raw_hold_median_error_m",
        "raw_hold_p95_error_m",
        "raw_hold_bias_north_m",
        "raw_hold_bias_east_m",
        "raw_hold_bias_norm_m",
        "raw_hold_path_length_m",
        "raw_hold_jitter_p95_m",
    ]
    write_csv(args.results_csv, result_rows, fieldnames)
    if result_rows:
        write_best_config(args.best_config_json, result_rows[0])
        top3 = select_live_test_top3(result_rows)
        args.top3_json.write_text(json.dumps(top3, indent=2), encoding="utf-8")
        replay_fields = [
            "timestamp",
            "north",
            "east",
            "vn",
            "ve",
            "last_fix_age_s",
            "last_innovation_m",
            "publish_dt_s",
            "truth_north",
            "truth_east",
            "error_m",
        ]
        write_csv(args.best_replay_csv, best_outputs, replay_fields)

    print("\nRaw baseline:")
    for key in (
        "median_error_m",
        "p95_error_m",
        "max_error_m",
        "rms_error_m",
        "bias_north_m",
        "bias_east_m",
        "bias_norm_m",
        "path_length_m",
        "jitter_p95_m",
    ):
        print(f"  {key}: {raw.get(key, float('nan')):.4f}")
    print("\nRaw hold-last 10 Hz baseline:")
    for key in (
        "median_error_m",
        "p95_error_m",
        "max_error_m",
        "rms_error_m",
        "bias_north_m",
        "bias_east_m",
        "bias_norm_m",
        "path_length_m",
        "jitter_p95_m",
    ):
        print(f"  {key}: {raw_hold.get(key, float('nan')):.4f}")

    print("\nTop 10 configs:")
    for index, row in enumerate(result_rows[:10], start=1):
        print(
            f"{index:02d}. score={float(row['score']):.4f} "
            f"med={float(row['median_error_m']):.3f} p95={float(row['p95_error_m']):.3f} "
            f"path={float(row['path_length_m']):.3f} jitter95={float(row['jitter_p95_m']):.3f} "
            f"rej={float(row['reject_ratio']):.2f} age95={float(row['last_fix_age_p95_s']):.3f} "
            f"effHz={float(row['effective_publish_rate_hz']):.2f} "
            f"meas={row['meas_noise_pos_m']} accel={row['process_accel_noise_mps2']} "
            f"innov={row['max_innovation_m']} pred_age={row['max_predict_age_s']} "
            f"inliers={row['min_inliers']}"
        )

    print("\nLive-test top 3 written to:", args.top3_json)
    print("Best config written to:", args.best_config_json)
    print("Results written to:", args.results_csv)
    print("Raw baseline written to:", args.raw_baseline_csv)

    if not args.no_plots and best_outputs:
        maybe_plot(args.plot_dir, fixes, truths, best_outputs, load_validation_publish_dt(args.csv))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
