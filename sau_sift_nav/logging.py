from __future__ import annotations

import csv
import json
import math
import os
from datetime import datetime
from pathlib import Path
from typing import Any, Dict


def write_log_metadata(path: str | None, metadata: Dict[str, Any]) -> None:
    if not path:
        return
    log_path = Path(path)
    if log_path.exists() and log_path.stat().st_size > 0:
        return
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w", newline="", encoding="utf-8") as f:
        f.write("# SAU SIFT NAV LOG\n")
        f.write(f"# created_at,{datetime.now().isoformat(timespec='seconds')}\n")
        for key in sorted(metadata):
            value = metadata[key]
            try:
                encoded = json.dumps(value, ensure_ascii=False, sort_keys=True)
            except TypeError:
                encoded = json.dumps(str(value), ensure_ascii=False)
            f.write(f"# {key},{encoded}\n")
        f.write("# ---\n")


def _has_csv_header(path: str, fieldnames: list[str]) -> bool:
    if not os.path.exists(path) or os.path.getsize(path) == 0:
        return False
    with open(path, newline="", encoding="utf-8", errors="replace") as f:
        for line in f:
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            reader = csv.reader([line])
            return next(reader, []) == fieldnames
    return False


def _open_log_for_append(path: str):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    return open(path, "a", newline="", encoding="utf-8")


def append_match_log(path: str, result: Dict[str, Any], snapshot: Dict[str, Any]) -> None:
    fieldnames = [
        "timestamp",
        "status",
        "frame_id",
        "scale_name",
        "tile_img",
        "global_x_1x",
        "global_y_1x",
        "pred_north_m",
        "pred_east_m",
        "true_north_m",
        "true_east_m",
        "truth_source",
        "truth_status",
        "truth_age_sec",
        "truth_pose_age_sec",
        "truth_gz_x",
        "truth_gz_y",
        "truth_gz_z",
        "error_m",
        "calc_error_m",
        "gazebo_error_m",
        "nav_north_m",
        "nav_east_m",
        "nav_raw_north_m",
        "nav_raw_east_m",
        "nav_filter_residual_m",
        "nav_filter_reset",
        "nav_gazebo_error_m",
        "vision_tx_status",
        "vision_tx_sent_count",
        "vision_tx_rate_hz",
        "vision_tx_age_sec",
        "vision_tx_north_m",
        "vision_tx_east_m",
        "vision_tx_map_north_m",
        "vision_tx_map_east_m",
        "vision_timestamp_source",
        "vision_extra_predict_sec",
        "vision_prediction_horizon_sec",
        "vision_align_source",
        "vision_align_seed_source",
        "vision_align_offset_north_m",
        "vision_align_offset_east_m",
        "vision_align_age_sec",
        "kf_capture_time",
        "kf_receive_time",
        "kf_raw_north_m",
        "kf_raw_east_m",
        "kf_aligned_north_m",
        "kf_aligned_east_m",
        "kf_inliers",
        "kf_accepted",
        "kf_reject_reason",
        "kf_reset",
        "kf_reset_reason",
        "kf_jump_m",
        "kf_innovation_m",
        "kf_fix_age_s",
        "kf_north_m",
        "kf_east_m",
        "kf_vn_mps",
        "kf_ve_mps",
        "kf_update_count",
        "nav_status",
        "reject_reason",
        "reject_error_m",
        "reject_jump_m",
        "reject_nav_age_sec",
        "inliers",
        "good_count",
        "tiles_scanned",
        "tiles_considered",
        "frame_quad_width_m",
        "frame_quad_height_m",
        "frame_quad_area_m2",
        "search_source",
        "search_radius_m",
        "duration_sec",
    ]
    has_header = _has_csv_header(path, fieldnames)
    telemetry = snapshot.get("telemetry", {})
    telemetry_truth = telemetry.get("seed_ned", {}) if isinstance(telemetry, dict) else {}
    independent_truth = snapshot.get("truth", {}) if isinstance(snapshot, dict) else {}
    if (
        isinstance(independent_truth, dict)
        and independent_truth.get("fresh")
        and independent_truth.get("north") is not None
        and independent_truth.get("east") is not None
    ):
        truth = independent_truth
        truth_source = independent_truth.get("source", "gazebo")
    else:
        truth = telemetry_truth
        truth_source = telemetry_truth.get("source", "")
    pred = result.get("pred_ned", {}) if isinstance(result, dict) else {}
    logged_match = snapshot.get("match", {}) if isinstance(snapshot, dict) else {}
    nav = (snapshot.get("nav_estimate") or {}) if isinstance(snapshot, dict) else {}
    vision_tx = (snapshot.get("vision_tx") or {}) if isinstance(snapshot, dict) else {}
    vision_alignment = (snapshot.get("vision_alignment") or {}) if isinstance(snapshot, dict) else {}
    visual_kf = (snapshot.get("visual_kf") or {}) if isinstance(snapshot, dict) else {}
    kf_measurement = visual_kf.get("last_measurement", {}) if isinstance(visual_kf, dict) else {}
    reject = logged_match.get("reject", {}) if isinstance(logged_match, dict) else {}
    global_px = result.get("global_px_1x", ["", ""])
    search = result.get("search", {}) if isinstance(result, dict) else {}
    quad = result.get("frame_quad_metrics", {}) if isinstance(result, dict) else {}
    calc_error_m = ""
    gazebo_error_m = ""
    nav_gazebo_error_m = ""
    if pred.get("north") is not None and pred.get("east") is not None:
        if truth.get("north") is not None and truth.get("east") is not None:
            calc_error_m = math.hypot(
                float(pred["north"]) - float(truth["north"]),
                float(pred["east"]) - float(truth["east"]),
            )
        if independent_truth.get("north") is not None and independent_truth.get("east") is not None:
            gazebo_error_m = math.hypot(
                float(pred["north"]) - float(independent_truth["north"]),
                float(pred["east"]) - float(independent_truth["east"]),
            )
    if (
        nav.get("north") is not None
        and nav.get("east") is not None
        and independent_truth.get("north") is not None
        and independent_truth.get("east") is not None
    ):
        nav_gazebo_error_m = math.hypot(
            float(nav["north"]) - float(independent_truth["north"]),
            float(nav["east"]) - float(independent_truth["east"]),
        )

    row = {
        "timestamp": result.get("timestamp", ""),
        "status": result.get("status", ""),
        "frame_id": result.get("frame_id", ""),
        "scale_name": result.get("scale_name", ""),
        "tile_img": result.get("tile_img", ""),
        "global_x_1x": global_px[0] if len(global_px) > 0 else "",
        "global_y_1x": global_px[1] if len(global_px) > 1 else "",
        "pred_north_m": pred.get("north", ""),
        "pred_east_m": pred.get("east", ""),
        "true_north_m": truth.get("north", ""),
        "true_east_m": truth.get("east", ""),
        "truth_source": truth_source,
        "truth_status": independent_truth.get("status", "") if isinstance(independent_truth, dict) else "",
        "truth_age_sec": independent_truth.get("age_sec", "") if isinstance(independent_truth, dict) else "",
        "truth_pose_age_sec": independent_truth.get("pose_age_sec", "") if isinstance(independent_truth, dict) else "",
        "truth_gz_x": independent_truth.get("gz_x", "") if isinstance(independent_truth, dict) else "",
        "truth_gz_y": independent_truth.get("gz_y", "") if isinstance(independent_truth, dict) else "",
        "truth_gz_z": independent_truth.get("gz_z", "") if isinstance(independent_truth, dict) else "",
        "error_m": logged_match.get("error_m", ""),
        "calc_error_m": calc_error_m,
        "gazebo_error_m": gazebo_error_m,
        "nav_north_m": nav.get("north", ""),
        "nav_east_m": nav.get("east", ""),
        "nav_raw_north_m": nav.get("raw_north", ""),
        "nav_raw_east_m": nav.get("raw_east", ""),
        "nav_filter_residual_m": nav.get("filter_residual_m", ""),
        "nav_filter_reset": nav.get("filter_reset", ""),
        "nav_gazebo_error_m": nav_gazebo_error_m,
        "vision_tx_status": vision_tx.get("status", ""),
        "vision_tx_sent_count": vision_tx.get("sent_count", ""),
        "vision_tx_rate_hz": vision_tx.get("rate_hz", ""),
        "vision_tx_age_sec": vision_tx.get("age_sec", ""),
        "vision_tx_north_m": vision_tx.get("north", ""),
        "vision_tx_east_m": vision_tx.get("east", ""),
        "vision_tx_map_north_m": vision_tx.get("map_north", ""),
        "vision_tx_map_east_m": vision_tx.get("map_east", ""),
        "vision_timestamp_source": vision_tx.get("vision_timestamp_source", vision_tx.get("timestamp_source", "")),
        "vision_extra_predict_sec": vision_tx.get("pose_extra_predict_sec", vision_tx.get("extra_predict_sec", "")),
        "vision_prediction_horizon_sec": vision_tx.get("prediction_horizon_sec", ""),
        "vision_align_source": vision_alignment.get("source", ""),
        "vision_align_seed_source": vision_alignment.get("seed_source", ""),
        "vision_align_offset_north_m": vision_alignment.get("offset_north", ""),
        "vision_align_offset_east_m": vision_alignment.get("offset_east", ""),
        "vision_align_age_sec": vision_alignment.get("age_sec", ""),
        "kf_capture_time": kf_measurement.get("capture_time", ""),
        "kf_receive_time": kf_measurement.get("receive_time", ""),
        "kf_raw_north_m": kf_measurement.get("raw_north", ""),
        "kf_raw_east_m": kf_measurement.get("raw_east", ""),
        "kf_aligned_north_m": kf_measurement.get("aligned_north", ""),
        "kf_aligned_east_m": kf_measurement.get("aligned_east", ""),
        "kf_inliers": kf_measurement.get("inliers", ""),
        "kf_accepted": kf_measurement.get("accepted", ""),
        "kf_reject_reason": kf_measurement.get("reject_reason", ""),
        "kf_reset": kf_measurement.get("reset", ""),
        "kf_reset_reason": kf_measurement.get("reset_reason", ""),
        "kf_jump_m": kf_measurement.get("jump_m", ""),
        "kf_innovation_m": kf_measurement.get("innovation_m", ""),
        "kf_fix_age_s": kf_measurement.get("fix_age_s", ""),
        "kf_north_m": kf_measurement.get("kf_north", ""),
        "kf_east_m": kf_measurement.get("kf_east", ""),
        "kf_vn_mps": kf_measurement.get("kf_vn", ""),
        "kf_ve_mps": kf_measurement.get("kf_ve", ""),
        "kf_update_count": kf_measurement.get("kf_update_count", ""),
        "nav_status": logged_match.get("status", ""),
        "reject_reason": reject.get("reason", ""),
        "reject_error_m": reject.get("error_m", ""),
        "reject_jump_m": reject.get("jump_m", ""),
        "reject_nav_age_sec": reject.get("nav_age_sec", ""),
        "inliers": result.get("inliers", ""),
        "good_count": result.get("good_count", ""),
        "tiles_scanned": result.get("tiles_scanned", ""),
        "tiles_considered": result.get("tiles_considered", ""),
        "frame_quad_width_m": quad.get("width_m", ""),
        "frame_quad_height_m": quad.get("height_m", ""),
        "frame_quad_area_m2": quad.get("area_m2", ""),
        "search_source": search.get("source", ""),
        "search_radius_m": search.get("radius_m", ""),
        "duration_sec": result.get("duration_sec", ""),
    }

    with _open_log_for_append(path) as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        if not has_header:
            writer.writeheader()
        writer.writerow(row)


def append_vision_publish_log(path: str, record: Dict[str, Any]) -> None:
    fieldnames = [
        "send_time",
        "status",
        "reason",
        "sent_count",
        "vpe_usec",
        "vpe_N",
        "vpe_E",
        "kf_N",
        "kf_E",
        "kf_vN",
        "kf_vE",
        "last_fix_age_s",
        "publish_dt_s",
        "publish_mode",
        "stream_mode",
        "source",
        "frame_id",
        "reset_counter",
        "pos_ok",
        "speed_ok",
    ]
    has_header = _has_csv_header(path, fieldnames)
    row = {name: record.get(name, "") for name in fieldnames}
    with _open_log_for_append(path) as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        if not has_header:
            writer.writeheader()
        writer.writerow(row)


def append_validation_log(path: str, snapshot: Dict[str, Any], now: float) -> None:
    fieldnames = [
        "time",
        "true_north_m",
        "true_east_m",
        "raw_sift_north_m",
        "raw_sift_east_m",
        "aligned_sift_north_m",
        "aligned_sift_east_m",
        "sift_inliers",
        "sift_accepted",
        "sift_reject_reason",
        "sift_reset",
        "sift_reset_reason",
        "sift_jump_m",
        "sift_innovation_m",
        "sift_capture_time",
        "sift_receive_time",
        "kf_north_m",
        "kf_east_m",
        "kf_vnorth_mps",
        "kf_veast_mps",
        "kf_last_fix_age_s",
        "kf_predict_age_s",
        "kf_initialized",
        "vpe_publish_time",
        "vpe_north_m",
        "vpe_east_m",
        "vpe_usec",
        "vpe_publish_dt_s",
        "vpe_rate_hz",
        "vpe_publish_jitter_s",
        "vpe_status",
        "vpe_sent_count",
    ]
    truth = (snapshot.get("truth") or {}) if isinstance(snapshot, dict) else {}
    visual_kf = (snapshot.get("visual_kf") or {}) if isinstance(snapshot, dict) else {}
    measurement = visual_kf.get("last_measurement", {}) if isinstance(visual_kf, dict) else {}
    last_fix = visual_kf.get("last_fix", {}) if isinstance(visual_kf, dict) else {}
    vision_tx = (snapshot.get("vision_tx") or {}) if isinstance(snapshot, dict) else {}

    kf_last_fix_age = ""
    if isinstance(last_fix, dict) and last_fix.get("capture_time") is not None:
        kf_last_fix_age = max(0.0, now - float(last_fix["capture_time"]))

    kf_predict_age = ""
    if visual_kf.get("timestamp") is not None and visual_kf.get("prediction_timestamp") is not None:
        kf_predict_age = max(0.0, float(visual_kf["prediction_timestamp"]) - float(visual_kf["timestamp"]))

    publish_dt = vision_tx.get("publish_dt_s", "")
    rate_hz = vision_tx.get("rate_hz", "")
    publish_jitter = ""
    try:
        if publish_dt not in ("", None) and rate_hz not in ("", None) and float(rate_hz) > 0.0:
            publish_jitter = abs(float(publish_dt) - 1.0 / float(rate_hz))
    except (TypeError, ValueError):
        publish_jitter = ""

    row = {
        "time": now,
        "true_north_m": truth.get("north", ""),
        "true_east_m": truth.get("east", ""),
        "raw_sift_north_m": measurement.get("raw_north", ""),
        "raw_sift_east_m": measurement.get("raw_east", ""),
        "aligned_sift_north_m": measurement.get("aligned_north", ""),
        "aligned_sift_east_m": measurement.get("aligned_east", ""),
        "sift_inliers": measurement.get("inliers", ""),
        "sift_accepted": measurement.get("accepted", ""),
        "sift_reject_reason": measurement.get("reject_reason", ""),
        "sift_reset": measurement.get("reset", ""),
        "sift_reset_reason": measurement.get("reset_reason", ""),
        "sift_jump_m": measurement.get("jump_m", ""),
        "sift_innovation_m": measurement.get("innovation_m", ""),
        "sift_capture_time": measurement.get("capture_time", ""),
        "sift_receive_time": measurement.get("receive_time", ""),
        "kf_north_m": visual_kf.get("north", ""),
        "kf_east_m": visual_kf.get("east", ""),
        "kf_vnorth_mps": visual_kf.get("vn", ""),
        "kf_veast_mps": visual_kf.get("ve", ""),
        "kf_last_fix_age_s": kf_last_fix_age,
        "kf_predict_age_s": kf_predict_age,
        "kf_initialized": visual_kf.get("initialized", ""),
        "vpe_publish_time": vision_tx.get("publish_time", ""),
        "vpe_north_m": vision_tx.get("north", ""),
        "vpe_east_m": vision_tx.get("east", ""),
        "vpe_usec": vision_tx.get("vpe_usec", ""),
        "vpe_publish_dt_s": publish_dt,
        "vpe_rate_hz": rate_hz,
        "vpe_publish_jitter_s": publish_jitter,
        "vpe_status": vision_tx.get("status", ""),
        "vpe_sent_count": vision_tx.get("sent_count", ""),
    }
    has_header = _has_csv_header(path, fieldnames)
    with _open_log_for_append(path) as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        if not has_header:
            writer.writeheader()
        writer.writerow(row)
