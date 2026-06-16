from __future__ import annotations

import json
import math
import threading
import time
from typing import Any, Dict, Optional, Tuple

import cv2
import numpy as np

from .geometry import MapGeometry
from .vision_filter import VisualPositionKalman2D


class SharedState:
    def __init__(
        self,
        geometry: MapGeometry,
        base_search_radius_m: float,
        max_search_radius_m: float,
        search_growth: float,
        reset_jump_m: float = 5.0,
        min_nav_inliers: int = 80,
        max_nav_error_m: float = 15.0,
        max_nav_jump_m: float = 35.0,
        telemetry_position_gate: bool = True,
        telemetry_seed_source: str = "auto",
        telemetry_max_age_sec: float = 1.0,
        search_nav_max_age_sec: float = 3.0,
        visual_velocity_alpha: float = 0.35,
        max_visual_speed_mps: float = 5.0,
        nav_filter_alpha: float = 1.0,
        nav_filter_reset_residual_m: float = 5.0,
        max_nav_step_speed_mps: float = 0.0,
        max_nav_step_slack_m: float = 0.0,
        vision_align_source: str = "none",
        vision_align_max_age_sec: float = 1.0,
        vision_stream_mode: str = "event_based_raw",
        kf_process_noise_pos: float = 0.5,
        kf_process_noise_vel: float = 1.0,
        kf_process_accel_noise_mps2: Optional[float] = None,
        kf_meas_noise_pos: float = 4.0,
        kf_initial_pos_std_m: float = 2.0,
        kf_initial_vel_std_mps: float = 1.0,
        kf_min_inliers: int = 40,
        kf_max_jump_m: float = 3.0,
        kf_max_innovation_m: float = 3.0,
        kf_max_fix_age_sec: float = 0.5,
        kf_reset_on_high_inlier_jump: bool = False,
        kf_reset_min_inliers: int = 90,
        kf_reset_residual_m: float = 8.0,
    ) -> None:
        self.geometry = geometry
        self.lock = threading.Lock()
        self.start_time = time.time()
        self.frame_id = 0
        self.latest_frame: Optional[np.ndarray] = None
        self.latest_frame_jpeg: Optional[bytes] = None
        self.frame_time: Optional[float] = None
        self.match: Dict[str, Any] = {"status": "IDLE"}
        self.match_time: Optional[float] = None
        self.matcher_busy = False
        self.video_status = "starting"
        self.telemetry: Dict[str, Any] = {"status": "starting"}
        self.truth: Dict[str, Any] = {}
        self.vision_tx: Dict[str, Any] = {"status": "idle", "sent_count": 0}
        self.errors: list[str] = []
        self.base_search_radius_m = base_search_radius_m
        self.max_search_radius_m = max_search_radius_m
        self.search_growth = search_growth
        self.current_search_radius_m = base_search_radius_m
        self.reset_jump_m = reset_jump_m
        self.min_nav_inliers = min_nav_inliers
        self.max_nav_error_m = max_nav_error_m
        self.max_nav_jump_m = max_nav_jump_m
        self.telemetry_position_gate = telemetry_position_gate
        self.telemetry_seed_source = telemetry_seed_source
        self.telemetry_max_age_sec = max(0.0, telemetry_max_age_sec)
        self.search_nav_max_age_sec = max(0.0, search_nav_max_age_sec)
        self.visual_velocity_alpha = min(1.0, max(0.0, visual_velocity_alpha))
        self.max_visual_speed_mps = max(0.1, max_visual_speed_mps)
        self.nav_filter_alpha = min(1.0, max(0.0, nav_filter_alpha))
        self.nav_filter_reset_residual_m = max(0.0, nav_filter_reset_residual_m)
        self.max_nav_step_speed_mps = max(0.0, max_nav_step_speed_mps)
        self.max_nav_step_slack_m = max(0.0, max_nav_step_slack_m)
        self.vision_align_source = vision_align_source
        self.vision_align_max_age_sec = max(0.0, vision_align_max_age_sec)
        self.vision_stream_mode = vision_stream_mode
        self.vision_alignment: Optional[Dict[str, Any]] = None
        self.reset_counter = 0
        self.nav_estimate: Optional[Dict[str, Any]] = None
        self.visual_kf = VisualPositionKalman2D(
            process_noise_pos=kf_process_noise_pos,
            process_noise_vel=kf_process_noise_vel,
            process_accel_noise=kf_process_accel_noise_mps2,
            meas_noise_pos=kf_meas_noise_pos,
            initial_pos_std=kf_initial_pos_std_m,
            initial_vel_std=kf_initial_vel_std_mps,
        )
        self.kf_min_inliers = max(0, int(kf_min_inliers))
        self.kf_max_jump_m = max(0.0, float(kf_max_jump_m))
        self.kf_max_innovation_m = max(0.0, float(kf_max_innovation_m))
        self.kf_max_fix_age_sec = max(0.0, float(kf_max_fix_age_sec))
        self.kf_reset_on_high_inlier_jump = bool(kf_reset_on_high_inlier_jump)
        self.kf_reset_min_inliers = max(0, int(kf_reset_min_inliers))
        self.kf_reset_residual_m = max(0.0, float(kf_reset_residual_m))
        self.visual_kf_last_fix: Optional[Dict[str, Any]] = None
        self.visual_kf_last_measurement: Dict[str, Any] = {"accepted": False, "reject_reason": "waiting_fix"}

    def set_error(self, message: str) -> None:
        with self.lock:
            self.errors = (self.errors + [message])[-8:]

    def set_video_status(self, status: str) -> None:
        with self.lock:
            self.video_status = status

    def update_frame(self, frame: np.ndarray) -> None:
        ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 76])
        with self.lock:
            self.frame_id += 1
            self.latest_frame = frame.copy()
            self.latest_frame_jpeg = encoded.tobytes() if ok else None
            self.frame_time = time.time()

    def get_frame(self) -> Tuple[int, Optional[np.ndarray], Optional[float]]:
        with self.lock:
            if self.latest_frame is None:
                return self.frame_id, None, self.frame_time
            return self.frame_id, self.latest_frame.copy(), self.frame_time

    def get_jpeg(self) -> Optional[bytes]:
        with self.lock:
            return self.latest_frame_jpeg

    def get_jpeg_with_id(self) -> Tuple[int, Optional[bytes]]:
        with self.lock:
            return self.frame_id, self.latest_frame_jpeg

    def set_matcher_busy(self, busy: bool) -> None:
        with self.lock:
            self.matcher_busy = busy

    def update_match(self, match: Dict[str, Any]) -> None:
        now = time.time()
        estimate_time = _estimate_time_from_match(match, now)
        with self.lock:
            stored_match = dict(match)
            self.match_time = now
            if match.get("status") == "OK":
                stored_match["visual_kf_measurement"] = self._update_visual_kf_from_match_locked(
                    match,
                    estimate_time=estimate_time,
                    receive_time=now,
                )
                reject = self._nav_reject_reason(match, estimate_time)
                if reject:
                    stored_match["status"] = "REJECTED"
                    stored_match["original_status"] = "OK"
                    stored_match["reject"] = reject
                    self.match = stored_match
                    if reject.get("reason") in {
                        "telemetry_position_gate",
                        "prediction_jump_gate",
                        "visual_step_speed_gate",
                    }:
                        self.current_search_radius_m = self.base_search_radius_m
                    else:
                        self.current_search_radius_m = min(
                            self.max_search_radius_m,
                            max(self.base_search_radius_m, self.current_search_radius_m * self.search_growth),
                        )
                    return

                self.match = stored_match
                self.current_search_radius_m = self.base_search_radius_m
                self._set_nav_estimate_from_match(match, estimate_time)
            else:
                self.match = stored_match
                self.current_search_radius_m = min(
                    self.max_search_radius_m,
                    max(self.base_search_radius_m, self.current_search_radius_m * self.search_growth),
                )

    def update_telemetry(self, patch: Dict[str, Any]) -> None:
        with self.lock:
            merged = dict(self.telemetry)
            merged.update(patch)
            merged["updated_at"] = time.time()
            self.telemetry = merged

    def update_truth(self, patch: Dict[str, Any]) -> None:
        with self.lock:
            merged = dict(self.truth)
            merged.update(patch)
            merged["updated_at"] = time.time()
            self.truth = merged

    def update_vision_tx(self, patch: Dict[str, Any]) -> None:
        with self.lock:
            merged = dict(self.vision_tx)
            merged.update(patch)
            merged["updated_at"] = time.time()
            self.vision_tx = merged

    def search_hint(self, allow_gps_seed: bool = True) -> Optional[Dict[str, Any]]:
        with self.lock:
            telemetry = _json_copy(self.telemetry)
            nav = _json_copy(self.nav_estimate)
            radius = self.current_search_radius_m

        now = time.time()
        local = _fresh_telemetry_item(
            telemetry.get("local_ned") if isinstance(telemetry, dict) else None,
            now,
            self.telemetry_max_age_sec,
        )
        seed = self._telemetry_position(telemetry, now)

        if nav:
            velocity = (
                float(nav.get("vx") or 0.0),
                float(nav.get("vy") or 0.0),
            )
            age = max(0.0, now - float(nav["timestamp"]))
            if self.search_nav_max_age_sec <= 0.0 or age <= self.search_nav_max_age_sec:
                north = float(nav["north"]) + velocity[0] * age
                east = float(nav["east"]) + velocity[1] * age
                radius = min(self.max_search_radius_m, radius + _speed(velocity) * age + 5.0)
                px = self.geometry.ned_to_pixel(north, east)
                return {
                    "source": "vision_predicted",
                    "center_ned": [north, east],
                    "center_px_1x": [float(px[0]), float(px[1])],
                    "radius_m": radius,
                }

        if allow_gps_seed and seed:
            north = seed[0]
            east = seed[1]
            px = self.geometry.ned_to_pixel(north, east)
            return {
                "source": seed[2],
                "center_ned": [north, east],
                "center_px_1x": [float(px[0]), float(px[1])],
                "radius_m": radius,
            }

        return None

    def current_vision_pose(
        self,
        max_age_sec: float,
        allow_gps_seed: bool = False,
        velocity_source: str = "zero",
        predict_position: bool = True,
        extra_predict_sec: float = 0.0,
    ) -> Optional[Dict[str, Any]]:
        with self.lock:
            telemetry = _json_copy(self.telemetry)
            nav = _json_copy(self.nav_estimate)
            reset_counter = self.reset_counter

        now = time.time()
        local = _fresh_telemetry_item(
            telemetry.get("local_ned") if isinstance(telemetry, dict) else None,
            now,
            self.telemetry_max_age_sec,
        )
        seed = self._telemetry_position(telemetry, now)

        if nav:
            velocity = self._pose_velocity(nav, local, velocity_source)
            age = max(0.0, now - float(nav["timestamp"]))
            if age <= max_age_sec:
                north = float(nav["north"])
                east = float(nav["east"])
                prediction_horizon_sec = age
                if predict_position:
                    prediction_horizon_sec = age + max(0.0, extra_predict_sec)
                    north += velocity[0] * prediction_horizon_sec
                    east += velocity[1] * prediction_horizon_sec
                alignment = self._vision_output_alignment(north, east, telemetry, now)
                if alignment is None:
                    return None
                aligned_north = north + float(alignment.get("offset_north", 0.0))
                aligned_east = east + float(alignment.get("offset_east", 0.0))
                down = _local_down(local, default=float(nav.get("down", 0.0)))
                return {
                    "source": "vision_predicted",
                    "north": aligned_north,
                    "east": aligned_east,
                    "map_north": north,
                    "map_east": east,
                    "down": down,
                    "vx": velocity[0],
                    "vy": velocity[1],
                    "vz": velocity[2],
                    "age_sec": age,
                    "prediction_horizon_sec": prediction_horizon_sec,
                    "extra_predict_sec": max(0.0, extra_predict_sec) if predict_position else 0.0,
                    "nav_timestamp": nav.get("timestamp"),
                    "frame_id": nav.get("frame_id"),
                    "reset_counter": reset_counter,
                    "alignment": alignment,
                }

        if allow_gps_seed and seed:
            velocity = self._pose_velocity(None, local, velocity_source)
            return {
                "source": seed[2],
                "north": seed[0],
                "east": seed[1],
                "down": _local_down(local, default=0.0),
                "vx": velocity[0],
                "vy": velocity[1],
                "vz": velocity[2],
                "age_sec": 0.0,
                "reset_counter": reset_counter,
            }

        return None

    def current_kalman_vision_pose(self, max_age_sec: float) -> Optional[Dict[str, Any]]:
        now = time.time()
        with self.lock:
            telemetry = _json_copy(self.telemetry)
            reset_counter = self.reset_counter
            last_fix = _json_copy(self.visual_kf_last_fix)
            kf_snapshot = self.visual_kf.snapshot(now)
            pred = self.visual_kf.predict_at(now)

        if pred is None or not last_fix:
            return None

        last_capture_time = last_fix.get("capture_time")
        if last_capture_time is None:
            return None
        last_fix_age_sec = max(0.0, now - float(last_capture_time))
        if max_age_sec > 0.0 and last_fix_age_sec > max_age_sec:
            return None

        local = _fresh_telemetry_item(
            telemetry.get("local_ned") if isinstance(telemetry, dict) else None,
            now,
            self.telemetry_max_age_sec,
        )
        return {
            "source": "kalman_predicted",
            "north": pred.north,
            "east": pred.east,
            "map_north": last_fix.get("raw_north"),
            "map_east": last_fix.get("raw_east"),
            "down": _local_down(local, default=0.0),
            "vx": pred.vn,
            "vy": pred.ve,
            "vz": 0.0,
            "age_sec": last_fix_age_sec,
            "prediction_horizon_sec": last_fix_age_sec,
            "extra_predict_sec": 0.0,
            "nav_timestamp": None,
            "frame_id": last_fix.get("frame_id"),
            "reset_counter": reset_counter,
            "alignment": last_fix.get("alignment"),
            "kf": {
                **kf_snapshot,
                "prediction_timestamp": pred.timestamp,
                "last_fix_age_sec": last_fix_age_sec,
            },
        }

    def snapshot(self) -> Dict[str, Any]:
        with self.lock:
            now = time.time()
            frame_age = None if self.frame_time is None else now - self.frame_time
            match_age = None if self.match_time is None else now - self.match_time
            telemetry = _json_copy(self.telemetry)
            truth = _json_copy(self.truth)
            match = _json_copy(self.match)
            nav_estimate = _json_copy(self.nav_estimate)
            vision_alignment = _json_copy(self.vision_alignment)
            visual_kf = self.visual_kf.snapshot(now)
            visual_kf["last_fix"] = _json_copy(self.visual_kf_last_fix)
            visual_kf["last_measurement"] = _json_copy(self.visual_kf_last_measurement)
            frame_id = self.frame_id
            video_status = self.video_status
            matcher_busy = self.matcher_busy
            vision_tx = _json_copy(self.vision_tx)
            errors = list(self.errors)
            radius = self.current_search_radius_m

        if vision_alignment and vision_alignment.get("created_at") is not None:
            vision_alignment["age_sec"] = max(0.0, now - float(vision_alignment["created_at"]))
        _annotate_telemetry_age(telemetry, now, self.telemetry_max_age_sec)
        _annotate_truth_age(truth, now, self.telemetry_max_age_sec)
        if truth.get("north") is not None and truth.get("east") is not None:
            truth_px = self.geometry.ned_to_pixel(float(truth["north"]), float(truth["east"]))
            truth["global_px_1x"] = [float(truth_px[0]), float(truth_px[1])]
        seed = self._telemetry_position(telemetry, now)
        if seed:
            true_px = self.geometry.ned_to_pixel(seed[0], seed[1])
            telemetry["global_px_1x"] = [float(true_px[0]), float(true_px[1])]
            telemetry["seed_ned"] = {
                "north": seed[0],
                "east": seed[1],
                "source": seed[2],
                "age_sec": seed[3],
            }

        if match.get("status") == "OK" and seed and match.get("pred_ned"):
            pred = match["pred_ned"]
            dn = float(pred["north"]) - seed[0]
            de = float(pred["east"]) - seed[1]
            match["error_m"] = math.hypot(dn, de)
            match["error_north_m"] = dn
            match["error_east_m"] = de

        if match.get("status") == "OK" and match.get("pred_ned") and truth.get("fresh"):
            pred = match["pred_ned"]
            if pred.get("north") is not None and pred.get("east") is not None:
                if truth.get("north") is not None and truth.get("east") is not None:
                    dn = float(pred["north"]) - float(truth["north"])
                    de = float(pred["east"]) - float(truth["east"])
                    match["independent_error_m"] = math.hypot(dn, de)
                    match["independent_error_north_m"] = dn
                    match["independent_error_east_m"] = de

        return {
            "uptime_sec": now - self.start_time,
            "frame": {"id": frame_id, "age_sec": frame_age},
            "video_status": video_status,
            "matcher_busy": matcher_busy,
            "match_age_sec": match_age,
            "match": match,
            "telemetry": telemetry,
            "truth": truth,
            "vision_tx": vision_tx,
            "vision_alignment": vision_alignment,
            "visual_kf": visual_kf,
            "nav_estimate": nav_estimate,
            "search": {"radius_m": radius},
            "nav_gates": {
                "telemetry_position_gate": self.telemetry_position_gate,
                "telemetry_seed_source": self.telemetry_seed_source,
                "telemetry_max_age_sec": self.telemetry_max_age_sec,
                "max_nav_error_m": self.max_nav_error_m,
                "max_nav_jump_m": self.max_nav_jump_m,
                "max_nav_step_speed_mps": self.max_nav_step_speed_mps,
                "max_nav_step_slack_m": self.max_nav_step_slack_m,
                "nav_filter_alpha": self.nav_filter_alpha,
                "nav_filter_reset_residual_m": self.nav_filter_reset_residual_m,
                "min_nav_inliers": self.min_nav_inliers,
                "vision_align_source": self.vision_align_source,
                "vision_align_max_age_sec": self.vision_align_max_age_sec,
                "vision_stream_mode": self.vision_stream_mode,
                "kf_min_inliers": self.kf_min_inliers,
                "kf_max_jump_m": self.kf_max_jump_m,
                "kf_max_innovation_m": self.kf_max_innovation_m,
                "kf_max_fix_age_sec": self.kf_max_fix_age_sec,
                "kf_reset_on_high_inlier_jump": self.kf_reset_on_high_inlier_jump,
                "kf_reset_min_inliers": self.kf_reset_min_inliers,
                "kf_reset_residual_m": self.kf_reset_residual_m,
            },
            "map": self.geometry.to_dict(),
            "errors": errors,
        }

    def _set_nav_estimate_from_match(self, match: Dict[str, Any], now: float) -> None:
        pred = match.get("pred_ned") or {}
        if pred.get("north") is None or pred.get("east") is None:
            return

        raw_north = float(pred["north"])
        raw_east = float(pred["east"])
        north = raw_north
        east = raw_east
        filter_residual_m = 0.0
        filter_reset = False
        search_source = (match.get("search") or {}).get("source")
        local = _fresh_telemetry_item(
            self.telemetry.get("local_ned") if isinstance(self.telemetry, dict) else None,
            now,
            self.telemetry_max_age_sec,
        )
        pred_down = pred.get("down")
        down = float(pred_down) if pred_down is not None else _local_down(local, 0.0)
        vx = 0.0
        vy = 0.0
        if self.nav_estimate is not None:
            dt = max(1.0e-3, now - float(self.nav_estimate["timestamp"]))
            predicted_north = float(self.nav_estimate["north"]) + float(self.nav_estimate.get("vx") or 0.0) * dt
            predicted_east = float(self.nav_estimate["east"]) + float(self.nav_estimate.get("vy") or 0.0) * dt
            filter_residual_m = math.hypot(raw_north - predicted_north, raw_east - predicted_east)
            filter_reset = _should_reset_visual_filter(
                search_source=search_source,
                residual_m=filter_residual_m,
                reset_residual_m=self.nav_filter_reset_residual_m,
            )
            if filter_reset:
                north = raw_north
                east = raw_east
                self.reset_counter = (self.reset_counter + 1) % 256
            elif self.nav_filter_alpha < 1.0:
                north = predicted_north + self.nav_filter_alpha * (raw_north - predicted_north)
                east = predicted_east + self.nav_filter_alpha * (raw_east - predicted_east)
            if not filter_reset:
                raw_vx = (north - float(self.nav_estimate["north"])) / dt
                raw_vy = (east - float(self.nav_estimate["east"])) / dt
                raw_vx, raw_vy = _clamp_velocity(raw_vx, raw_vy, self.max_visual_speed_mps)
                prev_vx = float(self.nav_estimate.get("vx") or 0.0)
                prev_vy = float(self.nav_estimate.get("vy") or 0.0)
                vx = self.visual_velocity_alpha * raw_vx + (1.0 - self.visual_velocity_alpha) * prev_vx
                vy = self.visual_velocity_alpha * raw_vy + (1.0 - self.visual_velocity_alpha) * prev_vy

        if self.nav_estimate is not None and not filter_reset:
            jump = math.hypot(north - float(self.nav_estimate["north"]), east - float(self.nav_estimate["east"]))
            if jump > self.reset_jump_m:
                self.reset_counter = (self.reset_counter + 1) % 256

        self.nav_estimate = {
            "source": "vision",
            "north": north,
            "east": east,
            "raw_north": raw_north,
            "raw_east": raw_east,
            "down": down,
            "timestamp": now,
            "vx": vx,
            "vy": vy,
            "vz": 0.0,
            "filter_alpha": self.nav_filter_alpha,
            "filter_residual_m": filter_residual_m,
            "filter_reset": filter_reset,
            "frame_id": match.get("frame_id"),
            "inliers": match.get("inliers"),
            "scale_name": match.get("scale_name"),
            "tile_img": match.get("tile_img"),
        }

    def _update_visual_kf_from_match_locked(
        self,
        match: Dict[str, Any],
        estimate_time: float,
        receive_time: float,
    ) -> Dict[str, Any]:
        pred = match.get("pred_ned") or {}
        raw_north_value = pred.get("north")
        raw_east_value = pred.get("east")
        inliers = int(match.get("inliers") or 0)
        capture_time = _estimate_time_from_match(match, estimate_time)
        record: Dict[str, Any] = {
            "capture_time": capture_time,
            "receive_time": receive_time,
            "raw_north": raw_north_value,
            "raw_east": raw_east_value,
            "aligned_north": None,
            "aligned_east": None,
            "inliers": inliers,
            "accepted": False,
            "reject_reason": "",
            "jump_m": None,
            "innovation_m": None,
            "fix_age_s": max(0.0, receive_time - capture_time),
            "frame_id": match.get("frame_id"),
            "reset": False,
            "reset_reason": "",
        }

        if raw_north_value is None or raw_east_value is None:
            record["reject_reason"] = "missing_pred_ned"
            self.visual_kf_last_measurement = record
            return dict(record)

        raw_north = float(raw_north_value)
        raw_east = float(raw_east_value)
        alignment = self._vision_output_alignment_locked(raw_north, raw_east, self.telemetry, receive_time)
        if alignment is None:
            record["reject_reason"] = "alignment_unavailable"
            self.visual_kf_last_measurement = record
            return dict(record)

        aligned_north = raw_north + float(alignment.get("offset_north", 0.0))
        aligned_east = raw_east + float(alignment.get("offset_east", 0.0))
        record.update(
            {
                "raw_north": raw_north,
                "raw_east": raw_east,
                "aligned_north": aligned_north,
                "aligned_east": aligned_east,
                "alignment": alignment,
            }
        )

        if inliers < self.kf_min_inliers:
            record["reject_reason"] = "low_inliers"
            self.visual_kf_last_measurement = record
            return dict(record)

        if self.kf_max_fix_age_sec > 0.0 and record["fix_age_s"] > self.kf_max_fix_age_sec:
            record["reject_reason"] = "stale_fix"
            self.visual_kf_last_measurement = record
            return dict(record)

        if (
            self.visual_kf.initialized
            and self.visual_kf.timestamp is not None
            and capture_time < float(self.visual_kf.timestamp) - 1.0e-3
        ):
            record["reject_reason"] = "out_of_order_fix"
            self.visual_kf_last_measurement = record
            return dict(record)

        if self.visual_kf_last_fix is not None:
            jump_m = math.hypot(
                aligned_north - float(self.visual_kf_last_fix["aligned_north"]),
                aligned_east - float(self.visual_kf_last_fix["aligned_east"]),
            )
            record["jump_m"] = jump_m
            if self.kf_max_jump_m > 0.0 and jump_m > self.kf_max_jump_m:
                if self._should_reset_visual_kf_locked(inliers, jump_m):
                    return self._reset_visual_kf_from_measurement_locked(
                        record,
                        raw_north=raw_north,
                        raw_east=raw_east,
                        aligned_north=aligned_north,
                        aligned_east=aligned_east,
                        capture_time=capture_time,
                        receive_time=receive_time,
                        inliers=inliers,
                        frame_id=match.get("frame_id"),
                        alignment=alignment,
                        reason="jump_gate",
                    )
                record["reject_reason"] = "jump_gate"
                self.visual_kf_last_measurement = record
                return dict(record)

        innovation = self.visual_kf.innovation(aligned_north, aligned_east, capture_time)
        if innovation is not None:
            record["innovation_m"] = innovation
            if self.kf_max_innovation_m > 0.0 and innovation > self.kf_max_innovation_m:
                if self._should_reset_visual_kf_locked(inliers, innovation):
                    return self._reset_visual_kf_from_measurement_locked(
                        record,
                        raw_north=raw_north,
                        raw_east=raw_east,
                        aligned_north=aligned_north,
                        aligned_east=aligned_east,
                        capture_time=capture_time,
                        receive_time=receive_time,
                        inliers=inliers,
                        frame_id=match.get("frame_id"),
                        alignment=alignment,
                        reason="innovation_gate",
                    )
                record["reject_reason"] = "innovation_gate"
                self.visual_kf_last_measurement = record
                return dict(record)

        update = self.visual_kf.update(aligned_north, aligned_east, capture_time)
        record.update(
            {
                "accepted": True,
                "reject_reason": "",
                "innovation_m": update.get("innovation_m", record.get("innovation_m")),
                "kf_north": update.get("north"),
                "kf_east": update.get("east"),
                "kf_vn": update.get("vn"),
                "kf_ve": update.get("ve"),
                "kf_update_count": update.get("update_count"),
            }
        )
        self.visual_kf_last_fix = {
            "capture_time": capture_time,
            "receive_time": receive_time,
            "raw_north": raw_north,
            "raw_east": raw_east,
            "aligned_north": aligned_north,
            "aligned_east": aligned_east,
            "inliers": inliers,
            "frame_id": match.get("frame_id"),
            "alignment": alignment,
        }
        self.visual_kf_last_measurement = dict(record)
        return dict(record)

    def _should_reset_visual_kf_locked(self, inliers: int, residual_m: float) -> bool:
        return (
            self.kf_reset_on_high_inlier_jump
            and inliers >= self.kf_reset_min_inliers
            and self.kf_reset_residual_m > 0.0
            and residual_m >= self.kf_reset_residual_m
        )

    def _reset_visual_kf_from_measurement_locked(
        self,
        record: Dict[str, Any],
        *,
        raw_north: float,
        raw_east: float,
        aligned_north: float,
        aligned_east: float,
        capture_time: float,
        receive_time: float,
        inliers: int,
        frame_id: Any,
        alignment: Dict[str, Any],
        reason: str,
    ) -> Dict[str, Any]:
        update = self.visual_kf.reset(aligned_north, aligned_east, capture_time)
        self.reset_counter = (self.reset_counter + 1) % 256
        record.update(
            {
                "accepted": True,
                "reject_reason": "",
                "reset": True,
                "reset_reason": reason,
                "innovation_m": update.get("innovation_m", record.get("innovation_m")),
                "kf_north": update.get("north"),
                "kf_east": update.get("east"),
                "kf_vn": update.get("vn"),
                "kf_ve": update.get("ve"),
                "kf_update_count": update.get("update_count"),
            }
        )
        self.visual_kf_last_fix = {
            "capture_time": capture_time,
            "receive_time": receive_time,
            "raw_north": raw_north,
            "raw_east": raw_east,
            "aligned_north": aligned_north,
            "aligned_east": aligned_east,
            "inliers": inliers,
            "frame_id": frame_id,
            "alignment": alignment,
            "reset": True,
            "reset_reason": reason,
        }
        self.visual_kf_last_measurement = dict(record)
        return dict(record)

    def _nav_reject_reason(self, match: Dict[str, Any], now: float) -> Optional[Dict[str, Any]]:
        inliers = int(match.get("inliers") or 0)
        if inliers < self.min_nav_inliers:
            return {
                "reason": "low_inliers",
                "inliers": inliers,
                "limit": self.min_nav_inliers,
            }

        pred = match.get("pred_ned") or {}
        if pred.get("north") is None or pred.get("east") is None:
            return {"reason": "missing_pred_ned"}

        north = float(pred["north"])
        east = float(pred["east"])
        search_source = (match.get("search") or {}).get("source")
        local = _fresh_telemetry_item(
            self.telemetry.get("local_ned") if isinstance(self.telemetry, dict) else None,
            now,
            self.telemetry_max_age_sec,
        )
        seed = self._telemetry_position(self.telemetry, now)

        if self.telemetry_position_gate and self.max_nav_error_m > 0 and seed:
            local_error = math.hypot(north - seed[0], east - seed[1])
            if local_error > self.max_nav_error_m:
                return {
                    "reason": "telemetry_position_gate",
                    "error_m": local_error,
                    "limit_m": self.max_nav_error_m,
                    "telemetry_north": seed[0],
                    "telemetry_east": seed[1],
                    "telemetry_source": seed[2],
                    "telemetry_age_sec": seed[3],
                }

        use_visual_prediction_gates = not search_source or search_source == "vision_predicted"

        if use_visual_prediction_gates and self.max_nav_jump_m > 0 and self.nav_estimate is not None:
            velocity = _velocity_from_local(local)
            age = max(0.0, now - float(self.nav_estimate["timestamp"]))
            if self.search_nav_max_age_sec <= 0.0 or age <= self.search_nav_max_age_sec:
                expected_north = float(self.nav_estimate["north"]) + velocity[0] * age
                expected_east = float(self.nav_estimate["east"]) + velocity[1] * age
                jump = math.hypot(north - expected_north, east - expected_east)
                if jump > self.max_nav_jump_m:
                    return {
                        "reason": "prediction_jump_gate",
                        "jump_m": jump,
                        "limit_m": self.max_nav_jump_m,
                        "expected_north": expected_north,
                        "expected_east": expected_east,
                        "nav_age_sec": age,
                    }

        if use_visual_prediction_gates and self.max_nav_step_speed_mps > 0 and self.nav_estimate is not None:
            age = max(1.0e-3, now - float(self.nav_estimate["timestamp"]))
            if self.search_nav_max_age_sec <= 0.0 or age <= self.search_nav_max_age_sec:
                expected_north = float(self.nav_estimate["north"]) + float(self.nav_estimate.get("vx") or 0.0) * age
                expected_east = float(self.nav_estimate["east"]) + float(self.nav_estimate.get("vy") or 0.0) * age
                step = math.hypot(north - expected_north, east - expected_east)
                limit = self.max_nav_step_slack_m + self.max_nav_step_speed_mps * age
                if step > limit:
                    return {
                        "reason": "visual_step_speed_gate",
                        "step_m": step,
                        "limit_m": limit,
                        "speed_limit_mps": self.max_nav_step_speed_mps,
                        "slack_m": self.max_nav_step_slack_m,
                        "expected_north": expected_north,
                        "expected_east": expected_east,
                        "nav_age_sec": age,
                    }

        return None

    def _vision_output_alignment(
        self,
        north: float,
        east: float,
        telemetry: Dict[str, Any],
        now: float,
    ) -> Optional[Dict[str, Any]]:
        with self.lock:
            return self._vision_output_alignment_locked(north, east, telemetry, now)

    def _vision_output_alignment_locked(
        self,
        north: float,
        east: float,
        telemetry: Dict[str, Any],
        now: float,
    ) -> Optional[Dict[str, Any]]:
        if self.vision_align_source == "none":
            return {
                "source": "none",
                "offset_north": 0.0,
                "offset_east": 0.0,
                "created_at": None,
                "age_sec": None,
            }

        existing = _json_copy(self.vision_alignment)
        if existing:
            existing["age_sec"] = max(0.0, now - float(existing["created_at"]))
            return existing

        seed = self._telemetry_position(telemetry, now, max_age_sec=self.vision_align_max_age_sec)
        if not seed:
            return None

        alignment = {
            "source": self.vision_align_source,
            "seed_source": seed[2],
            "seed_age_sec": seed[3],
            "seed_north": seed[0],
            "seed_east": seed[1],
            "map_north_at_lock": north,
            "map_east_at_lock": east,
            "offset_north": seed[0] - north,
            "offset_east": seed[1] - east,
            "created_at": now,
            "age_sec": 0.0,
        }
        if self.vision_alignment is None:
            self.vision_alignment = alignment
            return dict(alignment)
        existing = _json_copy(self.vision_alignment)
        existing["age_sec"] = max(0.0, now - float(existing["created_at"]))
        return existing

    def _pose_velocity(
        self,
        nav: Optional[Dict[str, Any]],
        local: Optional[Dict[str, Any]],
        source: str,
    ) -> Tuple[float, float, float]:
        if source == "local":
            vx, vy = _velocity_from_local(local)
            return vx, vy, _vertical_velocity_from_local(local)
        if source == "visual" and nav is not None:
            return (
                float(nav.get("vx") or 0.0),
                float(nav.get("vy") or 0.0),
                float(nav.get("vz") or 0.0),
            )
        return 0.0, 0.0, 0.0

    def _telemetry_position(
        self,
        telemetry: Dict[str, Any],
        now: Optional[float] = None,
        max_age_sec: Optional[float] = None,
    ) -> Optional[Tuple[float, float, str, Optional[float]]]:
        if now is None:
            now = time.time()
        if max_age_sec is None:
            max_age_sec = self.telemetry_max_age_sec
        local_seed = None
        local = _fresh_telemetry_item(
            telemetry.get("local_ned") if isinstance(telemetry, dict) else None,
            now,
            max_age_sec,
        )
        if local and local.get("north") is not None and local.get("east") is not None:
            local_seed = (
                float(local["north"]),
                float(local["east"]),
                "local_ned_seed",
                _telemetry_age(local, now),
            )

        global_seed = None
        global_position = _fresh_telemetry_item(
            telemetry.get("global_position") if isinstance(telemetry, dict) else None,
            now,
            max_age_sec,
        )
        if global_position and global_position.get("lat") is not None and global_position.get("lon") is not None:
            lat = float(global_position["lat"])
            lon = float(global_position["lon"])
            if abs(lat) > 1.0e-7 or abs(lon) > 1.0e-7:
                north, east = self.geometry.latlon_to_ned(lat, lon)
                global_seed = (
                    float(north),
                    float(east),
                    "global_position_seed",
                    _telemetry_age(global_position, now),
                )

        if self.telemetry_seed_source == "global":
            return global_seed
        if self.telemetry_seed_source == "local":
            return local_seed
        return local_seed or global_seed


def _json_copy(value: Any) -> Any:
    return json.loads(json.dumps(value))


def _estimate_time_from_match(match: Dict[str, Any], default: float) -> float:
    raw_time = match.get("frame_capture_time")
    if raw_time is None:
        return default
    try:
        frame_time = float(raw_time)
    except (TypeError, ValueError):
        return default
    if frame_time <= 0 or frame_time > default:
        return default
    return frame_time


def _telemetry_age(item: Optional[Dict[str, Any]], now: float) -> Optional[float]:
    if not isinstance(item, dict):
        return None
    timestamp = item.get("timestamp")
    if timestamp is None:
        return None
    try:
        age = now - float(timestamp)
    except (TypeError, ValueError):
        return None
    return max(0.0, age)


def _fresh_telemetry_item(
    item: Optional[Dict[str, Any]],
    now: float,
    max_age_sec: float,
) -> Optional[Dict[str, Any]]:
    if not isinstance(item, dict):
        return None
    if max_age_sec <= 0.0:
        return item
    age = _telemetry_age(item, now)
    if age is None or age > max_age_sec:
        return None
    return item


def _annotate_telemetry_age(telemetry: Dict[str, Any], now: float, max_age_sec: float) -> None:
    if not isinstance(telemetry, dict):
        return
    for key in ("local_ned", "global_position", "attitude", "vfr_hud"):
        item = telemetry.get(key)
        if not isinstance(item, dict):
            continue
        age = _telemetry_age(item, now)
        item["age_sec"] = age
        item["fresh"] = max_age_sec <= 0.0 or (age is not None and age <= max_age_sec)


def _annotate_truth_age(truth: Dict[str, Any], now: float, max_age_sec: float) -> None:
    if not isinstance(truth, dict):
        return
    age = _telemetry_age(truth, now)
    truth["age_sec"] = age
    truth["fresh"] = max_age_sec <= 0.0 or (age is not None and age <= max_age_sec)


def _velocity_from_local(local: Optional[Dict[str, Any]]) -> Tuple[float, float]:
    if not local:
        return 0.0, 0.0
    return float(local.get("vx") or 0.0), float(local.get("vy") or 0.0)


def _local_down(local: Optional[Dict[str, Any]], default: float) -> float:
    if local and local.get("down") is not None:
        return float(local["down"])
    return default


def _vertical_velocity_from_local(local: Optional[Dict[str, Any]]) -> float:
    if local and local.get("vz") is not None:
        return float(local["vz"])
    return 0.0


def _speed(velocity: Tuple[float, float]) -> float:
    return math.hypot(velocity[0], velocity[1])


def _clamp_velocity(vx: float, vy: float, limit: float) -> Tuple[float, float]:
    speed = math.hypot(vx, vy)
    if speed <= limit:
        return vx, vy
    scale = limit / speed
    return vx * scale, vy * scale


def _should_reset_visual_filter(
    search_source: Optional[str],
    residual_m: float,
    reset_residual_m: float,
) -> bool:
    if search_source and search_source != "vision_predicted":
        return True
    return reset_residual_m > 0.0 and residual_m > reset_residual_m
