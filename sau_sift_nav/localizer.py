from __future__ import annotations

import time
from typing import Any, Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from .config import ScaleConfig
from .geometry import MapGeometry
from .tiles import TileDb, TileRecord


class SiftLocalizer:
    def __init__(
        self,
        scales: Sequence[ScaleConfig],
        geometry: MapGeometry,
        ratio: float = 0.75,
        min_inliers: int = 30,
        ransac_reproj_thresh: float = 4.0,
        resize_w: Optional[int] = None,
        nfeatures: int = 700,
        cache_tiles: int = 256,
        max_tiles_per_scale: int = 12,
        early_stop_inliers: int = 180,
        dbs: Optional[Sequence[Any]] = None,
    ) -> None:
        self.scales = list(scales)
        self.geometry = geometry
        self.ratio = ratio
        self.min_inliers = min_inliers
        self.ransac_reproj_thresh = ransac_reproj_thresh
        self.resize_w = resize_w
        self.max_tiles_per_scale = max_tiles_per_scale
        self.early_stop_inliers = early_stop_inliers
        self.sift = cv2.SIFT_create(nfeatures=nfeatures)
        self.bf = cv2.BFMatcher(cv2.NORM_L2)
        self.dbs = list(dbs) if dbs is not None else [
            TileDb(scale, geometry, cache_tiles=cache_tiles) for scale in self.scales
        ]

    def match_frame(
        self,
        frame_bgr: np.ndarray,
        search_hint: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        started = time.perf_counter()
        if frame_bgr.ndim == 3:
            frame_gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        else:
            frame_gray = frame_bgr.copy()

        orig_h, orig_w = frame_gray.shape[:2]
        if self.resize_w is not None and frame_gray.shape[1] > self.resize_w:
            scale = self.resize_w / float(frame_gray.shape[1])
            frame_gray = cv2.resize(
                frame_gray,
                (self.resize_w, int(frame_gray.shape[0] * scale)),
                interpolation=cv2.INTER_AREA,
            )

        used_h, used_w = frame_gray.shape[:2]
        frame_eq = cv2.equalizeHist(frame_gray)
        kp_f, des_f = self.sift.detectAndCompute(frame_eq, None)
        if des_f is None or len(kp_f) < 10:
            return {
                "status": "NO_FEATURES",
                "message": "Frame has too few SIFT keypoints",
                "orig_size": [orig_w, orig_h],
                "used_size": [used_w, used_h],
                "keypoints": 0 if kp_f is None else len(kp_f),
                "duration_sec": time.perf_counter() - started,
                "search": _search_for_result(search_hint),
            }

        per_scale: List[Dict[str, Any]] = []
        best_result: Optional[Dict[str, Any]] = None
        for db in self.dbs:
            scale_started = time.perf_counter()
            records = self._candidate_records(db, search_hint)
            result = self._match_db(db, records, kp_f, des_f, frame_gray)
            result["duration_sec"] = time.perf_counter() - scale_started
            result["tiles_considered"] = len(records)
            per_scale.append(result)
            if result["status"] == "OK":
                if best_result is None or result["inliers"] > best_result["inliers"]:
                    best_result = result

        total_time = time.perf_counter() - started
        common = {
            "orig_size": [orig_w, orig_h],
            "used_size": [used_w, used_h],
            "keypoints": len(kp_f),
            "duration_sec": total_time,
            "per_scale": per_scale,
            "search": _search_for_result(search_hint),
        }
        if best_result is None:
            return {"status": "NO_MATCH", **common}

        return {"status": "OK", **best_result, **common}

    def _candidate_records(
        self,
        db: Any,
        search_hint: Optional[Dict[str, Any]],
    ) -> List[Any]:
        center = None
        radius_m = None
        if search_hint:
            raw_center = search_hint.get("center_px_1x")
            if raw_center:
                center = (float(raw_center[0]), float(raw_center[1]))
            if search_hint.get("radius_m") is not None:
                radius_m = float(search_hint["radius_m"])

        return db.candidates(center, radius_m, self.max_tiles_per_scale)

    def _match_db(
        self,
        db: Any,
        records: Sequence[Any],
        kp_f: Sequence[cv2.KeyPoint],
        des_f: np.ndarray,
        frame_gray: np.ndarray,
    ) -> Dict[str, Any]:
        best: Optional[Tuple[int, TileRecord, np.ndarray, int]] = None
        scanned = 0

        for record in records:
            scanned += 1
            des_t, kp_t = db.load(record)
            if des_t.shape[0] < 20:
                continue

            raw_matches = self.bf.knnMatch(des_f, des_t, k=2)
            good = []
            for pair in raw_matches:
                if len(pair) < 2:
                    continue
                m, n = pair
                if m.distance < self.ratio * n.distance:
                    good.append(m)

            if len(good) < self.min_inliers:
                continue

            pts_frame = np.float32([kp_f[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
            pts_tile = np.float32([kp_t[m.trainIdx][:2] for m in good]).reshape(-1, 1, 2)

            H, mask = cv2.findHomography(
                pts_frame,
                pts_tile,
                cv2.RANSAC,
                self.ransac_reproj_thresh,
            )
            if H is None or mask is None:
                continue

            inliers = int(mask.sum())
            if inliers >= self.min_inliers:
                candidate = (inliers, record, H, len(good))
                if best is None or candidate[0] > best[0]:
                    best = candidate
                    if inliers >= self.early_stop_inliers:
                        break

        if best is None:
            return {
                "status": "NO_MATCH",
                "scale_name": db.scale.name,
                "map_dir": db.scale.map_dir,
                "tiles_scanned": scanned,
            }

        inliers, record, homography, good_count = best
        result = self._build_result(db.scale, record, homography, good_count, inliers, frame_gray)
        result["tiles_scanned"] = scanned
        return result

    def _build_result(
        self,
        scale: ScaleConfig,
        record: Any,
        homography: np.ndarray,
        good_count: int,
        inliers: int,
        frame_gray: np.ndarray,
    ) -> Dict[str, Any]:
        h, w = frame_gray.shape[:2]
        center = np.array([[[w / 2.0, h / 2.0]]], dtype=np.float32)
        center_tile = cv2.perspectiveTransform(center, homography)[0, 0]

        tx0, ty0, tx1, ty1 = record.bbox
        global_x = float(tx0 + center_tile[0])
        global_y = float(ty0 + center_tile[1])
        global_x_1x = global_x * scale.to_1x
        global_y_1x = global_y * scale.to_1x
        pred_north, pred_east = self.geometry.pixel_to_ned(global_x_1x, global_y_1x)
        pred_lat, pred_lon = self.geometry.ned_to_latlon(pred_north, pred_east)

        corners = np.float32([[[0, 0], [w, 0], [w, h], [0, h]]])
        quad_tile = cv2.perspectiveTransform(corners, homography)[0]
        quad_1x = [
            [float((tx0 + point[0]) * scale.to_1x), float((ty0 + point[1]) * scale.to_1x)]
            for point in quad_tile
        ]
        quad_metrics = _quad_metrics(quad_1x, self.geometry.meters_per_px)

        tile_bbox_1x = [
            float(tx0 * scale.to_1x),
            float(ty0 * scale.to_1x),
            float(tx1 * scale.to_1x),
            float(ty1 * scale.to_1x),
        ]

        return {
            "status": "OK",
            "scale_name": scale.name,
            "map_dir": scale.map_dir,
            "tile_img": record.tile_img,
            "npz_path": record.npz_path,
            "good_count": int(good_count),
            "inliers": int(inliers),
            "tile_bbox": [int(tx0), int(ty0), int(tx1), int(ty1)],
            "tile_bbox_1x": tile_bbox_1x,
            "frame_center_tile": [float(center_tile[0]), float(center_tile[1])],
            "global_px": [global_x, global_y],
            "global_px_1x": [global_x_1x, global_y_1x],
            "pred_ned": {
                "north": float(pred_north),
                "east": float(pred_east),
                "down": None,
            },
            "pred_latlon": {"lat": float(pred_lat), "lon": float(pred_lon)},
            "frame_quad_1x": quad_1x,
            "frame_quad_metrics": quad_metrics,
            "H": [float(x) for x in homography.reshape(-1)],
        }


def _search_for_result(search_hint: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    if not search_hint:
        return {"source": "global", "radius_m": None, "center_px_1x": None}
    return {
        "source": search_hint.get("source", "unknown"),
        "radius_m": search_hint.get("radius_m"),
        "center_px_1x": search_hint.get("center_px_1x"),
        "center_ned": search_hint.get("center_ned"),
    }


def _quad_metrics(points: Sequence[Sequence[float]], meters_per_px: float) -> Dict[str, float]:
    if len(points) != 4:
        return {}

    def dist(a: Sequence[float], b: Sequence[float]) -> float:
        return float(np.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1])))

    top = dist(points[0], points[1])
    right = dist(points[1], points[2])
    bottom = dist(points[2], points[3])
    left = dist(points[3], points[0])
    area_px2 = 0.0
    for idx, point in enumerate(points):
        nxt = points[(idx + 1) % len(points)]
        area_px2 += float(point[0]) * float(nxt[1]) - float(nxt[0]) * float(point[1])
    area_px2 = abs(area_px2) * 0.5
    return {
        "top_m": top * meters_per_px,
        "right_m": right * meters_per_px,
        "bottom_m": bottom * meters_per_px,
        "left_m": left * meters_per_px,
        "width_m": ((top + bottom) * 0.5) * meters_per_px,
        "height_m": ((left + right) * 0.5) * meters_per_px,
        "area_m2": area_px2 * meters_per_px * meters_per_px,
    }
