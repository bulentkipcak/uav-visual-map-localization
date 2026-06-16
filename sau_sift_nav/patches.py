from __future__ import annotations

import json
import math
from collections import OrderedDict
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from .config import ScaleConfig
from .geometry import MasterMapGeometry


@dataclass(frozen=True)
class PatchRecord:
    npz_path: str
    tile_img: str
    bbox: Tuple[int, int, int, int]
    row: int
    col: int
    world_bbox: Tuple[float, float, float, float]

    @property
    def center(self) -> Tuple[float, float]:
        x0, y0, x1, y1 = self.bbox
        return (x0 + x1) * 0.5, (y0 + y1) * 0.5


class MasterPatchDb:
    """SIFT database over the 12288px georeferenced master-map patches."""

    def __init__(
        self,
        patch_dir: Path | str,
        geometry: MasterMapGeometry,
        cache_tiles: int = 256,
    ) -> None:
        self.patch_dir = Path(patch_dir)
        self.geometry = geometry
        self.scale = ScaleConfig(name="sift_master", map_dir=str(self.patch_dir), to_1x=1.0)
        self.cache_tiles = max(0, cache_tiles)
        self.cache: OrderedDict[str, Tuple[np.ndarray, np.ndarray]] = OrderedDict()

        metadata_path = self.patch_dir / "patch_metadata.json"
        if not metadata_path.exists():
            raise RuntimeError(f"Missing master patch metadata: {metadata_path}")

        with metadata_path.open("r", encoding="utf-8") as f:
            raw = json.load(f)
        patches = raw.get("patches", raw) if isinstance(raw, dict) else raw
        if not isinstance(patches, list):
            raise RuntimeError(f"Invalid master patch metadata format: {metadata_path}")

        records: List[PatchRecord] = []
        for item in patches:
            if not isinstance(item, dict):
                continue
            name = str(item["name"])
            npz_path = Path(str(item.get("npz_path", Path("sift") / f"{Path(name).stem}.npz")))
            if not npz_path.is_absolute():
                npz_path = self.patch_dir / npz_path
            if not npz_path.exists():
                sift_dir = self.patch_dir / "sift"
                fallback = sift_dir / f"{Path(name).stem}.npz"
                if fallback.exists():
                    npz_path = fallback
                else:
                    continue
            bbox = (
                int(item["pixel_x_min"]),
                int(item["pixel_y_min"]),
                int(item["pixel_x_max"]),
                int(item["pixel_y_max"]),
            )
            records.append(
                PatchRecord(
                    npz_path=str(npz_path),
                    tile_img=name,
                    bbox=bbox,
                    row=int(item["row"]),
                    col=int(item["col"]),
                    world_bbox=(
                        float(item["world_x_min"]),
                        float(item["world_y_min"]),
                        float(item["world_x_max"]),
                        float(item["world_y_max"]),
                    ),
                )
            )

        if not records:
            raise RuntimeError(f"No usable master SIFT patches found under {self.patch_dir}")
        self.records = sorted(records, key=lambda record: (record.row, record.col))
        self.records_by_rc: Dict[Tuple[int, int], PatchRecord] = {
            (record.row, record.col): record for record in self.records
        }

    def load(self, record: PatchRecord) -> Tuple[np.ndarray, np.ndarray]:
        cached = self.cache.get(record.npz_path)
        if cached is not None:
            self.cache.move_to_end(record.npz_path)
            return cached

        with np.load(record.npz_path) as data:
            descriptors = data["descriptors"].astype(np.float32, copy=False)
            keypoints = data["keypoints"].astype(np.float32, copy=False)
        loaded = (descriptors, keypoints)

        if self.cache_tiles > 0:
            self.cache[record.npz_path] = loaded
            self.cache.move_to_end(record.npz_path)
            while len(self.cache) > self.cache_tiles:
                self.cache.popitem(last=False)

        return loaded

    def candidates(
        self,
        center_px_1x: Optional[Tuple[float, float]],
        radius_m: Optional[float],
        max_tiles: Optional[int],
    ) -> List[PatchRecord]:
        if center_px_1x is None or radius_m is None or radius_m <= 0:
            return list(self.records)

        cx, cy = center_px_1x
        radius_px = radius_m / self.geometry.meters_per_px

        ranked = [
            (_distance_to_bbox(cx, cy, record.bbox), record)
            for record in self.records
        ]
        in_radius = [(dist, record) for dist, record in ranked if dist <= radius_px]
        if not in_radius:
            in_radius = ranked

        in_radius.sort(key=lambda item: (item[0], _center_distance(cx, cy, item[1])))
        windowed = self._window_candidates(cx, cy, max_tiles)
        if windowed:
            allowed = {record.tile_img for record in windowed}
            ordered = [record for _, record in in_radius if record.tile_img in allowed]
            if ordered:
                return ordered[:max_tiles] if max_tiles else ordered

        records = [record for _, record in in_radius]
        return records[:max_tiles] if max_tiles else records

    def _window_candidates(
        self,
        cx: float,
        cy: float,
        max_tiles: Optional[int],
    ) -> List[PatchRecord]:
        if not max_tiles or max_tiles < 9:
            return []
        side = int(math.isqrt(max_tiles))
        if side * side != max_tiles or side % 2 == 0:
            return []

        nearest = min(self.records, key=lambda record: _center_distance(cx, cy, record))
        half = side // 2
        records: List[PatchRecord] = []
        for row in range(nearest.row - half, nearest.row + half + 1):
            for col in range(nearest.col - half, nearest.col + half + 1):
                record = self.records_by_rc.get((row, col))
                if record is not None:
                    records.append(record)
        records.sort(key=lambda record: (_distance_to_bbox(cx, cy, record.bbox), record.row, record.col))
        return records


def _center_distance(x: float, y: float, record: PatchRecord) -> float:
    cx, cy = record.center
    return math.hypot(cx - x, cy - y)


def _distance_to_bbox(x: float, y: float, bbox: Tuple[int, int, int, int]) -> float:
    x0, y0, x1, y1 = bbox
    dx = max(x0 - x, 0.0, x - x1)
    dy = max(y0 - y, 0.0, y - y1)
    return math.hypot(dx, dy)
