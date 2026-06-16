from __future__ import annotations

import csv
import math
import os
from collections import OrderedDict
from dataclasses import dataclass
from glob import glob
from typing import Dict, List, Optional, Tuple

import numpy as np

from .config import ScaleConfig
from .geometry import MapGeometry


@dataclass(frozen=True)
class TileRecord:
    npz_path: str
    tile_img: str
    bbox: Tuple[int, int, int, int]

    @property
    def center(self) -> Tuple[float, float]:
        x0, y0, x1, y1 = self.bbox
        return (x0 + x1) * 0.5, (y0 + y1) * 0.5


def load_tile_meta(meta_csv: str) -> Dict[str, Tuple[int, int, int, int]]:
    meta: Dict[str, Tuple[int, int, int, int]] = {}
    with open(meta_csv, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            meta[row["tile_name"]] = (
                int(float(row["x_min"])),
                int(float(row["y_min"])),
                int(float(row["x_max"])),
                int(float(row["y_max"])),
            )
    return meta


class TileDb:
    def __init__(
        self,
        scale: ScaleConfig,
        geometry: MapGeometry,
        cache_tiles: int = 256,
    ) -> None:
        self.scale = scale
        self.geometry = geometry
        self.cache_tiles = max(0, cache_tiles)
        self.cache: OrderedDict[str, Tuple[np.ndarray, np.ndarray]] = OrderedDict()

        tiles_sift_dir = os.path.join(scale.map_dir, "tiles_sift")
        meta_csv = os.path.join(scale.map_dir, "tiles_meta.csv")
        if not os.path.isdir(tiles_sift_dir):
            raise RuntimeError(f"Missing tile SIFT dir: {tiles_sift_dir}")
        if not os.path.exists(meta_csv):
            raise RuntimeError(f"Missing tile meta csv: {meta_csv}")

        meta = load_tile_meta(meta_csv)
        records: List[TileRecord] = []
        for npz_path in sorted(glob(os.path.join(tiles_sift_dir, "*.npz"))):
            tile_img = os.path.splitext(os.path.basename(npz_path))[0] + ".jpg"
            bbox = meta.get(tile_img)
            if bbox is not None:
                records.append(TileRecord(npz_path=npz_path, tile_img=tile_img, bbox=bbox))

        if not records:
            raise RuntimeError(f"No usable SIFT tiles found under {tiles_sift_dir}")
        self.records = records

    def load(self, record: TileRecord) -> Tuple[np.ndarray, np.ndarray]:
        cached = self.cache.get(record.npz_path)
        if cached is not None:
            self.cache.move_to_end(record.npz_path)
            return cached

        data = np.load(record.npz_path)
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
    ) -> List[TileRecord]:
        if center_px_1x is None or radius_m is None or radius_m <= 0:
            return list(self.records)

        cx = center_px_1x[0] / self.scale.to_1x
        cy = center_px_1x[1] / self.scale.to_1x
        radius_px = radius_m / (self.geometry.meters_per_px * self.scale.to_1x)

        ranked: List[Tuple[float, TileRecord]] = []
        for record in self.records:
            dist = _distance_to_bbox(cx, cy, record.bbox)
            if dist <= radius_px:
                ranked.append((dist, record))

        if not ranked:
            # Keep the system alive if the radius is too small: try nearest tiles.
            ranked = [(_distance_to_bbox(cx, cy, record.bbox), record) for record in self.records]

        ranked.sort(key=lambda item: item[0])
        if max_tiles:
            ranked = ranked[:max_tiles]
        return [record for _, record in ranked]


def _distance_to_bbox(x: float, y: float, bbox: Tuple[int, int, int, int]) -> float:
    x0, y0, x1, y1 = bbox
    dx = max(x0 - x, 0.0, x - x1)
    dy = max(y0 - y, 0.0, y - y1)
    return math.hypot(dx, dy)
