from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, Tuple


ORIG_MAP_W = 25600
ORIG_MAP_H = 13568
PREVIEW_MAP = "EXPORT_GOOGLE_SAT_WM_preview_2000.jpg"


@dataclass(frozen=True)
class MapGeometry:
    """Conversions between map pixels, Gazebo world XY, and ArduPilot local NED."""

    map_w: int = ORIG_MAP_W
    map_h: int = ORIG_MAP_H
    mesh_x_min: float = -956.4617
    mesh_x_max: float = 954.4643
    mesh_y_min: float = -509.7802
    mesh_y_max: float = 503.0106
    collada_scale: float = 0.75
    model_pose_x: float = 48.6
    model_pose_y: float = -100.05
    ned_mode: str = "enu"
    home_lat: float = 40.74318017142464
    home_lon: float = 30.331305257172342
    home_alt: float = 212.5

    @property
    def mesh_w(self) -> float:
        return self.mesh_x_max - self.mesh_x_min

    @property
    def mesh_h(self) -> float:
        return self.mesh_y_max - self.mesh_y_min

    @property
    def meters_per_px(self) -> float:
        return (self.mesh_w * self.collada_scale) / float(self.map_w)

    def pixel_to_world(self, px: float, py: float) -> Tuple[float, float]:
        mx = self.mesh_x_min + (px / self.map_w) * self.mesh_w
        my = self.mesh_y_max - (py / self.map_h) * self.mesh_h
        wx = self.model_pose_x + self.collada_scale * mx
        wy = self.model_pose_y + self.collada_scale * my
        return wx, wy

    def world_to_pixel(self, wx: float, wy: float) -> Tuple[float, float]:
        mx = (wx - self.model_pose_x) / self.collada_scale
        my = (wy - self.model_pose_y) / self.collada_scale
        px = ((mx - self.mesh_x_min) / self.mesh_w) * self.map_w
        py = ((self.mesh_y_max - my) / self.mesh_h) * self.map_h
        return px, py

    def world_to_ned(self, wx: float, wy: float) -> Tuple[float, float]:
        if self.ned_mode == "enu":
            return wy, wx
        if self.ned_mode == "xy":
            return wx, wy
        if self.ned_mode == "neg_enu":
            return -wy, -wx
        if self.ned_mode == "neg_xy":
            return -wx, -wy
        raise ValueError(f"Unknown ned_mode: {self.ned_mode}")

    def ned_to_world(self, north: float, east: float) -> Tuple[float, float]:
        if self.ned_mode == "enu":
            return east, north
        if self.ned_mode == "xy":
            return north, east
        if self.ned_mode == "neg_enu":
            return -east, -north
        if self.ned_mode == "neg_xy":
            return -north, -east
        raise ValueError(f"Unknown ned_mode: {self.ned_mode}")

    def pixel_to_ned(self, px: float, py: float) -> Tuple[float, float]:
        return self.world_to_ned(*self.pixel_to_world(px, py))

    def ned_to_pixel(self, north: float, east: float) -> Tuple[float, float]:
        return self.world_to_pixel(*self.ned_to_world(north, east))

    def ned_to_latlon(self, north: float, east: float) -> Tuple[float, float]:
        earth_radius = 6378137.0
        lat0 = math.radians(self.home_lat)
        lat = self.home_lat + math.degrees(north / earth_radius)
        lon = self.home_lon + math.degrees(east / (earth_radius * math.cos(lat0)))
        return lat, lon

    def latlon_to_ned(self, lat: float, lon: float) -> Tuple[float, float]:
        earth_radius = 6378137.0
        lat0 = math.radians(self.home_lat)
        north = math.radians(lat - self.home_lat) * earth_radius
        east = math.radians(lon - self.home_lon) * earth_radius * math.cos(lat0)
        return north, east

    def to_dict(self) -> Dict[str, Any]:
        return {
            "map_w": self.map_w,
            "map_h": self.map_h,
            "meters_per_px": self.meters_per_px,
            "ned_mode": self.ned_mode,
            "home_lat": self.home_lat,
            "home_lon": self.home_lon,
            "home_alt": self.home_alt,
        }


@dataclass(frozen=True)
class MasterMapGeometry:
    """Direct georeferenced master-map pixel/world/NED conversion."""

    map_w: int = 12288
    map_h: int = 12288
    x_min: float = -750.0
    x_max: float = 750.0
    y_min: float = -750.0
    y_max: float = 750.0
    ned_mode: str = "enu"
    home_lat: float = 40.74318017142464
    home_lon: float = 30.331305257172342
    home_alt: float = 212.5

    @property
    def meters_per_px(self) -> float:
        return (self.x_max - self.x_min) / float(self.map_w)

    def pixel_to_world(self, px: float, py: float) -> Tuple[float, float]:
        wx = self.x_min + px * self.meters_per_px
        wy = self.y_max - py * self.meters_per_px
        return wx, wy

    def world_to_pixel(self, wx: float, wy: float) -> Tuple[float, float]:
        px = (wx - self.x_min) / self.meters_per_px
        py = (self.y_max - wy) / self.meters_per_px
        return px, py

    def world_to_ned(self, wx: float, wy: float) -> Tuple[float, float]:
        if self.ned_mode == "enu":
            return wy, wx
        if self.ned_mode == "xy":
            return wx, wy
        if self.ned_mode == "neg_enu":
            return -wy, -wx
        if self.ned_mode == "neg_xy":
            return -wx, -wy
        raise ValueError(f"Unknown ned_mode: {self.ned_mode}")

    def ned_to_world(self, north: float, east: float) -> Tuple[float, float]:
        if self.ned_mode == "enu":
            return east, north
        if self.ned_mode == "xy":
            return north, east
        if self.ned_mode == "neg_enu":
            return -east, -north
        if self.ned_mode == "neg_xy":
            return -north, -east
        raise ValueError(f"Unknown ned_mode: {self.ned_mode}")

    def pixel_to_ned(self, px: float, py: float) -> Tuple[float, float]:
        return self.world_to_ned(*self.pixel_to_world(px, py))

    def ned_to_pixel(self, north: float, east: float) -> Tuple[float, float]:
        return self.world_to_pixel(*self.ned_to_world(north, east))

    def ned_to_latlon(self, north: float, east: float) -> Tuple[float, float]:
        earth_radius = 6378137.0
        lat0 = math.radians(self.home_lat)
        lat = self.home_lat + math.degrees(north / earth_radius)
        lon = self.home_lon + math.degrees(east / (earth_radius * math.cos(lat0)))
        return lat, lon

    def latlon_to_ned(self, lat: float, lon: float) -> Tuple[float, float]:
        earth_radius = 6378137.0
        lat0 = math.radians(self.home_lat)
        north = math.radians(lat - self.home_lat) * earth_radius
        east = math.radians(lon - self.home_lon) * earth_radius * math.cos(lat0)
        return north, east

    def to_dict(self) -> Dict[str, Any]:
        return {
            "map_w": self.map_w,
            "map_h": self.map_h,
            "meters_per_px": self.meters_per_px,
            "x_min": self.x_min,
            "x_max": self.x_max,
            "y_min": self.y_min,
            "y_max": self.y_max,
            "ned_mode": self.ned_mode,
            "home_lat": self.home_lat,
            "home_lon": self.home_lon,
            "home_alt": self.home_alt,
        }
