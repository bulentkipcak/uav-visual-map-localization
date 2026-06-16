from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List


DEFAULT_SCALES: Dict[str, Dict[str, float | str]] = {
    "1x": {"map_dir": "map_1x", "to_1x": 1.0},
    "1/2x": {"map_dir": "map_1_2x", "to_1x": 2.0},
    "1/4x": {"map_dir": "map_1_4x", "to_1x": 4.0},
}


@dataclass(frozen=True)
class ScaleConfig:
    name: str
    map_dir: str
    to_1x: float


def parse_scales(raw: str) -> List[ScaleConfig]:
    scales: List[ScaleConfig] = []
    for name in [part.strip() for part in raw.split(",") if part.strip()]:
        if name not in DEFAULT_SCALES:
            known = ", ".join(DEFAULT_SCALES)
            raise SystemExit(f"Unknown scale '{name}'. Known: {known}")
        item = DEFAULT_SCALES[name]
        scales.append(
            ScaleConfig(
                name=name,
                map_dir=str(item["map_dir"]),
                to_1x=float(item["to_1x"]),
            )
        )
    if not scales:
        raise SystemExit("At least one scale is required")
    return scales
