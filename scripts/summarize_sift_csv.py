#!/usr/bin/env python3
"""Summarize live_sift_nav CSV logs for camera/localization tuning."""

from __future__ import annotations

import argparse
import csv
import math
import statistics
from collections import Counter
from datetime import datetime
from pathlib import Path


def parse_float(row: dict[str, str], key: str) -> float | None:
    try:
        value = row.get(key, "")
        if not value:
            return None
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def quantile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, math.ceil(fraction * len(ordered)) - 1))
    return ordered[index]


def print_stats(name: str, values: list[float | None]) -> None:
    clean = [value for value in values if value is not None]
    if not clean:
        return
    print(
        f"{name}: n={len(clean)} "
        f"mean={statistics.fmean(clean):.3f} "
        f"median={statistics.median(clean):.3f} "
        f"p95={quantile(clean, 0.95):.3f} "
        f"p99={quantile(clean, 0.99):.3f} "
        f"min={min(clean):.3f} "
        f"max={max(clean):.3f}"
    )


def parse_timestamps(rows: list[dict[str, str]]) -> list[datetime]:
    timestamps: list[datetime] = []
    for row in rows:
        value = row.get("timestamp", "")
        if not value:
            continue
        try:
            timestamps.append(datetime.fromisoformat(value))
        except ValueError:
            continue
    return timestamps


def summarize(path: Path) -> None:
    with path.open(newline="") as csv_file:
        rows = list(csv.DictReader(line for line in csv_file if not line.lstrip().startswith("#")))

    timestamps = parse_timestamps(rows)
    duration_s = None
    if len(timestamps) >= 2:
        duration_s = (max(timestamps) - min(timestamps)).total_seconds()

    accepted = [row for row in rows if row.get("nav_status") == "OK"]
    rejected = [row for row in rows if row.get("nav_status") == "REJECTED"]

    print(f"file: {path}")
    print(f"rows: {len(rows)}")
    if duration_s and duration_s > 0:
        print(f"duration_s: {duration_s:.1f}")
        print(f"row_rate_hz: {len(rows) / duration_s:.2f}")
        print(f"accepted_rate_hz: {len(accepted) / duration_s:.2f}")
    print(f"accepted: {len(accepted)}")
    print(f"rejected: {len(rejected)}")
    if rows:
        print(f"status: {Counter(row.get('status', '') for row in rows).most_common()}")
        print(f"nav_status: {Counter(row.get('nav_status', '') for row in rows).most_common()}")
        print(f"reject_reason: {Counter(row.get('reject_reason', '') for row in rows).most_common()}")
        print(f"truth_source: {Counter(row.get('truth_source', '') for row in rows).most_common()}")
        if "truth_status" in rows[0]:
            print(f"truth_status: {Counter(row.get('truth_status', '') for row in rows).most_common()}")
        print(f"search_source: {Counter(row.get('search_source', '') for row in rows).most_common(5)}")
        print(f"tile_img: {Counter(row.get('tile_img', '') for row in rows).most_common(8)}")

    for key in (
        "duration_sec",
        "calc_error_m",
        "gazebo_error_m",
        "nav_gazebo_error_m",
        "error_m",
        "nav_filter_residual_m",
        "vision_tx_north_m",
        "vision_tx_east_m",
        "vision_tx_map_north_m",
        "vision_tx_map_east_m",
        "vision_align_offset_north_m",
        "vision_align_offset_east_m",
        "inliers",
        "good_count",
        "tiles_scanned",
        "tiles_considered",
        "frame_quad_width_m",
        "frame_quad_height_m",
        "frame_quad_area_m2",
        "search_radius_m",
    ):
        print_stats(key, [parse_float(row, key) for row in rows])

    accepted_errors = [
        parse_float(row, "calc_error_m")
        for row in accepted
        if parse_float(row, "calc_error_m") is not None
    ]
    print_stats("accepted_calc_error_m", accepted_errors)
    print_stats("accepted_duration_sec", [parse_float(row, "duration_sec") for row in accepted])
    print_stats("accepted_inliers", [parse_float(row, "inliers") for row in accepted])

    if accepted_errors:
        print(f"accepted_err_over_2m: {sum(error > 2.0 for error in accepted_errors)}")
        print(f"accepted_err_over_3m: {sum(error > 3.0 for error in accepted_errors)}")
        print(f"accepted_err_over_5m: {sum(error > 5.0 for error in accepted_errors)}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path", type=Path)
    args = parser.parse_args()
    summarize(args.csv_path)


if __name__ == "__main__":
    main()
