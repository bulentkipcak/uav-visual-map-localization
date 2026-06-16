from __future__ import annotations

import argparse
import csv
import json
import pickle
import time
from pathlib import Path
from typing import Any, Dict, List

import cv2
import numpy as np


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Build georeferenced SIFT patches from SIFT_master.png")
    parser.add_argument("--master-dir", default="QGIS/SAU CAMPUS/output/SIFT")
    parser.add_argument("--master-image", default="SIFT_master.png")
    parser.add_argument("--out-dir", default="patches")
    parser.add_argument("--expected-size", type=int, default=12288)
    parser.add_argument("--patch-size", type=int, default=2048)
    parser.add_argument("--step", type=int, default=1024)
    parser.add_argument("--x-min", type=float, default=-750.0)
    parser.add_argument("--x-max", type=float, default=750.0)
    parser.add_argument("--y-min", type=float, default=-750.0)
    parser.add_argument("--y-max", type=float, default=750.0)
    parser.add_argument("--nfeatures", type=int, default=1500)
    parser.add_argument("--contrast", type=float, default=0.04)
    parser.add_argument("--edge", type=float, default=10.0)
    parser.add_argument("--sigma", type=float, default=1.6)
    parser.add_argument("--compress-npz", action="store_true")
    parser.add_argument("--no-pickle", action="store_true")
    parser.add_argument("--no-equalize", action="store_true")
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    started = time.perf_counter()
    master_dir = Path(args.master_dir)
    master_image = Path(args.master_image)
    if not master_image.is_absolute():
        master_image = master_dir / master_image
    out_dir = Path(args.out_dir)
    if not out_dir.is_absolute():
        out_dir = master_dir / out_dir
    sift_dir = out_dir / "sift"

    log(f"master={master_image}")
    log(f"out_dir={out_dir}")
    image = cv2.imread(str(master_image), cv2.IMREAD_COLOR)
    if image is None:
        raise SystemExit(f"Master image could not be read: {master_image}")
    height, width = image.shape[:2]
    log(f"master_size={width}x{height}")
    if (width, height) != (args.expected_size, args.expected_size):
        raise SystemExit(
            f"Expected master image {args.expected_size}x{args.expected_size}, got {width}x{height}"
        )

    pixel_size_x = (args.x_max - args.x_min) / float(width)
    pixel_size_y = (args.y_max - args.y_min) / float(height)
    if abs(pixel_size_x - pixel_size_y) > 1e-9:
        raise SystemExit(f"Non-square pixels are not supported: {pixel_size_x} vs {pixel_size_y}")
    pixel_size = pixel_size_x
    log(f"pixel_size_m={pixel_size:.10f}")

    xs = list(range(0, width - args.patch_size + 1, args.step))
    ys = list(range(0, height - args.patch_size + 1, args.step))
    patch_count = len(xs) * len(ys)
    expected_side = ((args.expected_size - args.patch_size) // args.step) + 1
    expected_count = expected_side * expected_side
    log(
        f"grid rows={len(ys)} cols={len(xs)} count={patch_count} "
        f"expected={expected_side}x{expected_side}={expected_count}"
    )
    if patch_count != expected_count:
        raise SystemExit(f"Patch count mismatch: got {patch_count}, expected {expected_count}")

    out_dir.mkdir(parents=True, exist_ok=True)
    sift_dir.mkdir(parents=True, exist_ok=True)
    sift = cv2.SIFT_create(
        nfeatures=args.nfeatures,
        contrastThreshold=args.contrast,
        edgeThreshold=args.edge,
        sigma=args.sigma,
    )

    metadata_rows: List[Dict[str, Any]] = []
    pickle_rows: List[Dict[str, Any]] = []
    total_keypoints = 0
    save_npz = np.savez_compressed if args.compress_npz else np.savez

    for row, y0 in enumerate(ys):
        for col, x0 in enumerate(xs):
            x1 = x0 + args.patch_size
            y1 = y0 + args.patch_size
            name = f"patch_r{row:02d}_c{col:02d}.png"
            patch_path = out_dir / name
            patch = image[y0:y1, x0:x1]
            if patch.shape[0] != args.patch_size or patch.shape[1] != args.patch_size:
                raise SystemExit(f"Unexpected patch shape for {name}: {patch.shape}")
            if not cv2.imwrite(str(patch_path), patch):
                raise SystemExit(f"Patch image could not be written: {patch_path}")

            gray = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
            if not args.no_equalize:
                gray = cv2.equalizeHist(gray)
            keypoints, descriptors = sift.detectAndCompute(gray, None)
            if descriptors is None or not keypoints:
                descriptors = np.empty((0, 128), dtype=np.float32)
                keypoint_array = np.empty((0, 4), dtype=np.float32)
            else:
                descriptors = descriptors.astype(np.float32, copy=False)
                keypoint_array = np.array(
                    [[kp.pt[0], kp.pt[1], kp.size, kp.angle] for kp in keypoints],
                    dtype=np.float32,
                )

            world_x_min = args.x_min + x0 * pixel_size
            world_x_max = args.x_min + x1 * pixel_size
            world_y_max = args.y_max - y0 * pixel_size
            world_y_min = args.y_max - y1 * pixel_size
            npz_rel = Path("sift") / f"{patch_path.stem}.npz"
            npz_path = out_dir / npz_rel
            world_bbox = np.array([world_x_min, world_y_min, world_x_max, world_y_max], dtype=np.float64)
            pixel_bbox = np.array([x0, y0, x1, y1], dtype=np.int32)
            save_npz(
                npz_path,
                descriptors=descriptors,
                keypoints=keypoint_array,
                pixel_bbox=pixel_bbox,
                world_bbox=world_bbox,
                row=np.array([row], dtype=np.int32),
                col=np.array([col], dtype=np.int32),
                pixel_size_m=np.array([pixel_size], dtype=np.float64),
            )

            metadata = {
                "name": name,
                "row": row,
                "col": col,
                "pixel_x_min": x0,
                "pixel_y_min": y0,
                "pixel_x_max": x1,
                "pixel_y_max": y1,
                "world_x_min": world_x_min,
                "world_x_max": world_x_max,
                "world_y_min": world_y_min,
                "world_y_max": world_y_max,
                "center_world_x": (world_x_min + world_x_max) * 0.5,
                "center_world_y": (world_y_min + world_y_max) * 0.5,
                "pixel_size_m": pixel_size,
                "npz_path": str(npz_rel),
                "keypoint_count": int(keypoint_array.shape[0]),
            }
            metadata_rows.append(metadata)
            if not args.no_pickle:
                pickle_rows.append(
                    {
                        "metadata": metadata,
                        "keypoints": keypoint_array,
                        "descriptors": descriptors,
                    }
                )
            total_keypoints += int(keypoint_array.shape[0])
            log(f"{name}: keypoints={keypoint_array.shape[0]}")

    map_meta = {
        "master_image": str(master_image),
        "map_width_px": width,
        "map_height_px": height,
        "x_min": args.x_min,
        "x_max": args.x_max,
        "y_min": args.y_min,
        "y_max": args.y_max,
        "pixel_size_m": pixel_size,
        "patch_size_px": args.patch_size,
        "step_px": args.step,
        "patch_count": patch_count,
        "sift_nfeatures": args.nfeatures,
    }
    json_path = out_dir / "patch_metadata.json"
    csv_path = out_dir / "patch_metadata.csv"
    manifest_path = out_dir / "sift_database_manifest.json"
    with json_path.open("w", encoding="utf-8") as f:
        json.dump({"map": map_meta, "patches": metadata_rows}, f, indent=2)
        f.write("\n")
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(metadata_rows[0].keys()))
        writer.writeheader()
        writer.writerows(metadata_rows)
    with manifest_path.open("w", encoding="utf-8") as f:
        json.dump({"map": map_meta, "patches": metadata_rows}, f, indent=2)
        f.write("\n")

    if not args.no_pickle:
        pickle_path = out_dir / "sift_database.pkl"
        with pickle_path.open("wb") as f:
            pickle.dump({"map": map_meta, "patches": pickle_rows}, f, protocol=pickle.HIGHEST_PROTOCOL)
        log(f"pickle={pickle_path}")

    elapsed = time.perf_counter() - started
    log(f"metadata_json={json_path}")
    log(f"metadata_csv={csv_path}")
    log(f"manifest={manifest_path}")
    log(f"done patches={patch_count} total_keypoints={total_keypoints} elapsed={elapsed:.1f}s")
    return 0


def log(message: str) -> None:
    print(f"[build_sift_master_patches] {message}", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
