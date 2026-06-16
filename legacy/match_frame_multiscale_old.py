import os
import csv
import cv2
import numpy as np
from glob import glob

FRAME_PATH = "data/1-50m.png"   
MAP_DIRS = [
    ("1x",   "map_1x"),
    ("1/2x", "map_1_2x"),
    ("1/4x", "map_1_4x"),
]

RATIO = 0.75
MIN_INLIERS = 30
RANSAC_REPROJ_THRESH = 4.0

RESIZE_W = None

# =======================

def load_tile_meta(meta_csv):
    meta = {}
    with open(meta_csv, "r", encoding="utf-8") as f:
        r = csv.DictReader(f)
        for row in r:
            meta[row["tile_name"]] = (
                int(float(row["x_min"])), int(float(row["y_min"])),
                int(float(row["x_max"])), int(float(row["y_max"]))
            )
    return meta

def match_in_map(frame_gray, kp_f, des_f, map_dir):
    tiles_sift_dir = os.path.join(map_dir, "tiles_sift")
    tiles_meta_csv = os.path.join(map_dir, "tiles_meta.csv")

    meta = load_tile_meta(tiles_meta_csv)
    npz_files = sorted(glob(os.path.join(tiles_sift_dir, "*.npz")))
    if not npz_files:
        raise RuntimeError(f"{tiles_sift_dir} boş görünüyor.")

    bf = cv2.BFMatcher(cv2.NORM_L2)

    best = None  # (inliers, tile_img, npz_path, H, good_count, tile_bbox)

    for npz_path in npz_files:
        data = np.load(npz_path)
        des_t = data["descriptors"]
        kp_t  = data["keypoints"]  # (N,4) x,y,size,angle

        if des_t.shape[0] < 20:
            continue

        matches = bf.knnMatch(des_f, des_t, k=2)

        good = []
        for m, n in matches:
            if m.distance < RATIO * n.distance:
                good.append(m)

        if len(good) < MIN_INLIERS:
            continue

        pts_frame = np.float32([kp_f[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        pts_tile  = np.float32([kp_t[m.trainIdx][:2] for m in good]).reshape(-1, 1, 2)

        H, mask = cv2.findHomography(pts_frame, pts_tile, cv2.RANSAC, RANSAC_REPROJ_THRESH)
        if H is None or mask is None:
            continue

        inliers = int(mask.sum())
        if inliers >= MIN_INLIERS:
            tile_img_name = os.path.splitext(os.path.basename(npz_path))[0] + ".jpg"
            tile_bbox = meta.get(tile_img_name, None)
            cand = (inliers, tile_img_name, npz_path, H, len(good), tile_bbox)

            if best is None or cand[0] > best[0]:
                best = cand

    return best

def frame_center_global(frame_gray, H, tile_bbox):
    h, w = frame_gray.shape[:2]
    cx, cy = w/2.0, h/2.0
    p = np.array([[[cx, cy]]], dtype=np.float32)
    pt = cv2.perspectiveTransform(p, H)[0, 0]  # tile içi (x,y)

    tile_xmin, tile_ymin, _, _ = tile_bbox
    gx = float(tile_xmin + pt[0])
    gy = float(tile_ymin + pt[1])
    return gx, gy, pt

def main():
    frame = cv2.imread(FRAME_PATH, cv2.IMREAD_GRAYSCALE)
    if frame is None:
        raise RuntimeError(f"Frame okunamadı: {FRAME_PATH}")

    if RESIZE_W is not None and frame.shape[1] > RESIZE_W:
        scale = RESIZE_W / frame.shape[1]
        frame = cv2.resize(frame, (RESIZE_W, int(frame.shape[0]*scale)), interpolation=cv2.INTER_AREA)

    # Frame SIFT
    sift = cv2.SIFT_create(nfeatures=1500)  # canlı için sınırlamak iyi
    frame_eq = cv2.equalizeHist(frame)
    kp_f, des_f = sift.detectAndCompute(frame_eq, None)

    if des_f is None or len(kp_f) < 10:
        raise RuntimeError("Frame'de yeterli SIFT keypoint yok (blur/az doku olabilir).")

    best_overall = None  # (scale_name, map_dir, best_tuple)
    print("Frame keypoints:", len(kp_f))

    for scale_name, map_dir in MAP_DIRS:
        best = match_in_map(frame, kp_f, des_f, map_dir)
        if best is None:
            print(f"{scale_name} ({map_dir}): match yok")
            continue

        inliers, tile_img, npz_path, H, good_count, tile_bbox = best
        print(f"{scale_name} ({map_dir}): inliers={inliers}, good={good_count}, tile={tile_img}")

        if best_overall is None or inliers > best_overall[2][0]:
            best_overall = (scale_name, map_dir, best)

    if best_overall is None:
        print("Hiçbir ölçekte eşleşme bulunamadı.")
        return

    scale_name, map_dir, best = best_overall
    inliers, tile_img, npz_path, H, good_count, tile_bbox = best

    print("\n=== BEST OVERALL ===")
    print("Scale:", scale_name, "| Map:", map_dir)
    print("Tile :", tile_img)
    print("Inliers:", inliers, "| Good:", good_count)
    print("Tile bbox:", tile_bbox)

    if tile_bbox is not None:
        gx, gy, pt = frame_center_global(frame, H, tile_bbox)
        print("Frame center in TILE (px):", (pt[0], pt[1]))
        print("Estimated GLOBAL map position (px):", (gx, gy))

if __name__ == "__main__":
    main()
