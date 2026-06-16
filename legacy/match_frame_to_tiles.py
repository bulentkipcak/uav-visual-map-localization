import os
import cv2
import numpy as np
import csv
from glob import glob

FRAME_PATH = "deneme.png"          # <-- bunu değiştir
TILES_SIFT_DIR = "tiles_sift"
TILES_META_CSV = "tiles_meta.csv"

# Eşleştirme parametreleri
RATIO = 0.75
MIN_INLIERS = 30
RANSAC_REPROJ_THRESH = 4.0

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

def main():
    # Frame oku
    frame = cv2.imread(FRAME_PATH, cv2.IMREAD_GRAYSCALE)
    if frame is None:
        raise RuntimeError(f"Frame okunamadı: {FRAME_PATH}")

    # Frame SIFT
    sift = cv2.SIFT_create()
    frame_eq = cv2.equalizeHist(frame)
    kp_f, des_f = sift.detectAndCompute(frame_eq, None)

    if des_f is None or len(kp_f) < 10:
        raise RuntimeError("Frame'de yeterli SIFT keypoint yok (çok blur/az doku olabilir).")

    # Matcher (SIFT için L2)
    bf = cv2.BFMatcher(cv2.NORM_L2)

    # Meta yükle
    meta = load_tile_meta(TILES_META_CSV)

    best = None  # (inliers, tile_npz, tile_img, H, good_matches)

    npz_files = sorted(glob(os.path.join(TILES_SIFT_DIR, "*.npz")))
    print("Tiles in DB:", len(npz_files))

    for i, npz_path in enumerate(npz_files, 1):
        data = np.load(npz_path)
        des_t = data["descriptors"]
        kp_t = data["keypoints"]  # (N,4) -> x,y,size,angle

        if des_t.shape[0] < 20:
            continue

        # kNN match
        matches = bf.knnMatch(des_f, des_t, k=2)

        good = []
        for m, n in matches:
            if m.distance < RATIO * n.distance:
                good.append(m)

        if len(good) < MIN_INLIERS:
            continue

        # Homography için nokta çiftleri
        pts_frame = np.float32([kp_f[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        pts_tile  = np.float32([kp_t[m.trainIdx][:2] for m in good]).reshape(-1, 1, 2)

        H, mask = cv2.findHomography(pts_frame, pts_tile, cv2.RANSAC, RANSAC_REPROJ_THRESH)
        if H is None or mask is None:
            continue

        inliers = int(mask.sum())
        if inliers >= MIN_INLIERS:
            tile_img_name = os.path.splitext(os.path.basename(npz_path))[0] + ".jpg"
            # (bizim npz adı tile_r... ile, tile adı da aynı kökten geliyor)
            # meta.csv'de tile_name jpg olarak var; eşleştirelim:
            # meta anahtarı tile_r..._x..._y....jpg
            # bu yüzden tile_img_name tam uyuyor olmalı.
            tile_bbox = meta.get(tile_img_name, None)

            cand = (inliers, npz_path, tile_img_name, H, len(good), tile_bbox)

            if best is None or cand[0] > best[0]:
                best = cand

        if i % 100 == 0:
            print(f"[{i}/{len(npz_files)}] scanned... current best inliers:",
                  (best[0] if best else 0))

    if best is None:
        print("Eşleşme bulunamadı. (RATIO/MIN_INLIERS/RANSAC_THRESH ayarı gerekebilir)")
        return

    inliers, npz_path, tile_img, H, good_count, tile_bbox = best
    print("\nBEST MATCH")
    print("Tile:", tile_img)
    print("NPZ :", npz_path)
    print("Good matches:", good_count)
    print("Inliers:", inliers)
    print("Tile bbox (map pixel coords):", tile_bbox)
    print("Homography H:\n", H)
    h, w = frame.shape[:2]
    cx, cy = w/2.0, h/2.0

    p = np.array([[[cx, cy]]], dtype=np.float32)          # (1,1,2)
    pt = cv2.perspectiveTransform(p, H)[0,0]              # tile içi koordinat

    tile_xmin, tile_ymin, _, _ = tile_bbox
    global_x = tile_xmin + pt[0]
    global_y = tile_ymin + pt[1]

    print("Frame center in TILE (px):", pt)
    print("Estimated GLOBAL map position (px):", (global_x, global_y))


if __name__ == "__main__":
    main()
