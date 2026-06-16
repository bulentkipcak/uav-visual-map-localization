import os
import csv
import cv2
import numpy as np
from glob import glob

TILES_DIR = "tiles"
OUT_DIR   = "tiles_sift"
INDEX_CSV = "tiles_sift_index.csv"

# SIFT parametreleri (başlangıç için iyi)
NFEATURES = 0          # 0 = sınırsız
CONTRAST  = 0.04
EDGE      = 10
SIGMA     = 1.6

def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    sift = cv2.SIFT_create(
        nfeatures=NFEATURES,
        contrastThreshold=CONTRAST,
        edgeThreshold=EDGE,
        sigma=SIGMA
    )

    tile_paths = sorted(glob(os.path.join(TILES_DIR, "*.jpg")))
    if not tile_paths:
        raise RuntimeError(f"{TILES_DIR} içinde jpg yok.")

    rows = []
    total = 0

    for i, p in enumerate(tile_paths, 1):
        img = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
        if img is None:
            print("Skip (read fail):", p)
            continue

        # Hafif normalize (isteğe bağlı ama genelde iyi)
        img = cv2.equalizeHist(img)

        kps, des = sift.detectAndCompute(img, None)

        if des is None or len(kps) == 0:
            kp_count = 0
            des = np.empty((0, 128), dtype=np.float32)
            kps_arr = np.empty((0, 4), dtype=np.float32)  # x,y,size,angle
        else:
            kp_count = len(kps)
            kps_arr = np.array([[kp.pt[0], kp.pt[1], kp.size, kp.angle] for kp in kps], dtype=np.float32)

        tile_name = os.path.basename(p)
        out_name  = os.path.splitext(tile_name)[0] + ".npz"
        out_path  = os.path.join(OUT_DIR, out_name)

        np.savez_compressed(out_path, descriptors=des.astype(np.float32), keypoints=kps_arr)

        rows.append([tile_name, out_name, kp_count])
        total += 1

        if i % 50 == 0:
            print(f"[{i}/{len(tile_paths)}] processed...")

    with open(INDEX_CSV, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["tile_image", "tile_npz", "keypoint_count"])
        w.writerows(rows)

    print(f"Done. Processed tiles: {total}")
    print(f"Saved: {OUT_DIR}/ and {INDEX_CSV}")

if __name__ == "__main__":
    main()
