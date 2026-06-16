import os
import csv
import cv2
import time
import numpy as np
from glob import glob
from datetime import datetime

FRAME_PATH = "data/5-50m.png"
MAP_DIRS = [
    ("1x",   "map_1x"),
    ("1/2x", "map_1_2x"),
    ("1/4x", "map_1_4x"),
]

RATIO = 0.75
MIN_INLIERS = 30
RANSAC_REPROJ_THRESH = 4.0

RESIZE_W = None

# ÇIKTI CSV (append)
OUT_CSV = "results.csv"

# =======================
# PREVIEW MAP (küçük kopya üstüne çizim)
BIG_MAP_PATH = "EXPORT_GOOGLE_SAT_WM_preview_2000.jpg"
OUT_MARKED_MAP = "marked_predictions_preview_2000.jpg"

# Orijinal 1x map boyutu (senin identify çıktın)
ORIG_MAP_W = 25600
ORIG_MAP_H = 13568

# Her ölçekteki global_x/global_y değerini 1x koordinata çevirmek için çarpan
SCALE_TO_1X = {"1x": 1.0, "1/2x": 2.0, "1/4x": 4.0}

# Çizimde gözükecek etiket
SCALE_LABEL = {"1x": "1", "1/2x": "1/2", "1/4x": "1/4"}

# Her scale farklı renk (BGR)
SCALE_COLOR = {
    "1x":   (255, 80,  80),   # maviye yakın
    "1/2x": (80,  220, 255),  # sarımsı
    "1/4x": (80,  255, 120),  # yeşilimsi
}
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


def match_in_map(kp_f, des_f, map_dir):
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
    cx, cy = w / 2.0, h / 2.0
    p = np.array([[[cx, cy]]], dtype=np.float32)
    pt = cv2.perspectiveTransform(p, H)[0, 0]  # tile içi (x,y)

    tile_xmin, tile_ymin, _, _ = tile_bbox
    gx = float(tile_xmin + pt[0])
    gy = float(tile_ymin + pt[1])
    return gx, gy, float(pt[0]), float(pt[1])


def ensure_csv_header(path, fieldnames):
    if not os.path.exists(path) or os.path.getsize(path) == 0:
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=fieldnames)
            w.writeheader()


def H_to_str(H):
    return " ".join([f"{x:.6g}" for x in H.reshape(-1)])


def print_row_summary(row):
    print(f"[{row['scale_name']}] {row['status']}"
          f"{' | tile=' + row['tile_img'] if row['tile_img'] else ''}"
          f"{' | inliers=' + str(row['inliers']) if row['inliers'] != '' else ''}"
          f"{' | good=' + str(row['good_count']) if row['good_count'] != '' else ''}"
          f"{' | t=' + str(row['match_time_sec']) + 's' if row.get('match_time_sec','') != '' else ''}")

    print("  ├─ map_dir        :", row["map_dir"])
    print("  ├─ frame          :", row["frame_path"])
    print("  ├─ used_size      :", (row["used_w"], row["used_h"]))
    print("  ├─ ratio/min_inl  :", (row["ratio"], row["min_inliers"]))
    print("  ├─ ransac_thresh  :", row["ransac_reproj_thresh"])
    print("  ├─ tile_bbox      :", (
        row["tile_xmin"], row["tile_ymin"], row["tile_xmax"], row["tile_ymax"]
    ))

    if row["global_x"] != "":
        print("  ├─ frame center (tile px):",
              (row["frame_center_tile_x"], row["frame_center_tile_y"]))
        print("  └─ global pos (px)       :",
              (row["global_x"], row["global_y"]))
    else:
        print("  └─ global pos            : N/A")

    print()


def _draw_x(img, x, y, color, size=12, thickness=2):
    # cv2.drawMarker daha temiz bir X çiziyor
    cv2.drawMarker(
        img, (int(x), int(y)), color,
        markerType=cv2.MARKER_TILTED_CROSS,
        markerSize=int(size),
        thickness=int(thickness),
        line_type=cv2.LINE_AA
    )


def draw_predictions_on_preview_map(rows, preview_path, out_path, total_time_sec=None):
    img = cv2.imread(preview_path, cv2.IMREAD_COLOR)
    if img is None:
        raise RuntimeError(f"Preview map okunamadı: {preview_path}")

    ph, pw = img.shape[:2]

    # 1x global koordinatı preview'a taşıma ölçeği
    sx = pw / float(ORIG_MAP_W)
    sy = ph / float(ORIG_MAP_H)

    def clamp_int(v, lo, hi):
        return int(max(lo, min(hi, v)))

    # Önce tüm OK tahminleri çiz (farklı renk X + scale yazısı)
    plotted = []
    for r in rows:
        if not str(r.get("status", "")).startswith("OK"):
            continue
        if r.get("global_x", "") == "" or r.get("global_y", "") == "":
            continue

        scale_name = r["scale_name"]
        factor_to_1x = SCALE_TO_1X.get(scale_name, 1.0)

        x1 = float(r["global_x"]) * factor_to_1x
        y1 = float(r["global_y"]) * factor_to_1x

        x = x1 * sx
        y = y1 * sy

        px = clamp_int(x, 0, pw - 1)
        py = clamp_int(y, 0, ph - 1)

        color = SCALE_COLOR.get(scale_name, (0, 255, 0))
        label = SCALE_LABEL.get(scale_name, scale_name)

        _draw_x(img, px, py, color, size=16, thickness=3)

        # Yazıyı biraz kaydır (üst üste binmeyi azaltır)
        cv2.putText(
            img, label, (px + 10, py - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA
        )

        plotted.append((scale_name, px, py))

    # Best overall'ı ayrıca vurgula (daha büyük, beyaz kontur + kırmızı X)
    best_row = None
    for r in rows:
        if r.get("is_best_overall", "0") == "1" and r.get("global_x", "") != "" and r.get("global_y", "") != "":
            best_row = r
            break

    if best_row is not None:
        scale_name = best_row["scale_name"]
        factor_to_1x = SCALE_TO_1X.get(scale_name, 1.0)

        x1 = float(best_row["global_x"]) * factor_to_1x
        y1 = float(best_row["global_y"]) * factor_to_1x

        x = x1 * sx
        y = y1 * sy

        px = clamp_int(x, 0, pw - 1)
        py = clamp_int(y, 0, ph - 1)

        # beyaz kontur
        _draw_x(img, px, py, (255, 255, 255), size=28, thickness=6)
        # üstüne kırmızı
        _draw_x(img, px, py, (0, 0, 255), size=26, thickness=4)

    # Üst bilgi: toplam süre + her scale süresi
    lines = []
    if total_time_sec is not None:
        lines.append(f"Total time: {total_time_sec:.3f} s")

    # scale sürelerini sırayla yaz
    for r in rows:
        t = r.get("match_time_sec", "")
        if t != "" and r.get("status", "").startswith("OK"):
            label = SCALE_LABEL.get(r["scale_name"], r["scale_name"])
            lines.append(f"{label}: {float(t):.3f} s")

    # Metinleri sol-üst köşeye bas (okunabilirlik için siyah kontur)
    y0 = 30
    for i, txt in enumerate(lines):
        y = y0 + i * 26
        cv2.putText(img, txt, (15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(img, txt, (15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2, cv2.LINE_AA)

    cv2.imwrite(out_path, img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    print(f"İşaretli preview map yazıldı: {out_path} (sx={sx:.6f}, sy={sy:.6f}, size={pw}x{ph})")
    print(f"Plotted predictions: {len(plotted)}")


def main():
    t_start_total = time.perf_counter()

    frame = cv2.imread(FRAME_PATH, cv2.IMREAD_GRAYSCALE)
    if frame is None:
        raise RuntimeError(f"Frame okunamadı: {FRAME_PATH}")

    orig_h, orig_w = frame.shape[:2]

    if RESIZE_W is not None and frame.shape[1] > RESIZE_W:
        scale = RESIZE_W / frame.shape[1]
        frame = cv2.resize(frame, (RESIZE_W, int(frame.shape[0] * scale)), interpolation=cv2.INTER_AREA)

    h, w = frame.shape[:2]

    sift = cv2.SIFT_create(nfeatures=500)
    frame_eq = cv2.equalizeHist(frame)
    kp_f, des_f = sift.detectAndCompute(frame_eq, None)

    if des_f is None or len(kp_f) < 10:
        raise RuntimeError("Frame'de yeterli SIFT keypoint yok (blur/az doku olabilir).")

    print("Frame:", FRAME_PATH)
    print("Original size:", (orig_w, orig_h))
    print("Used size    :", (w, h))
    print("Frame keypoints:", len(kp_f))
    print("-" * 60)

    fieldnames = [
        "timestamp",
        "frame_path",
        "orig_w", "orig_h",
        "used_w", "used_h",
        "ratio", "min_inliers", "ransac_reproj_thresh",
        "scale_name", "map_dir",
        "status",
        "tile_img", "npz_path",
        "good_count", "inliers",
        "tile_xmin", "tile_ymin", "tile_xmax", "tile_ymax",
        "frame_center_tile_x", "frame_center_tile_y",
        "global_x", "global_y",
        "H_flat",
        "is_best_overall",
        "match_time_sec",
        "total_time_sec"
    ]
    ensure_csv_header(OUT_CSV, fieldnames)

    rows = []
    best_overall = None  # (scale_name, map_dir, best_tuple)

    for scale_name, map_dir in MAP_DIRS:
        t0 = time.perf_counter()
        best = match_in_map(kp_f, des_f, map_dir)
        t1 = time.perf_counter()
        match_time = t1 - t0

        row = {
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "frame_path": FRAME_PATH,
            "orig_w": orig_w, "orig_h": orig_h,
            "used_w": w, "used_h": h,
            "ratio": RATIO,
            "min_inliers": MIN_INLIERS,
            "ransac_reproj_thresh": RANSAC_REPROJ_THRESH,
            "scale_name": scale_name,
            "map_dir": map_dir,
            "status": "NO_MATCH",
            "tile_img": "",
            "npz_path": "",
            "good_count": "",
            "inliers": "",
            "tile_xmin": "", "tile_ymin": "", "tile_xmax": "", "tile_ymax": "",
            "frame_center_tile_x": "", "frame_center_tile_y": "",
            "global_x": "", "global_y": "",
            "H_flat": "",
            "is_best_overall": "0",
            "match_time_sec": f"{match_time:.6f}",
            "total_time_sec": ""
        }

        if best is None:
            print(f"[{scale_name}] NO MATCH | t={match_time:.3f}s")
            rows.append(row)
            print_row_summary(row)
            continue

        inliers, tile_img, npz_path, H, good_count, tile_bbox = best

        row["status"] = "OK"
        row["tile_img"] = tile_img
        row["npz_path"] = npz_path
        row["good_count"] = good_count
        row["inliers"] = inliers
        row["H_flat"] = H_to_str(H)

        if tile_bbox is not None:
            txmin, tymin, txmax, tymax = tile_bbox
            row["tile_xmin"] = txmin
            row["tile_ymin"] = tymin
            row["tile_xmax"] = txmax
            row["tile_ymax"] = tymax

            gx, gy, fct_x, fct_y = frame_center_global(frame, H, tile_bbox)
            row["frame_center_tile_x"] = fct_x
            row["frame_center_tile_y"] = fct_y
            row["global_x"] = gx
            row["global_y"] = gy
        else:
            row["status"] = "OK_NO_BBOX"

        rows.append(row)
        print_row_summary(row)

        if best_overall is None or inliers > best_overall[2][0]:
            best_overall = (scale_name, map_dir, best)

    # best_overall işaretle + print
    if best_overall is not None:
        best_scale, best_map, best = best_overall
        inliers, tile_img, npz_path, H, good_count, tile_bbox = best

        for r in rows:
            if r["scale_name"] == best_scale and r["map_dir"] == best_map and r["status"].startswith("OK"):
                r["is_best_overall"] = "1"
                break

        print("\n" + "=" * 40)
        print("        BEST OVERALL MATCH")
        print("=" * 40)
        print("Scale        :", best_scale)
        print("Map dir      :", best_map)
        print("Tile         :", tile_img)
        print("Inliers      :", inliers)
        print("Good matches :", good_count)
        if tile_bbox is not None:
            gx, gy, fct_x, fct_y = frame_center_global(frame, H, tile_bbox)
            print("Tile bbox    :", tile_bbox)
            print("Frame center (tile px):", (fct_x, fct_y))
            print("Global pos (px)       :", (gx, gy))
        else:
            print("Tile bbox    : None")
                # Süreleri de bas
        best_match_time = None
        for r in rows:
            if r["scale_name"] == best_scale and r["map_dir"] == best_map:
                try:
                    best_match_time = float(r.get("match_time_sec", ""))
                except:
                    best_match_time = None
                break

        if best_match_time is not None:
            print(f"Best match time : {best_match_time:.3f}s")

        print("=" * 40 + "\n")
    else:
        print("\nBEST OVERALL: bulunamadı (hiç eşleşme yok).\n")

    total_time = time.perf_counter() - t_start_total

    # total_time_sec her satıra yaz (CSV + çıktı için)
    for r in rows:
        r["total_time_sec"] = f"{total_time:.6f}"

    print(f"TOTAL TIME: {total_time:.3f}s")
        # Süre özeti
    parts = []
    for r in rows:
        if r.get("match_time_sec", "") != "":
            lbl = SCALE_LABEL.get(r["scale_name"], r["scale_name"])
            parts.append(f"{lbl}={float(r['match_time_sec']):.3f}s")
    print("TIME SUMMARY:", " | ".join(parts))


    # Preview map üstüne hepsini çiz (X, farklı renk, scale etiketi + süreler)
    draw_predictions_on_preview_map(rows, BIG_MAP_PATH, OUT_MARKED_MAP, total_time_sec=total_time)

    # CSV'ye ekle
    with open(OUT_CSV, "a", newline="", encoding="utf-8") as f:
        wcsv = csv.DictWriter(f, fieldnames=fieldnames)
        for r in rows:
            wcsv.writerow(r)

    print(f"CSV yazıldı: {OUT_CSV}")


if __name__ == "__main__":
    main()
