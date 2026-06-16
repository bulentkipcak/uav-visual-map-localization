import os
import csv
from PIL import Image
Image.MAX_IMAGE_PIXELS = None  # büyük görsellere izin ver


IMG_PATH   = "EXPORT_GOOGLE_SAT_WM.jpg"
OUT_DIR    = "tiles"
META_CSV   = "tiles_meta.csv"

TILE_SIZE  = 1024
OVERLAP    = 256
STEP       = TILE_SIZE - OVERLAP

JPEG_QUALITY = 92  # 85-95 arası iyi

def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    img = Image.open(IMG_PATH).convert("RGB")
    W, H = img.size
    print("Image size:", W, H)

    # Kenarları da kapsamak için son başlangıç noktalarını garanti edelim
    xs = list(range(0, max(W - TILE_SIZE, 0) + 1, STEP))
    ys = list(range(0, max(H - TILE_SIZE, 0) + 1, STEP))
    if not xs: xs = [0]
    if not ys: ys = [0]
    if xs[-1] != W - TILE_SIZE and W > TILE_SIZE:
        xs.append(W - TILE_SIZE)
    if ys[-1] != H - TILE_SIZE and H > TILE_SIZE:
        ys.append(H - TILE_SIZE)

    rows = []
    count = 0

    for r, y in enumerate(ys):
        for c, x in enumerate(xs):
            # Kaynak kutu (harita içindeki gerçek koordinatlar)
            x0, y0 = x, y
            x1, y1 = x + TILE_SIZE, y + TILE_SIZE

            # Crop için bbox (taşarsa, Pillow crop siyah doldurmaz; biz padleyeceğiz)
            crop = img.crop((x0, y0, min(x1, W), min(y1, H)))

            # Pad (kenarda küçük çıkarsa 1024x1024'e tamamla)
            if crop.size != (TILE_SIZE, TILE_SIZE):
                tile = Image.new("RGB", (TILE_SIZE, TILE_SIZE), (0, 0, 0))
                tile.paste(crop, (0, 0))
            else:
                tile = crop

            name = f"tile_r{r:03d}_c{c:03d}_x{x0}_y{y0}.jpg"
            out_path = os.path.join(OUT_DIR, name)
            tile.save(out_path, "JPEG", quality=JPEG_QUALITY, optimize=True)

            rows.append([name, x0, y0, x1, y1])
            count += 1

    with open(META_CSV, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["tile_name", "x_min", "y_min", "x_max", "y_max"])
        w.writerows(rows)

    print(f"Done. Tiles: {count}")
    print(f"Saved: {OUT_DIR}/ and {META_CSV}")

if __name__ == "__main__":
    main()
