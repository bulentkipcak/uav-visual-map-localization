#!/usr/bin/env python3
"""Create a clean side-by-side overlap/grid figure for the report.

The script is independent from the localization pipeline. It reads the existing
master-map patch metadata and draws two full-image grids:

* left: patch boundaries without overlap
* right: patch boundaries after applying the configured overlap step
"""

from __future__ import annotations

import json
from pathlib import Path

from PIL import Image, ImageDraw


Image.MAX_IMAGE_PIXELS = None

ROOT = Path(__file__).resolve().parents[1]
METADATA_PATH = ROOT / "QGIS/SAU CAMPUS/output/SIFT/patches/patch_metadata.json"
OUTPUT_PATH = ROOT / "figures/sekil_2_17_overlap_gosterimi.png"
OVERLAY_OUTPUT_PATH = ROOT / "figures/sekil_2_17_overlap_overlay_transparent.png"

CANVAS_W = 3400
CANVAS_H = 1900
PANEL_SIZE = 1500
LEFT_ORIGIN = (120, 230)
RIGHT_ORIGIN = (1780, 230)
BG = (248, 250, 252)
TEXT = (20, 28, 38)
MUTED = (88, 101, 119)
NO_OVERLAP_LINE = (255, 255, 255, 230)
NO_OVERLAP_SHADOW = (15, 23, 42, 105)
OVERLAP_LINE = (22, 163, 74, 245)
OVERLAP_LINE_SHADOW = (15, 23, 42, 80)


def main() -> int:
    metadata = load_metadata(METADATA_PATH)
    map_meta = metadata["map"]
    patches = metadata["patches"]
    master_path = resolve_master_path(map_meta)

    map_w = int(map_meta["map_width_px"])
    map_h = int(map_meta["map_height_px"])
    patch_size = int(map_meta["patch_size_px"])
    step = int(map_meta["step_px"])
    overlap = patch_size - step
    rows = sorted({int(p["row"]) for p in patches})
    cols = sorted({int(p["col"]) for p in patches})

    if len(rows) != 11 or len(cols) != 11:
        raise SystemExit(f"Expected 11x11 patch grid, got {len(rows)}x{len(cols)}")
    if overlap <= 0:
        raise SystemExit("Patch metadata does not describe overlapping patches.")

    master = Image.open(master_path).convert("RGB")
    if master.size != (map_w, map_h):
        raise SystemExit(f"Master size mismatch: image={master.size}, metadata={(map_w, map_h)}")

    canvas = Image.new("RGB", (CANVAS_W, CANVAS_H), BG)
    draw = ImageDraw.Draw(canvas, "RGBA")
    draw_labels(draw)

    left_panel = draw_panel(
        master=master,
        map_w=map_w,
        map_h=map_h,
        spacing_px=patch_size,
        line=NO_OVERLAP_LINE,
        shadow=NO_OVERLAP_SHADOW,
        line_width=7,
        shadow_width=10,
    )
    right_panel = draw_panel(
        master=master,
        map_w=map_w,
        map_h=map_h,
        spacing_px=step,
        line=OVERLAP_LINE,
        shadow=OVERLAP_LINE_SHADOW,
        line_width=4,
        shadow_width=7,
    )

    canvas.paste(left_panel, LEFT_ORIGIN)
    canvas.paste(right_panel, RIGHT_ORIGIN)
    draw_panel_frame(draw, LEFT_ORIGIN)
    draw_panel_frame(draw, RIGHT_ORIGIN)

    overlay = Image.new("RGBA", (CANVAS_W, CANVAS_H), (0, 0, 0, 0))
    overlay_draw = ImageDraw.Draw(overlay, "RGBA")
    draw_regular_grid_on_canvas(
        overlay_draw,
        origin=LEFT_ORIGIN,
        map_w=map_w,
        map_h=map_h,
        spacing_px=patch_size,
        line=NO_OVERLAP_LINE,
        shadow=NO_OVERLAP_SHADOW,
        line_width=7,
        shadow_width=10,
    )
    draw_regular_grid_on_canvas(
        overlay_draw,
        origin=RIGHT_ORIGIN,
        map_w=map_w,
        map_h=map_h,
        spacing_px=step,
        line=OVERLAP_LINE,
        shadow=OVERLAP_LINE_SHADOW,
        line_width=4,
        shadow_width=7,
    )

    output = canvas
    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    output.save(OUTPUT_PATH)
    overlay.save(OVERLAY_OUTPUT_PATH)

    print(f"metadata: {METADATA_PATH}")
    print(f"master:   {master_path}")
    print(f"output:   {OUTPUT_PATH}")
    print(f"overlay:  {OVERLAY_OUTPUT_PATH}")
    print(f"patches:  {len(patches)} ({len(rows)}x{len(cols)})")
    print(f"patch:    {patch_size}px")
    print(f"step:     {step}px")
    print(f"overlap:  {overlap}px ({overlap / patch_size:.0%})")
    return 0


def load_metadata(path: Path) -> dict:
    if not path.exists():
        raise SystemExit(f"Missing metadata: {path}")
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict) or "map" not in data or "patches" not in data:
        raise SystemExit(f"Invalid metadata format: {path}")
    return data


def resolve_master_path(map_meta: dict) -> Path:
    raw = Path(map_meta.get("master_image", "QGIS/SAU CAMPUS/output/SIFT/SIFT_master.png"))
    path = raw if raw.is_absolute() else ROOT / raw
    if path.exists():
        return path
    fallback = ROOT / "QGIS/SAU CAMPUS/output/SIFT/SIFT_master.png"
    if fallback.exists():
        return fallback
    raise SystemExit(f"Missing master image: {path}")


def resample_lanczos() -> int:
    return getattr(getattr(Image, "Resampling", Image), "LANCZOS")


def draw_panel(
    master: Image.Image,
    map_w: int,
    map_h: int,
    spacing_px: int,
    line: tuple[int, int, int, int],
    shadow: tuple[int, int, int, int],
    line_width: int,
    shadow_width: int,
) -> Image.Image:
    panel = master.resize((PANEL_SIZE, PANEL_SIZE), resample_lanczos()).convert("RGBA")
    panel = Image.alpha_composite(panel, Image.new("RGBA", panel.size, (0, 0, 0, 32)))
    overlay = Image.new("RGBA", panel.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay, "RGBA")
    draw_regular_grid_local(
        draw,
        map_w=map_w,
        map_h=map_h,
        spacing_px=spacing_px,
        line=line,
        shadow=shadow,
        line_width=line_width,
        shadow_width=shadow_width,
    )
    return Image.alpha_composite(panel, overlay).convert("RGB")


def draw_regular_grid_local(
    draw: ImageDraw.ImageDraw,
    map_w: int,
    map_h: int,
    spacing_px: int,
    line: tuple[int, int, int, int],
    shadow: tuple[int, int, int, int],
    line_width: int,
    shadow_width: int,
) -> None:
    sx = PANEL_SIZE / map_w
    sy = PANEL_SIZE / map_h
    xs = list(range(0, map_w + 1, spacing_px))
    ys = list(range(0, map_h + 1, spacing_px))
    if xs[-1] != map_w:
        xs.append(map_w)
    if ys[-1] != map_h:
        ys.append(map_h)

    for x in xs:
        px = round(x * sx)
        draw.line((px + 2, 0, px + 2, PANEL_SIZE), fill=shadow, width=shadow_width)
        draw.line((px, 0, px, PANEL_SIZE), fill=line, width=line_width)
    for y in ys:
        py = round(y * sy)
        draw.line((0, py + 2, PANEL_SIZE, py + 2), fill=shadow, width=shadow_width)
        draw.line((0, py, PANEL_SIZE, py), fill=line, width=line_width)


def draw_regular_grid_on_canvas(
    draw: ImageDraw.ImageDraw,
    origin: tuple[int, int],
    map_w: int,
    map_h: int,
    spacing_px: int,
    line: tuple[int, int, int, int],
    shadow: tuple[int, int, int, int],
    line_width: int,
    shadow_width: int,
) -> None:
    x0, y0 = origin
    sx = PANEL_SIZE / map_w
    sy = PANEL_SIZE / map_h
    xs = list(range(0, map_w + 1, spacing_px))
    ys = list(range(0, map_h + 1, spacing_px))
    if xs[-1] != map_w:
        xs.append(map_w)
    if ys[-1] != map_h:
        ys.append(map_h)

    for x in xs:
        px = x0 + round(x * sx)
        draw.line((px + 2, y0, px + 2, y0 + PANEL_SIZE), fill=shadow, width=shadow_width)
        draw.line((px, y0, px, y0 + PANEL_SIZE), fill=line, width=line_width)
    for y in ys:
        py = y0 + round(y * sy)
        draw.line((x0, py + 2, x0 + PANEL_SIZE, py + 2), fill=shadow, width=shadow_width)
        draw.line((x0, py, x0 + PANEL_SIZE, py), fill=line, width=line_width)


def draw_labels(draw: ImageDraw.ImageDraw) -> None:
    try:
        from PIL import ImageFont

        regular_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
        bold_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
        title_font = ImageFont.truetype(bold_path, 48)
        small_font = ImageFont.truetype(regular_path, 28)
    except Exception:
        from PIL import ImageFont

        title_font = ImageFont.load_default()
        small_font = ImageFont.load_default()

    draw.text((LEFT_ORIGIN[0], 88), "Şekil A", fill=TEXT, font=title_font)
    draw.text((LEFT_ORIGIN[0], 148), "Overlap yok: patch sınırları", fill=MUTED, font=small_font)
    draw.text((RIGHT_ORIGIN[0], 88), "Şekil B", fill=TEXT, font=title_font)
    draw.text((RIGHT_ORIGIN[0], 148), "Overlap var: step sonrası grid", fill=MUTED, font=small_font)


def draw_panel_frame(draw: ImageDraw.ImageDraw, origin: tuple[int, int]) -> None:
    x, y = origin
    draw.rectangle((x - 2, y - 2, x + PANEL_SIZE + 2, y + PANEL_SIZE + 2), outline=(15, 23, 42, 230), width=4)


if __name__ == "__main__":
    raise SystemExit(main())
