#!/usr/bin/env python3
"""Make a printable A4 sheet of ARUCO_ORIGINAL markers in several physical sizes.

usage: make_markers.py [OUT.pdf] [--png-dpi N]

- Dictionary: OpenCV DICT_ARUCO_ORIGINAL (5x5 bits, 1024 ids). Same dictionary as
  crates/tb4_aruco (ARUCO_ORIGINAL), which is generated from OpenCV 4.6.0.
- "Size" is the black outer square (the marker_size convention used by the detector).
  A white quiet zone is added around each marker.
- The PDF is drawn at 300 dpi, so 100 mm on the sheet is 100 mm on paper.
  Print with "Actual size" / 100 % scale. Do not use "fit to page".
- Writes a PNG next to the PDF (same name, .png) for checking on screen.
"""
import sys
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

DPI = 300
MM_PER_IN = 25.4
PAGE_W_MM, PAGE_H_MM = 210, 297          # A4 portrait
MARGIN_MM = 8
QUIET_MM = 2                              # white border around each black square
GAP_MM = 4
LABEL_MM = 7

# Pages, each a list of rows of (id, black side in mm). The 100 mm marker matches marker_size 0.10.
PAGES = [
    [   # page 1: sizes 100 down to 15 mm
        [(0, 100), (1, 80)],
        [(2, 60), (3, 50), (4, 40)],
        [(5, 30), (6, 20), (7, 15)],
    ],
    [   # page 2: more variants, bigger and smaller
        [(8, 90), (9, 90)],
        [(10, 70), (11, 55), (12, 45)],
        [(13, 35), (14, 25), (15, 12), (16, 10)],
    ],
]
MARKERS = [m for page in PAGES for row in page for m in row]

DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)


def mm_px(mm):
    return int(round(mm / MM_PER_IN * DPI))


def marker_image(marker_id, side_mm):
    """Black square of side_mm (bordered by the marker's own black frame), white quiet zone added."""
    side_px = mm_px(side_mm)
    # 5 data bits + 1 black border bit per side = 7 modules; the border bit is part of the black square
    img = np.zeros((side_px, side_px), dtype=np.uint8)
    draw_marker = getattr(cv2.aruco, "generateImageMarker", None) or cv2.aruco.drawMarker
    draw_marker(DICT, marker_id, side_px, img, 1)
    q = mm_px(QUIET_MM)
    return cv2.copyMakeBorder(img, q, q, q, q, cv2.BORDER_CONSTANT, value=255), side_px


def build_page(rows, number, total):
    page = Image.new("L", (mm_px(PAGE_W_MM), mm_px(PAGE_H_MM)), 255)
    draw = ImageDraw.Draw(page)
    try:
        font = ImageFont.truetype("DejaVuSans.ttf", mm_px(3.2))
        small = ImageFont.truetype("DejaVuSans.ttf", mm_px(2.6))
    except OSError:
        font = small = ImageFont.load_default()

    title = (f"ARUCO_ORIGINAL test markers (page {number}/{total}). Print at 100 % (actual size), "
             "no fit-to-page.  Size = black square side. 2 mm white quiet zone included.")
    draw.text((mm_px(MARGIN_MM), mm_px(MARGIN_MM)), title, fill=0, font=small)

    y_mm = MARGIN_MM + 10
    for row in rows:
        x_mm = MARGIN_MM
        row_h = max(side for _, side in row) + 2 * QUIET_MM
        for marker_id, side_mm in row:
            img, side_px = marker_image(marker_id, side_mm)
            px = mm_px(x_mm)
            py = mm_px(y_mm)
            page.paste(Image.fromarray(img), (px, py))
            label = f"ID {marker_id}   {side_mm} mm"
            draw.text((px, py + img.shape[0] + mm_px(1)), label, fill=0, font=font)
            # the cell is at least as wide as its label, so small markers never overlap the next label
            label_w_mm = draw.textlength(label, font=font) / DPI * MM_PER_IN
            cell_w_mm = max(side_mm + 2 * QUIET_MM, label_w_mm)
            x_mm += cell_w_mm + GAP_MM
        y_mm += row_h + LABEL_MM + GAP_MM

    # 100 mm scale bar (check that the printout is actual size), reserved bottom strip
    bar_y = mm_px(PAGE_H_MM - MARGIN_MM - 4)
    bar_x0 = mm_px(MARGIN_MM)
    bar_x1 = bar_x0 + mm_px(100)
    draw.line([(bar_x0, bar_y), (bar_x1, bar_y)], fill=0, width=mm_px(0.5))
    for x in (bar_x0, bar_x1):
        draw.line([(x, bar_y - mm_px(2)), (x, bar_y + mm_px(2))], fill=0, width=mm_px(0.5))
    draw.text((bar_x1 + mm_px(3), bar_y - mm_px(2)), "100 mm scale bar: must measure 100 mm",
              fill=0, font=small)
    return page


def main(argv):
    out = Path(argv[1]) if len(argv) > 1 else Path(__file__).resolve().parent.parent / "markers" / "aruco_original_sheet.pdf"
    out.parent.mkdir(parents=True, exist_ok=True)
    pages = [build_page(rows, i + 1, len(PAGES)) for i, rows in enumerate(PAGES)]
    pages[0].save(out, "PDF", resolution=DPI, save_all=True, append_images=pages[1:])
    # PNG preview: one file per page, e.g. aruco_original_sheet_p1.png
    for i, page in enumerate(pages, start=1):
        page.save(out.with_name(f"{out.stem}_p{i}.png"), dpi=(DPI, DPI))
    print(f"wrote {out} ({len(pages)} pages)")
    for i in range(1, len(pages) + 1):
        print(f"wrote {out.with_name(f'{out.stem}_p{i}.png')}")
    for marker_id, side_mm in MARKERS:
        print(f"  id {marker_id:>2}  {side_mm:>3} mm  ({mm_px(side_mm)} px at {DPI} dpi)")


if __name__ == "__main__":
    main(sys.argv)
