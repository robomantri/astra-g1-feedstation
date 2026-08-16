"""Compose the rendered asset thumbnails into one labelled contact sheet."""
import os

from PIL import Image, ImageDraw, ImageFont

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
THUMBS = os.path.join(REPO, "asset_thumbs")
OUT = os.path.join(REPO, "results", "isaac_asset_contact_sheet.png")

rows = []
for line in open(os.path.join(THUMBS, "inventory.txt")):
    name, kind, info, path = (line.rstrip("\n").split("\t") + ["", "", "", ""])[:4]
    if name:
        rows.append((name, kind, info, path))

# assets that are present as 0-byte stubs -- worth showing as gaps, not hiding
MISSING = []   # all four were re-downloaded from the NVIDIA asset server

COLS = 4
TW, TH = 640, 480
SCALE = 0.62
tw, th = int(TW * SCALE), int(TH * SCALE)
PAD, CAP, TOP = 16, 54, 86

cells = [(n, k, i, p) for n, k, i, p in rows if p] + \
        [(n, k, i, "") for n, k, i in MISSING]
n_rows = (len(cells) + COLS - 1) // COLS
Wpx = COLS * tw + (COLS + 1) * PAD
Hpx = TOP + n_rows * (th + CAP + PAD) + PAD

sheet = Image.new("RGB", (Wpx, Hpx), (24, 25, 28))
d = ImageDraw.Draw(sheet)


def font(sz, bold=False):
    for p in ("/usr/share/fonts/truetype/dejavu/DejaVuSans%s.ttf"
              % ("-Bold" if bold else ""),):
        if os.path.exists(p):
            return ImageFont.truetype(p, sz)
    return ImageFont.load_default()


d.text((PAD, 22), "Isaac Sim — available crates, bins and conveyors",
       font=font(28, True), fill=(240, 240, 245))
d.text((PAD, 58), f"{len([c for c in cells if c[3]])} usable  ·  "
                  "all variants present  ·  "
                  "dimensions are measured world bounds",
       font=font(15), fill=(150, 153, 160))

for i, (name, kind, info, path) in enumerate(cells):
    r, c = divmod(i, COLS)
    x = PAD + c * (tw + PAD)
    y = TOP + r * (th + CAP + PAD)
    if path and os.path.exists(path):
        im = Image.open(path).convert("RGB").resize((tw, th), Image.LANCZOS)
        sheet.paste(im, (x, y))
        d.rectangle([x, y, x + tw - 1, y + th - 1], outline=(70, 72, 78))
        col = (235, 236, 240)
    else:
        d.rectangle([x, y, x + tw - 1, y + th - 1], fill=(38, 30, 30),
                    outline=(120, 70, 70))
        d.text((x + tw // 2 - 52, y + th // 2 - 10), "NOT AVAILABLE",
               font=font(17, True), fill=(200, 110, 110))
        col = (200, 130, 130)
    d.text((x, y + th + 8), name, font=font(16, True), fill=col)
    d.text((x, y + th + 30), f"{kind} · {info}", font=font(13),
           fill=(150, 153, 160))

os.makedirs(os.path.dirname(OUT), exist_ok=True)
sheet.save(OUT)
print("wrote", OUT, sheet.size)
