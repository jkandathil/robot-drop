"""
One-shot asset prep for the Andrew Robot UI.

- Removes the flat light-gray rectangular background from the Noze logo by
  keying out low-saturation (gray/white) pixels, leaving the green mark on a
  transparent canvas. Anti-aliased edges are feathered so there's no hard halo.
- Tightly crops to the mark and writes a transparent PNG.
- Emits favicon assets (PNG sizes + a multi-size .ico) for the app window/tab.

Re-runnable: always reads from the original backup, never from its own output.
"""
import os
from PIL import Image

LOGO_DIR = os.path.join(os.path.dirname(__file__), '..', 'logo_noze')
SRC = os.path.join(LOGO_DIR, 'noze_logo.png')
BACKUP = os.path.join(LOGO_DIR, 'noze_logo_original.png')
OUT_LOGO = os.path.join(LOGO_DIR, 'noze_logo.png')          # overwrite in place
UI_DIR = os.path.dirname(__file__)

# Keep an untouched copy of the original the first time we run.
if not os.path.exists(BACKUP):
    Image.open(SRC).convert('RGBA').save(BACKUP)

img = Image.open(BACKUP).convert('RGBA')
px = img.load()
w, h = img.size

# A pixel is "logo" when green clearly dominates red and blue; otherwise it's the
# gray/white background. We feather alpha across a small greenness band so the
# edge stays smooth instead of jagged.
LOW, HIGH = 6, 26   # greenness thresholds (alpha ramps 0->255 between these)
for y in range(h):
    for x in range(w):
        r, g, b, a = px[x, y]
        greenness = g - max(r, b)
        if greenness <= LOW:
            alpha = 0
        elif greenness >= HIGH:
            alpha = 255
        else:
            alpha = int((greenness - LOW) / (HIGH - LOW) * 255)
        px[x, y] = (r, g, b, alpha)

# Tight crop to the visible mark, then pad to a square so it scales evenly.
bbox = img.getbbox()
if bbox:
    img = img.crop(bbox)
side = max(img.size)
square = Image.new('RGBA', (side, side), (0, 0, 0, 0))
square.paste(img, ((side - img.width) // 2, (side - img.height) // 2))
img = square

img.save(OUT_LOGO)
print('wrote transparent logo:', OUT_LOGO, img.size)

# --- Favicons -------------------------------------------------------------
ico_path = os.path.join(UI_DIR, 'favicon.ico')
png32 = os.path.join(UI_DIR, 'favicon-32.png')
png180 = os.path.join(UI_DIR, 'favicon-180.png')  # apple-touch sized, handy reuse

img.resize((32, 32), Image.LANCZOS).save(png32)
img.resize((180, 180), Image.LANCZOS).save(png180)
img.save(ico_path, sizes=[(16, 16), (32, 32), (48, 48), (64, 64)])
print('wrote favicons:', ico_path, png32, png180)
