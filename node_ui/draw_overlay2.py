from PIL import Image, ImageDraw, ImageFont
import os

artifact_dir = r"C:\Users\jayan.kandathil\.gemini\antigravity\brain\3f25e609-8632-4782-a015-e67b74e58fb1"
in_path = os.path.join(artifact_dir, "media__1781301508094.png")
out_path = os.path.join(artifact_dir, "robot_overlay2.png")

img = Image.open(in_path).convert("RGBA")
w, h = img.size

overlay = Image.new("RGBA", img.size, (0, 0, 0, 0))
draw = ImageDraw.Draw(overlay)

# Bounding box of the grid in the original photo
# The grid is roughly from 35% to 95% width, and 15% to 85% height
x0, y0 = int(w * 0.35), int(h * 0.15)
x1, y1 = int(w * 0.95), int(h * 0.85)

# Semi-transparent red fill
draw.rectangle([x0, y0, x1, y1], fill=(255, 0, 0, 30))

# Thick red border
line_w = int(max(w, h) * 0.008)
draw.rectangle([x0, y0, x1, y1], outline=(255, 0, 0, 255), width=line_w)

# Fonts
try:
    font = ImageFont.truetype("arial.ttf", size=int(min(w, h) * 0.06))
except:
    font = ImageFont.load_default()

def draw_text(x, y, text):
    for dx in [-2, 0, 2]:
        for dy in [-2, 0, 2]:
            draw.text((x+dx, y+dy), text, fill="white", font=font)
    draw.text((x, y), text, fill="red", font=font)

# Applying the user's diagram orientation:
# TL = max X, max Y -> Right edge, Top edge (x1, y0)
# TR = max X, min Y -> Right edge, Bottom edge (x1, y1)
# BL = min X, max Y -> Left edge, Top edge (x0, y0)
# BR = min X, min Y -> Left edge, Bottom edge (x0, y1)

# Left edge, Top edge -> Near Domino 1
draw_text(x0 - int(w*0.08), y0 - int(h*0.05), "BL") 

# Left edge, Bottom edge -> Near Waste
draw_text(x0 - int(w*0.08), y1 - int(h*0.02), "BR") 

# Right edge, Top edge -> Near Domino 6
draw_text(x1 + int(w*0.02), y0 - int(h*0.05), "TL") 

# Right edge, Bottom edge -> Near Domino 9
draw_text(x1 + int(w*0.02), y1 - int(h*0.02), "TR") 

out = Image.alpha_composite(img, overlay)
out.save(out_path)
print("Saved to", out_path)
