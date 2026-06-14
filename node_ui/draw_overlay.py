from PIL import Image, ImageDraw, ImageFont
import os

artifact_dir = r"C:\Users\jayan.kandathil\.gemini\antigravity\brain\3f25e609-8632-4782-a015-e67b74e58fb1"
in_path = os.path.join(artifact_dir, "media__1781301508094.png")
out_path = os.path.join(artifact_dir, "robot_overlay.png")

img = Image.open(in_path).convert("RGBA")
w, h = img.size

# We want to draw an overlay box over the right 2/3 of the image, where the dominoes are
# And leave the left 1/3 for the robot
x0, y0 = int(w * 0.35), int(h * 0.15)
x1, y1 = int(w * 0.95), int(h * 0.85)

overlay = Image.new("RGBA", img.size, (0, 0, 0, 0))
draw = ImageDraw.Draw(overlay)

# Semi-transparent red fill
draw.rectangle([x0, y0, x1, y1], fill=(255, 0, 0, 30))

# Thick red border
line_w = int(max(w, h) * 0.008)
draw.rectangle([x0, y0, x1, y1], outline=(255, 0, 0, 255), width=line_w)

# Text labels
# We'll just draw basic text, scaled up by drawing it multiple times or using an inner loop if font isn't huge
try:
    font = ImageFont.truetype("arial.ttf", size=int(min(w, h) * 0.08))
except:
    font = ImageFont.load_default()

def draw_text(x, y, text):
    # outline
    for dx in [-2, 0, 2]:
        for dy in [-2, 0, 2]:
            draw.text((x+dx, y+dy), text, fill="white", font=font)
    draw.text((x, y), text, fill="red", font=font)

draw_text(x0 - int(w*0.08), y0 - int(h*0.05), "TL")
draw_text(x1 + int(w*0.02), y0 - int(h*0.05), "TR")
draw_text(x0 - int(w*0.08), y1 - int(h*0.02), "BL")
draw_text(x1 + int(w*0.02), y1 - int(h*0.02), "BR")

out = Image.alpha_composite(img, overlay)
out.save(out_path)
print("Saved overlay to", out_path)
