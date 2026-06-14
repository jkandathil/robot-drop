from PIL import Image, ImageDraw
import os

artifact_dir = r"C:\Users\jayan.kandathil\.gemini\antigravity\brain\3f25e609-8632-4782-a015-e67b74e58fb1"
out_path = os.path.join(artifact_dir, "robot_layout.png")

img = Image.new("RGB", (400, 300), "white")
draw = ImageDraw.Draw(img)

draw.ellipse([10, 100, 60, 150], fill="#888888")
draw.text((20, 120), "Robot", fill="white")

doms = [
    ("1", 80, 20), ("3", 140, 20), ("6", 200, 20),
    ("2", 80, 80), ("5", 140, 80), ("7", 200, 80),
    ("Waste", 10, 160), ("4", 80, 140), ("9", 200, 140)
]

for txt, x, y in doms:
    draw.rectangle([x, y, x+50, y+50], fill="#EEEEEE", outline="black")
    draw.text((x+15, y+20), txt, fill="black")

draw.rectangle([70, 10, 260, 200], outline="red", width=3)

draw.text((50, 0), "TL", fill="red")
draw.text((270, 0), "TR", fill="red")
draw.text((50, 190), "BL", fill="red")
draw.text((270, 190), "BR", fill="red")

img = img.resize((800, 600), Image.NEAREST)
img.save(out_path)
print("Saved to", out_path)
