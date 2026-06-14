import math

L1 = 152.13
L2 = 151.67
MAX_REACH = L1 + L2

width = 100
height = 100
angle_deg = 0

th = math.radians(angle_deg)
ct, st = math.cos(th), math.sin(th)
wdx, wdy = st, -ct                 # (0, -1)
hdx, hdy = -ct, -st                # (-1, 0)

off = {'TL': (0.0, 0.0),
       'TR': (-wdx * width, -wdy * width),
       'BL': (-hdx * height, -hdy * height),
       'BR': (-wdx * width - hdx * height, -wdy * width - hdy * height)}

print("Offsets:", off)

# Let's say user jogged to TR (Robot's Right, so Y is negative).
# Say X=250, Y=-50
px0, py0 = 250, -50
corner_at = 'TR'

ox, oy = off[corner_at]
tlx, tly = px0 + ox, py0 + oy

phys = {'TL': (tlx, tly),
        'TR': (tlx + wdx * width, tly + wdy * width),
        'BL': (tlx + hdx * height, tly + hdy * height),
        'BR': (tlx + wdx * width + hdx * height, tly + wdy * width + hdy * height)}

for name, (px, py) in phys.items():
    dist = math.hypot(px, py)
    print(f"{name}: px={px}, py={py}, dist={dist:.1f} (limit {MAX_REACH:.1f})")
