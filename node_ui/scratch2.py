import json

with open("correction_map.json", "r") as f:
    CORRECTION_MAP = json.load(f)

def get_correction_offset(x, y):
    math_pts = CORRECTION_MAP['math']
    phys_pts = CORRECTION_MAP['phys']
    
    bl_m = math_pts['bl']; br_m = math_pts['br']; tr_m = math_pts['tr']; tl_m = math_pts['tl']
    bl_p = phys_pts['bl']; br_p = phys_pts['br']; tr_p = phys_pts['tr']; tl_p = phys_pts['tl']
    
    w = br_m['x'] - bl_m['x']
    h = tl_m['y'] - bl_m['y']
    
    u = (x - bl_m['x']) / w
    v = (y - bl_m['y']) / h
    
    phys_x = (1-u)*(1-v)*bl_p['x'] + u*(1-v)*br_p['x'] + u*v*tr_p['x'] + (1-u)*v*tl_p['x']
    phys_y = (1-u)*(1-v)*bl_p['y'] + u*(1-v)*br_p['y'] + u*v*tr_p['y'] + (1-u)*v*tl_p['y']
    
    return (phys_x - x, phys_y - y)

# If user jogged to TR (Robot's Right, X=Forward, Y=Left)
px0, py0 = 250, -50

# What is the offset at px0, py0?
cx, cy = get_correction_offset(px0, py0)
print(f"Offset at {px0}, {py0} is cx={cx:.2f}, cy={cy:.2f}")

# The code in teach_rect applies the reverse offset to find true physical coordinates
px_true = px0 - cx
py_true = py0 - cy

# Then derives TL: TL_true = TR_true + (0, 100)
tlx_true = px_true
tly_true = py_true + 100

# Then it runs inverse_kinematics, which adds the offset back for THAT coordinate!
cx_tl, cy_tl = get_correction_offset(tlx_true, tly_true)
tlx_raw = tlx_true + cx_tl
tly_raw = tly_true + cy_tl

print(f"TL raw target is {tlx_raw:.2f}, {tly_raw:.2f}")
import math
dist = math.hypot(tlx_raw, tly_raw)
print(f"TL distance is {dist:.1f} (limit 303.8)")
