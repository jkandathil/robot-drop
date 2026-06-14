import json
import os

with open("calibration.json", "r") as f:
    CALIBRATION = json.load(f)

def forward_kinematics(t1, t2):
    return 100, 100

def _uv_from_xy(area, x, y):
    return 0.5, 0.5

def test_load():
    try:
        name = "logo_print.json"
        with open(name, "r") as f:
            content = json.load(f)
            
        loaded_locs = content.get("locations", [])
        wells = []
        for loc in loaded_locs:
            if loc.get("type") == "WELL" and "pos" in loc:
                pose = loc["pos"]
                x, y = forward_kinematics(pose[0], pose[1])
                try:
                    u, v = _uv_from_xy('substrate', x, y)
                except Exception:
                    u, v = 0.5, 0.5
                wells.append({
                    "name": loc.get("name", "Well"),
                    "pose": pose,
                    "u": u,
                    "v": v
                })
        if wells:
            CALIBRATION['locations']['wells'] = wells
            
        print("Success")
    except Exception as e:
        print("Exception:", type(e).__name__, str(e))

test_load()
