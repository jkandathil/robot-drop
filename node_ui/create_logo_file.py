import json
import requests
import math
import os

SERVER_URL = 'http://127.0.0.1:5000'
UV_FILE = '../sample_print_folder/logo_uv.json'
OUT_FILE = 'logo_print.json'

def main():
    if not os.path.exists(UV_FILE):
        print(f"Cannot find {UV_FILE}")
        return

    with open(UV_FILE, 'r') as f:
        uv_points = json.load(f)
        
    try:
        state_res = requests.get(f"{SERVER_URL}/cal/state", timeout=2)
        cal = state_res.json()
    except Exception as e:
        print(f"Could not connect to server: {e}")
        return

    corners = cal['corners_xy']['substrate']
    tl, tr, bl = corners.get('TL'), corners.get('TR'), corners.get('BL')
    
    if tl and tr and bl:
        w_mm = math.hypot(tr['x'] - tl['x'], tr['y'] - tl['y'])
        h_mm = math.hypot(bl['x'] - tl['x'], bl['y'] - tl['y'])
    else:
        print("Robot substrate area is not calibrated. Please map the area or load a map first!")
        return

    print(f"Physical area: {w_mm:.1f}x{h_mm:.1f} mm")
    if w_mm == 0 or h_mm == 0:
        w_mm, h_mm = 1, 1

    scale_u = min(w_mm, h_mm) / w_mm
    scale_v = min(w_mm, h_mm) / h_mm

    locations = []
    sequence = []
    
    for i, pt in enumerate(uv_points):
        u = 0.5 + (pt['u'] - 0.5) * scale_u
        v = 0.5 + (pt['v'] - 0.5) * scale_v
        
        res = requests.post(f"{SERVER_URL}/cal/preview", json={"area": "substrate", "u": u, "v": v})
        if res.status_code == 200:
            pose = res.json()['pose']
            # Optional: Ensure it uses the deposition depth if one is configured
            dz = cal['calibration']['substrate'].get('deposition_z')
            if dz is not None:
                # preview returns the surface plane. We can optionally drop it to dz, but
                # to be safe we'll use exactly what preview returns (the surface).
                pass
                
            name = f"LogoPoint-{i+1}"
            locations.append({"name": name, "type": "WELL", "pos": pose})
            
            # Simple sequence that dispenses at each point
            sequence.append({"action": "dispense", "pos": pose})
        else:
            print(f"Skipping point {i} - out of bounds or error: {res.text}")

    with open(OUT_FILE, "w") as f:
        json.dump({"locations": locations, "sequence": sequence}, f, indent=2)
    
    print(f"Created {OUT_FILE} with {len(sequence)} actions!")

if __name__ == "__main__":
    main()
