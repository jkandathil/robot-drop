import json
import requests
import time
import os
import math

JSON_FILE = 'logo_uv.json'
SERVER_URL = 'http://127.0.0.1:5000'

def main():
    if not os.path.exists(JSON_FILE):
        print(f"Error: {JSON_FILE} not found!")
        return
        
    with open(JSON_FILE, 'r') as f:
        uv_points = json.load(f)
        
    print("Clearing old points...")
    try:
        requests.post(f"{SERVER_URL}/loc/clear_targets", json={"area": "substrate"}, timeout=2)
    except Exception as e:
        print(f"Connection error (is the server running?): {e}")
        return

    print("Fetching calibration state to fix aspect ratio...")
    try:
        state_res = requests.get(f"{SERVER_URL}/cal/state", timeout=2)
        cal_data = state_res.json()
        
        corners_xy = cal_data['corners_xy']['substrate']
        tl = corners_xy.get('TL')
        tr = corners_xy.get('TR')
        bl = corners_xy.get('BL')
        
        if tl and tr and bl:
            w_mm = math.hypot(tr['x'] - tl['x'], tr['y'] - tl['y'])
            h_mm = math.hypot(bl['x'] - tl['x'], bl['y'] - tl['y'])
        else:
            w_mm, h_mm = 1.0, 1.0
            
        if w_mm == 0 or h_mm == 0:
            w_mm, h_mm = 1.0, 1.0
            
    except Exception as e:
        print(f"Failed to fetch calibration: {e}")
        w_mm, h_mm = 1.0, 1.0

    print(f"Physical area: {w_mm:.1f} x {h_mm:.1f} mm. Adjusting UVs...")
    min_dim = min(w_mm, h_mm)
    scale_u = min_dim / w_mm
    scale_v = min_dim / h_mm

    print(f"Loaded {len(uv_points)} coordinates from {JSON_FILE}.")
    print("Sending points to the dispensing area zone (substrate)...")
    
    success_count = 0
    for i, pt in enumerate(uv_points):
        u_sq, v_sq = pt['u'], pt['v']
        
        # Apply aspect ratio correction
        u = 0.5 + (u_sq - 0.5) * scale_u
        v = 0.5 + (v_sq - 0.5) * scale_v
        
        name = f"LogoPoint-{i+1}"
        
        # POST to the server's add_uv endpoint
        try:
            res = requests.post(f"{SERVER_URL}/loc/add_uv", json={
                "area": "substrate",
                "name": name,
                "u": u,
                "v": v
            }, timeout=2)
            
            if res.status_code == 200:
                success_count += 1
            else:
                print(f"Failed to add {name}: {res.text}")
        except Exception as e:
            print(f"Connection error: {e}")
            break
            
        # Brief pause to not overwhelm the server
        time.sleep(0.05)
        
    print(f"\nDone! Successfully added {success_count} out of {len(uv_points)} points to the dispensing area zone.")
    print("You can now view these points in the app's UI under the Substrate calibration section!")

if __name__ == '__main__':
    main()
