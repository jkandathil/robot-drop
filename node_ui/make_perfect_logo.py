import json
import math
import requests

def main():
    with open(r'c:\Users\jayan.kandathil\Documents\andrew-robot_test\sample_print_folder\logo_uv.json', 'r') as f:
        uv_points = json.load(f)

    with open(r'c:\Users\jayan.kandathil\Documents\andrew-robot_test\node_ui\calibration.json', 'r') as f:
        cal = json.load(f)
        
    corners = cal['substrate']['corners']
    tl = corners['TL']
    tr = corners['TR']
    bl = corners['BL']
    br = corners['BR']
    
    # We need to map UV to XY manually
    # Or just use the server's endpoint!
    # Let's just use the server's bilinear_pose logic by fetching it, or implementing it here.
    
    locations = []
    sequence = []
    
    # Just POST to /loc/add_uv for all points, then read them from calibration.json, then write to logo_print.json
    print("Clearing...")
    requests.post("http://127.0.0.1:5000/loc/clear_targets", json={"area": "substrate"})
    
    # fix aspect ratio
    w = math.hypot(tr[0] - tl[0], tr[1] - tl[1])
    h = math.hypot(bl[0] - tl[0], bl[1] - tl[1])
    min_dim = min(w, h)
    su = min_dim / w
    sv = min_dim / h
    
    for i, pt in enumerate(uv_points):
        u = 0.5 + (pt['u'] - 0.5) * su
        v = 0.5 + (pt['v'] - 0.5) * sv
        
        # Add to server
        name = f"LogoPoint-{i+1}"
        requests.post("http://127.0.0.1:5000/loc/add_uv", json={
            "area": "substrate",
            "name": name,
            "u": u,
            "v": v
        })
        
    # Now server has them. Let's read calibration.json
    with open(r'c:\Users\jayan.kandathil\Documents\andrew-robot_test\node_ui\calibration.json', 'r') as f:
        cal = json.load(f)
        
    wells = cal['locations']['wells']
    
    for i, w in enumerate(wells):
        locations.append({
            "name": w["name"],
            "type": "WELL",
            "pos": w["pose"]
        })
        # The UI's sequence designer references steps by location index,
        # not by raw pose (see renderSequence/runSequence in index.html).
        sequence.append({
            "action": "dispense",
            "locIdx": i,
            "locName": w["name"]
        })
        
    out = {
        "locations": locations,
        "sequence": sequence
    }
    
    with open(r'c:\Users\jayan.kandathil\Documents\andrew-robot_test\node_ui\logo_print.json', 'w') as f:
        json.dump(out, f, indent=2)
        
    print("DONE! PERFECT FILE CREATED!")

if __name__ == "__main__":
    main()
