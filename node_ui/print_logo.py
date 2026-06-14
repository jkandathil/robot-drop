import os
import sys
import math
import requests
from PIL import Image, ImageFilter

# Add server directory to path so we can use its kinematics functions
UI_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(UI_DIR)
import server

LOGO_PATH = os.path.join(UI_DIR, '..', 'logo_noze', 'noze_logo.png')

def get_base_pos():
    """Get the current physical position of the robot to use as the center."""
    try:
        res = requests.get('http://127.0.0.1:5000/current_pos', timeout=2)
        data = res.json()
        if data['status'] != 'success':
            raise Exception(data['message'])
        return data['pos']
    except Exception as e:
        raise Exception(f"Could not connect to robot server: {e}")

def extract_boundary_path(image_path, num_points=60, size_mm=40.0):
    """
    Extracts the boundary of the logo, subsamples it, and returns
    a list of (x_offset_mm, y_offset_mm) points centered at (0,0).
    """
    img = Image.open(image_path).convert('RGBA')
    # Use the alpha channel to find the edges
    alpha = img.split()[3]
    edges = alpha.filter(ImageFilter.FIND_EDGES)
    
    w, h = edges.size
    px = edges.load()
    
    edge_pts = []
    # Collect all pixels that are part of the edge
    for y in range(h):
        for x in range(w):
            if px[x, y] > 50:
                edge_pts.append((x, y))
                
    if not edge_pts:
        raise Exception("No edges found in the image!")
        
    # Sort the points into a continuous path using a simple nearest-neighbor algorithm
    path = []
    current = edge_pts.pop(0)
    path.append(current)
    
    while edge_pts:
        # Find the closest point to the current point
        nearest_idx = min(range(len(edge_pts)), key=lambda i: (edge_pts[i][0]-current[0])**2 + (edge_pts[i][1]-current[1])**2)
        current = edge_pts.pop(nearest_idx)
        path.append(current)
        
    # Subsample the path to the desired number of points
    if len(path) > num_points:
        step = len(path) / float(num_points)
        path = [path[int(i * step)] for i in range(num_points)]
        
    # Calculate bounding box to normalize and center the path
    min_x = min(p[0] for p in path)
    max_x = max(p[0] for p in path)
    min_y = min(p[1] for p in path)
    max_y = max(p[1] for p in path)
    
    w_px = max_x - min_x
    h_px = max_y - min_y
    
    # Scale pixels to mm
    scale = size_mm / max(w_px, h_px)
    
    cx = (max_x + min_x) / 2.0
    cy = (max_y + min_y) / 2.0
    
    offsets = []
    for x, y in path:
        # Map Image coordinates to Robot Physical Coordinates:
        # Image X (left/right) maps to Python -Y (left/right, where +Y is Left)
        # Image Y (top/bottom) maps to Python -X (forward/back, where +X is Forward)
        py_y = -(x - cx) * scale
        py_x = -(y - cy) * scale
        offsets.append((py_x, py_y))
        
    return offsets

def main():
    print(f"Reading logo from {LOGO_PATH}...")
    try:
        offsets = extract_boundary_path(LOGO_PATH, num_points=60, size_mm=40.0)
    except Exception as e:
        print(f"Failed to process logo: {e}")
        return
        
    print(f"Extracted {len(offsets)} points along the boundary.")
    print("Connecting to robot to read the current center position...")
    
    try:
        base_pos = get_base_pos()
    except Exception as e:
        print(e)
        print("\nPlease move the robot to the desired center of the logo and ensure the server is running.")
        return
        
    base_shoulder, base_elbow, base_wrist, base_z = base_pos
    print(f"Center set at servo ticks: S={base_shoulder}, E={base_elbow}, W={base_wrist}, Z={base_z}")
    
    # Use server's kinematics to convert the base pose to physical mm
    base_phy_x, base_phy_y = server.forward_kinematics(base_shoulder, base_elbow)
    cx, cy = server.get_correction_offset(base_phy_x, base_phy_y)
    base_tip_x = base_phy_x - cx
    base_tip_y = base_phy_y - cy
    
    sequence = []
    skipped = 0
    
    print("Computing robot coordinates for each point...")
    for ox, oy in offsets:
        target_tip_x = base_tip_x + ox
        target_tip_y = base_tip_y + oy
        try:
            fsx, fex = server.inverse_kinematics(target_tip_x, target_tip_y)
            # Apply wrist correction to keep the tool angle consistent
            world_angle = base_shoulder + base_elbow + base_wrist
            new_wrist = int(world_angle - fsx - fex)
            
            pose = [int(fsx), int(fex), new_wrist, base_z]
            sequence.append({"action": "dispense", "pos": pose})
        except ValueError:
            skipped += 1
            
    print(f"Successfully generated {len(sequence)} reachable points (skipped {skipped} out of bounds).")
    
    if len(sequence) == 0:
        print("No reachable points! Try moving the center position further away from the edges of the robot's reach.")
        return

    choice = input("Send this sequence to the robot to print now? (y/n): ")
    if choice.lower().startswith('y'):
        print("Sending sequence to robot...")
        res = requests.post("http://127.0.0.1:5000/run_sequence", json={
            "sequence": sequence,
            "repeat_count": 1,
            "dispense_dwell": 0.5  # Half second per point should be plenty for a dot
        })
        if res.status_code == 200:
            print("Server response:", res.json())
            print("Printing started! You can monitor the progress in the UI.")
        else:
            print("Failed to start print:", res.text)
    else:
        print("Aborted.")

if __name__ == '__main__':
    main()
