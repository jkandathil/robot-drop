import os
import json
from PIL import Image, ImageFilter

LOGO_PATH = os.path.join('..', 'logo_noze', 'noze_logo.png')
OUTPUT_JSON = 'logo_uv.json'

def generate_uv_path(image_path, num_points=60):
    img = Image.open(image_path).convert('RGBA')
    alpha = img.split()[3]
    edges = alpha.filter(ImageFilter.FIND_EDGES)
    
    w, h = edges.size
    px = edges.load()
    
    edge_pts = []
    for y in range(h):
        for x in range(w):
            if px[x, y] > 50:
                edge_pts.append((x, y))
                
    if not edge_pts:
        raise Exception("No edges found in the image!")
        
    path = []
    current = edge_pts.pop(0)
    path.append(current)
    
    while edge_pts:
        nearest_idx = min(range(len(edge_pts)), key=lambda i: (edge_pts[i][0]-current[0])**2 + (edge_pts[i][1]-current[1])**2)
        current = edge_pts.pop(nearest_idx)
        path.append(current)
        
    if len(path) > num_points:
        step = len(path) / float(num_points)
        path = [path[int(i * step)] for i in range(num_points)]
        
    min_x = min(p[0] for p in path)
    max_x = max(p[0] for p in path)
    min_y = min(p[1] for p in path)
    max_y = max(p[1] for p in path)
    
    w_px = max_x - min_x
    h_px = max_y - min_y
    
    # Map image to u,v [0, 1] range. 
    # Let's add a small margin (5%) so it doesn't touch the exact boundary of the calibration map.
    margin = 0.05
    scale_factor = (1.0 - 2 * margin) / max(w_px, h_px)
    
    cx = (max_x + min_x) / 2.0
    cy = (max_y + min_y) / 2.0
    
    uv_points = []
    for x, y in path:
        # u is the X axis (left to right)
        # v is the Y axis (top to bottom)
        # Image x is left to right, Image y is top to bottom
        u = 0.5 + (x - cx) * scale_factor
        v = 0.5 + (y - cy) * scale_factor
        uv_points.append({"u": u, "v": v})
        
    return uv_points

def main():
    print(f"Reading logo from {LOGO_PATH}...")
    try:
        uv_points = generate_uv_path(LOGO_PATH, num_points=60)
    except Exception as e:
        print(f"Failed to process logo: {e}")
        return
        
    print(f"Extracted {len(uv_points)} (u, v) points.")
    
    with open(OUTPUT_JSON, 'w') as f:
        json.dump(uv_points, f, indent=2)
        
    print(f"Saved UV coordinates to {OUTPUT_JSON}.")

if __name__ == '__main__':
    main()
