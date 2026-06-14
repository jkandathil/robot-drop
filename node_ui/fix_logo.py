import json

cal_file = r"c:\Users\jayan.kandathil\Documents\andrew-robot_test\node_ui\calibration.json"
logo_file = r"c:\Users\jayan.kandathil\Documents\andrew-robot_test\node_ui\logo_print.json"

with open(cal_file, "r") as f:
    cal = json.load(f)

wells = cal.get("locations", {}).get("wells", [])

locations = []
sequence = []

for w in wells:
    locations.append({
        "name": w["name"],
        "type": "WELL",
        "pos": w["pose"]
    })
    sequence.append({
        "action": "dispense",
        "pos": w["pose"]
    })

out = {
    "locations": locations,
    "sequence": sequence
}

with open(logo_file, "w") as f:
    json.dump(out, f, indent=2)

print(f"Saved {len(locations)} points to logo_print.json!")
