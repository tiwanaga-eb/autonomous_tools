"""Export smoothed route to JSON and CSV files."""
import json
import os
import csv
import numpy as np


def export_route(route_xyz: np.ndarray, meta: dict, out_dir: str = "output") -> None:
    """Save route waypoints to output/route.json and output/route.csv."""
    os.makedirs(out_dir, exist_ok=True)

    # Compute total distance
    diffs = np.diff(route_xyz, axis=0)
    total_distance = float(np.sum(np.linalg.norm(diffs, axis=1)))

    waypoints = [
        {"index": int(i), "x_m": float(pt[0]), "y_m": float(pt[1]), "z_m": float(pt[2])}
        for i, pt in enumerate(route_xyz)
    ]

    payload = {
        "generator": "terrain_route_planner",
        "input_las": meta.get("input_las", None),
        "input_tin": meta.get("input_tin", None),
        "cell_size_m": meta.get("cell_size_m", 1.0),
        "slope_limit_deg": meta.get("slope_limit_deg", 20.0),
        "total_distance_m": round(total_distance, 2),
        "total_waypoints": len(route_xyz),
        "waypoints": waypoints,
    }

    json_path = os.path.join(out_dir, "route.json")
    with open(json_path, "w") as f:
        json.dump(payload, f, indent=2)
    print(f"Exported: {json_path}")

    csv_path = os.path.join(out_dir, "route.csv")
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["index", "x_m", "y_m", "z_m"])
        writer.writeheader()
        writer.writerows(waypoints)
    print(f"Exported: {csv_path}")
