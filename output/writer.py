"""
output/writer.py
CSV and JSON output writers for connectivity logs.
"""

import csv
import json
import os
from typing import List
from core.utils import xy2ll
import config


CSV_FIELDS = [
    "node_a", "node_b", "tech",
    "start_time", "end_time", "duration",
    "min_dist_m", "max_dist_m",
    "start_lat_a", "start_lon_a",
    "start_lat_b", "start_lon_b",
    "end_lat_a",   "end_lon_a",
    "end_lat_b",   "end_lon_b",
]


def write_csv(connectivity_log: List[dict], path: str) -> None:
    """Write connectivity windows to CSV."""
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS, extrasaction="ignore")
        writer.writeheader()
        for w in connectivity_log:
            writer.writerow({
                "node_a":      w["node_a"],
                "node_b":      w["node_b"],
                "tech":        w["tech"],
                "start_time":  round(w["start_time"], 1),
                "end_time":    round(w["end_time"],   1),
                "duration":    round(w["duration"],   1),
                "min_dist_m":  round(w["min_dist"],   2),
                "max_dist_m":  round(w["max_dist"],   2),
                "start_lat_a": round(w["start_lat_a"], 7),
                "start_lon_a": round(w["start_lon_a"], 7),
                "start_lat_b": round(w["start_lat_b"], 7),
                "start_lon_b": round(w["start_lon_b"], 7),
                "end_lat_a":   round(w["end_lat_a"],   7),
                "end_lon_a":   round(w["end_lon_a"],   7),
                "end_lat_b":   round(w["end_lat_b"],   7),
                "end_lon_b":   round(w["end_lon_b"],   7),
            })
    print(f"[output] CSV written: {path}  ({len(connectivity_log)} rows)")


def write_json(
    connectivity_log: List[dict],
    nodes: dict,
    coi_circuit: List[str],
    stats: dict,
    path: str,
) -> None:
    """Write full simulation summary and connectivity windows to JSON."""
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)

    bt_windows   = [w for w in connectivity_log if w["tech"] == "bluetooth"]
    wifi_windows = [w for w in connectivity_log if w["tech"] == "wifi"]

    windows = [{
        "node_a": w["node_a"], "node_b": w["node_b"],
        "start_time_s": w["start_time"], "end_time_s": w["end_time"],
        "duration_s": w["duration"], "technology": w["tech"],
        "min_distance_m": w["min_dist"], "max_distance_m": w["max_dist"],
    } for w in connectivity_log]

    summary = {
        "schema_version": "2.0",
        "simulation": {
            "duration_s":        config.SIM_DURATION,
            "area":              "Georgetown, Washington DC",
            "bbox":              config.BBOX,
            "bluetooth_range_m": config.BT_RANGE,
            "wifi_range_m":      config.WIFI_RANGE,
            "num_nodes":         len(nodes),
            "default_altitude_m": config.DEFAULT_ALTITUDE_M,
            "coi_circuit":       coi_circuit,
        },
        "nodes": nodes,
        "statistics": {
            "total_windows":       len(connectivity_log),
            "bluetooth_windows":   len(bt_windows),
            "wifi_windows":        len(wifi_windows),
            "coi_windows":         sum(
                1 for w in connectivity_log
                if "COI" in (w["node_a"], w["node_b"])
            ),
            "total_contact_time_s": round(
                sum(w["duration"] for w in connectivity_log), 1
            ),
            "avg_window_duration_s": round(
                sum(w["duration"] for w in connectivity_log)
                / max(1, len(connectivity_log)), 1
            ),
            "longest_window_s": round(
                max((w["duration"] for w in connectivity_log), default=0.0), 1
            ),
            **{k: (float(v) if isinstance(v, float) else int(v))
               for k, v in stats.items()
               if k != "file_source_node"},
            "file_source_node": stats.get("file_source_node", "unknown"),
        },
        "windows": windows,
    }

    with open(path, "w") as f:
        json.dump(summary, f, indent=2, default=float)

    print(f"[output] JSON written: {path}  "
          f"(BT={len(bt_windows)}, WiFi={len(wifi_windows)})")
