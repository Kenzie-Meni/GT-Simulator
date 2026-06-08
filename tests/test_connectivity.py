"""
tests/test_connectivity.py
Unit tests for connectivity detection and output writing.
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import tempfile, json, csv
from core.utils import classify_tech, dist_m
from output.writer import write_csv, write_json
import config


def test_classify_tech_bt():
    assert classify_tech(0.0)   == "bluetooth"
    assert classify_tech(20.0)  == "bluetooth"
    assert classify_tech(40.0)  == "bluetooth"
    print("PASS test_classify_tech_bt")


def test_classify_tech_wifi():
    assert classify_tech(41.0)  == "wifi"
    assert classify_tech(60.0)  == "wifi"
    print("PASS test_classify_tech_wifi")


def test_classify_tech_none():
    assert classify_tech(61.0)  == "none"
    assert classify_tech(200.0) == "none"
    print("PASS test_classify_tech_none")


def test_write_csv():
    windows = [
        {
            "node_a": "COI", "node_b": "WIS_N", "tech": "bluetooth",
            "start_time": 10.0, "end_time": 25.0, "duration": 15.0,
            "min_dist": 5.0, "max_dist": 38.0,
            "start_lat_a": 38.906, "start_lon_a": -77.059,
            "start_lat_b": 38.906, "start_lon_b": -77.059,
            "end_lat_a":   38.907, "end_lon_a":   -77.058,
            "end_lat_b":   38.906, "end_lon_b":   -77.059,
        }
    ]
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "test.csv")
        write_csv(windows, path)
        with open(path) as f:
            rows = list(csv.DictReader(f))
        assert len(rows) == 1
        assert rows[0]["tech"] == "bluetooth"
        assert float(rows[0]["duration"]) == 15.0
    print("PASS test_write_csv")


def test_write_json():
    from simulation.static_node import StaticNode
    from moo.nsga2 import decode as decode_fn

    windows = []
    node = StaticNode(
        node_id="WIS_N", x=100.0, y=200.0,
        mother_x=500.0, mother_y=500.0,
        decode_fn=decode_fn, road_length=800.0
    )
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "test.json")
        write_json(windows, [node], ["N_37", "WIS_N"], {}, path)
        with open(path) as f:
            j = json.load(f)
        assert "simulation" in j
        assert "static_nodes" in j
        assert "WIS_N" in j["static_nodes"]
    print("PASS test_write_json")


if __name__ == "__main__":
    test_classify_tech_bt()
    test_classify_tech_wifi()
    test_classify_tech_none()
    test_write_csv()
    test_write_json()
    print("\nAll connectivity tests passed.")
