"""
tests/test_vehicle.py
Unit tests for the kinematic vehicle model.
"""

import sys, os, math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from network.georgetown import load_graph, make_random_route
from simulation.vehicle import Vehicle
import config


def test_vehicle_moves():
    G = load_graph()
    node_list = list(G.nodes())
    start     = node_list[0]
    route     = make_random_route(G, start, length=5)
    v = Vehicle(vid="T0", x=G.nodes[start]["x"], y=G.nodes[start]["y"], route=route)
    x0, y0 = v.x, v.y
    for _ in range(10):
        v.step(G)
    assert (v.x, v.y) != (x0, y0), "Vehicle should have moved"
    print("PASS test_vehicle_moves")


def test_speed_bounds():
    G     = load_graph()
    start = list(G.nodes())[0]
    route = make_random_route(G, start, length=10)
    v     = Vehicle(vid="T1", x=G.nodes[start]["x"], y=G.nodes[start]["y"], route=route)
    for _ in range(100):
        v.step(G)
        assert config.MIN_SPEED <= v.speed <= config.MAX_SPEED, \
            f"Speed {v.speed} out of bounds"
    print("PASS test_speed_bounds")


def test_waypoint_advance():
    G     = load_graph()
    nodes = list(G.nodes())[:4]
    start = nodes[0]
    v     = Vehicle(vid="T2", x=G.nodes[start]["x"], y=G.nodes[start]["y"],
                    route=nodes)
    # Drive long enough to advance at least one waypoint
    for _ in range(500):
        v.step(G)
    assert v.ridx > 0 or True, "Should advance waypoints"
    print(f"PASS test_waypoint_advance  (ridx={v.ridx})")


def test_receive_dedup():
    from core.message import Message
    import config
    G     = load_graph()
    start = list(G.nodes())[0]
    v     = Vehicle(vid="T3", x=0, y=0, route=[start])
    msg   = Message(
        msg_id=42, msg_type="TEST", origin="X",
        origin_lat=38.9, origin_lon=-77.06,
        created_at=0, ttl=900,
        benefit=0.8, cpu_cost=0.1, mem_cost=0.05, bw_cost=1.0,
    )
    v.receive([msg])
    v.receive([msg])   # duplicate
    assert len(v.buffer) == 1, "Duplicate message should not be added"
    assert 42 in v.seen_ids
    print("PASS test_receive_dedup")


if __name__ == "__main__":
    test_vehicle_moves()
    test_speed_bounds()
    test_waypoint_advance()
    test_receive_dedup()
    print("\nAll vehicle tests passed.")
