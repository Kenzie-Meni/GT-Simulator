"""
run_simulation.py
Main entry point — run this file to execute the full simulation.

    python run_simulation.py

Outputs land in the directory specified by config.OUTPUT_DIR (default: data/)
    data/connectivity.csv
    data/connectivity.json
    data/simulation.mp4          (if config.RECORD_ANIMATION is True)
"""

import os
import random
import numpy as np
import networkx as nx

import config
from network.georgetown import load_graph, make_random_route
from moo.nsga2 import nsga2
from simulation.vehicle import Vehicle
from simulation.static_node import StaticNode
from simulation.engine import run, SimulationResult
from output.writer import write_csv, write_json
from output.animator import build_animation


def main():
    random.seed(config.SEED)
    np.random.seed(config.SEED)
    os.makedirs(config.OUTPUT_DIR, exist_ok=True)

    # ── 1. Load road network ───────────────────────────────────────────────
    print("=" * 60)
    print("DTN Georgetown Simulation")
    print("=" * 60)
    G = load_graph(try_osmnx=False)
    node_list = list(G.nodes())

    # Bounding box for road_length approximation
    all_x = [G.nodes[n]["x"] for n in G.nodes()]
    all_y = [G.nodes[n]["y"] for n in G.nodes()]
    road_length = max(max(all_x) - min(all_x), max(all_y) - min(all_y))

    # ── 2. Offline NSGA-II ─────────────────────────────────────────────────
    print("\n[moo] Running offline NSGA-II...")
    pareto_front, decode_fn = nsga2()
    print(f"[moo] Pareto front: {len(pareto_front)} solutions")

    # ── 3. Static IoT nodes ────────────────────────────────────────────────
    static_ids = [n for n in config.STATIC_NODE_IDS if n in G.nodes()]
    # Use centre-of-graph as mother node proxy (or last static node)
    mother_x = float(np.mean(all_x))
    mother_y = float(np.mean(all_y))

    static_nodes = [
        StaticNode(
            node_id     = nid,
            x           = G.nodes[nid]["x"],
            y           = G.nodes[nid]["y"],
            mother_x    = mother_x,
            mother_y    = mother_y,
            decode_fn   = decode_fn,
            road_length = road_length,
        )
        for nid in static_ids
    ]
    print(f"[sim] Static nodes: {static_ids}")

    # ── 4. Build COI circuit ───────────────────────────────────────────────
    circuit = [n for n in config.COI_CIRCUIT if n in G.nodes()]
    # Fill gaps in circuit with shortest paths
    full_circuit = []
    for i in range(len(circuit) - 1):
        try:
            path = nx.shortest_path(G, circuit[i], circuit[i + 1])
            full_circuit.extend(path[:-1])
        except nx.NetworkXNoPath:
            pass
    if circuit:
        full_circuit.append(circuit[-1])
    if len(full_circuit) < 3:
        full_circuit = node_list[:6]  # fallback
    print(f"[sim] COI circuit: {len(full_circuit)} waypoints")

    # ── 5. Build vehicles ──────────────────────────────────────────────────
    coi = Vehicle(
        vid     = "COI",
        x       = G.nodes[full_circuit[0]]["x"],
        y       = G.nodes[full_circuit[0]]["y"],
        route   = full_circuit,
        speed   = config.COI_START_SPEED,
        heading = 0.0,
        is_coi  = True,
        color   = "#f72585",
    )
    from core.utils import xy2ll
    coi.lat, coi.lon = xy2ll(coi.x, coi.y)

    # ── 5b. Build escort vehicles ──────────────────────────────────────────
    escort_circuit_defs = [config.ESCORT_1_CIRCUIT, config.ESCORT_2_CIRCUIT]
    escorts = []
    for i in range(min(config.NUM_ESCORTS, len(escort_circuit_defs))):
        raw = [n for n in escort_circuit_defs[i] if n in G.nodes()]
        if len(raw) < 2:
            continue
        full_esc = []
        for j in range(len(raw) - 1):
            try:
                path = nx.shortest_path(G, raw[j], raw[j + 1])
                full_esc.extend(path[:-1])
            except nx.NetworkXNoPath:
                pass
        if raw:
            full_esc.append(raw[-1])
        if len(full_esc) < 2:
            continue
        start_node = full_esc[0]
        e = Vehicle(
            vid       = f"ESC{i + 1}",
            x         = G.nodes[start_node]["x"],
            y         = G.nodes[start_node]["y"],
            route     = full_esc,
            speed     = config.COI_START_SPEED,
            heading   = 0.0,
            is_coi    = False,
            is_escort = True,
            color     = config.ESCORT_COLORS[i % len(config.ESCORT_COLORS)],
        )
        e.lat, e.lon = xy2ll(e.x, e.y)
        escorts.append(e)
    print(f"[sim] Escort vehicles: {[e.vid for e in escorts]}")

    traffic_colors = [
        "#4caf50", "#b5ead7", "#c77dff", "#90e0ef", "#caffbf",
        "#a8dadc", "#ffe066", "#d4a5a5", "#aed9a8",
    ]
    eligible = [n for n in node_list if G.degree(n) > 1]
    traffic = []
    for i in range(config.NUM_VEHICLES):
        start     = random.choice(eligible)
        is_follow = i < config.NUM_FOLLOWERS
        color     = (config.FOLLOWER_COLORS[i % len(config.FOLLOWER_COLORS)]
                     if is_follow
                     else traffic_colors[i % len(traffic_colors)])
        # Short seed route — dynamic routing takes over at intersections
        route = make_random_route(G, start, config.ROUTE_LENGTH)
        v = Vehicle(
            vid             = f"V{i:02d}",
            x               = G.nodes[start]["x"],
            y               = G.nodes[start]["y"],
            route           = route,
            speed           = random.uniform(config.VEHICLE_SPEED_MIN,
                                             config.VEHICLE_SPEED_MAX),
            heading         = random.uniform(0, 6.283),
            dynamic_routing = True,
            is_follower     = is_follow,
            color           = color,
        )
        v.lat, v.lon = xy2ll(v.x, v.y)
        traffic.append(v)
    n_follow = sum(1 for v in traffic if v.is_follower)
    print(f"[sim] Traffic: {len(traffic)} vehicles  ({n_follow} followers)")

    vehicles = [coi] + escorts + traffic
    print(f"[sim] Vehicles: 1 COI + {len(escorts)} escorts + {len(traffic)} traffic")

    # ── 6. Run simulation ──────────────────────────────────────────────────
    print(f"\n[sim] Simulating {config.SIM_DURATION}s "
          f"({config.SIM_DURATION//60} min) at dt={config.DT}s ...")
    pre_result = SimulationResult()
    pre_result.mother_x = mother_x
    pre_result.mother_y = mother_y
    result = run(G, vehicles, static_nodes, result=pre_result, verbose=False)

    n_total = result.stats["total_messages"]
    n_del   = result.stats["delivered"]
    print(f"\n[sim] Done.")
    print(f"  Messages generated  : {n_total}")
    print(f"  Delivered           : {n_del}  "
          f"({'%.1f%%' % (result.delivery_rate*100)})")
    print(f"  Avg delivery delay  : {result.avg_delay:.0f}s")
    print(f"  Connectivity windows: {len(result.connectivity_log)}")
    print(f"  LP solves           : {result.stats.get('scheduler_lp_success',0)}")
    print(f"  LP fallbacks        : {result.stats.get('scheduler_lp_fallback',0)}")
    print(f"\n[file] File transfer results:")
    print(f"  Source node         : {result.stats.get('file_source_node','?')}")
    print(f"  Chunks delivered    : {result.stats.get('chunks_delivered',0)} / "
          f"{result.stats.get('chunks_total',0)}  "
          f"({result.stats.get('chunk_completion_pct',0):.1f}%)")
    print(f"  ACKs generated      : {result.stats.get('acks_generated',0)}")

    # ── 7. Write outputs ───────────────────────────────────────────────────
    static_pos_map = {n.node_id: (n.x, n.y) for n in static_nodes}

    csv_path  = os.path.join(config.OUTPUT_DIR, "connectivity.csv")
    json_path = os.path.join(config.OUTPUT_DIR, "connectivity.json")
    write_csv(result.connectivity_log, csv_path)
    write_json(result.connectivity_log, static_nodes,
               full_circuit, result.stats, json_path)

    # ── 8. Animation ───────────────────────────────────────────────────────
    if config.RECORD_ANIMATION:
        mp4_path = os.path.join(config.OUTPUT_DIR, "simulation.mp4")
        build_animation(
            G, result, static_pos_map, full_circuit, mp4_path,
            mother_pos=(mother_x, mother_y),
        )

    print("\n[done] All outputs in:", config.OUTPUT_DIR)


if __name__ == "__main__":
    main()
