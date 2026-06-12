"""
run_simulation.py
Main entry point — run this file to execute the full simulation.

    python run_simulation.py

Outputs land in the directory specified by config.OUTPUT_DIR (default: data/)
    data/connectivity.csv
    data/connectivity.json
    data/simulation.mp4          (if config.RECORD_ANIMATION is True)

The simulate() function can also be imported by run_batch.py for multi-run
batch collection without animation or file output.
"""

import math
import os
import random
import numpy as np
import networkx as nx

import config
from network.georgetown import load_graph, make_random_route, name_to_node
from moo.nsga2 import nsga2
from simulation.vehicle import Vehicle
from simulation.static_node import StaticNode
from simulation.engine import run, SimulationResult
from output.writer import write_csv, write_json
from output.animator import build_animation


# ── Stats helpers ──────────────────────────────────────────────────────────────

def _compute_stats(result: SimulationResult, seed: int, dest_label: str) -> dict:
    """
    Distil a SimulationResult into a flat dict of scalar metrics suitable
    for CSV export and aggregate analysis.
    """
    # ── Sightings ──────────────────────────────────────────────────────────
    sightings   = [m for m in result.all_messages if m.msg_type == "COI_SIGHTING"]
    n_sightings = len(sightings)
    delivered   = result.delivered_msgs          # COI_SIGHTING only

    delays = [m.delivery_time - m.created_at for m in delivered]
    hops   = [m.hop_count for m in delivered]
    t_first_sight = (min(m.delivery_time for m in delivered)
                     if delivered else float("nan"))
    expired_sightings = sum(
        1 for m in sightings
        if not m.delivered and m.is_expired(config.SIM_DURATION)
    )

    # ── Chunks ─────────────────────────────────────────────────────────────
    chunks_total     = result.stats["chunks_total"]
    chunks_delivered = result.stats["chunks_delivered"]
    chunk_msgs       = result.delivered_chunks
    chunk_delays     = [m.delivery_time - m.created_at for m in chunk_msgs]
    t_first_chunk    = (min(m.delivery_time for m in chunk_msgs)
                        if chunk_msgs else float("nan"))

    # ── ACKs ───────────────────────────────────────────────────────────────
    acks_generated = result.stats["acks_generated"]
    acks_delivered = result.stats.get("acks_delivered_to_source", 0)

    # ── Network ────────────────────────────────────────────────────────────
    clog      = result.connectivity_log
    n_windows = len(clog)
    bt_wins   = sum(1 for w in clog if w.get("tech") == "bluetooth")
    wifi_wins = sum(1 for w in clog if w.get("tech") == "wifi")
    durations = [w["duration"] for w in clog]
    avg_dur   = sum(durations) / len(durations) if durations else 0.0

    def _safe_mean(lst):
        return sum(lst) / len(lst) if lst else float("nan")

    return {
        "seed":                    seed,
        "dest_label":              dest_label,
        "file_source_node":        result.stats.get("file_source_node", "?"),
        # Sightings
        "sightings_generated":     n_sightings,
        "sightings_delivered":     len(delivered),
        "sighting_delivery_rate":  len(delivered) / n_sightings if n_sightings > 0 else 0.0,
        "sighting_avg_delay_s":    _safe_mean(delays),
        "sighting_min_delay_s":    min(delays) if delays else float("nan"),
        "sighting_max_delay_s":    max(delays) if delays else float("nan"),
        "sighting_p50_delay_s":    float(np.median(delays)) if delays else float("nan"),
        "sighting_avg_hops":       _safe_mean(hops),
        "sightings_expired":       expired_sightings,
        "time_to_first_sighting_s": t_first_sight,
        # Chunks
        "chunks_total":            chunks_total,
        "chunks_delivered":        chunks_delivered,
        "chunk_completion_pct":    chunks_delivered / chunks_total * 100.0 if chunks_total > 0 else 0.0,
        "chunks_per_min":          chunks_delivered / (config.SIM_DURATION / 60.0),
        "time_to_first_chunk_s":   t_first_chunk,
        "chunk_avg_delay_s":       _safe_mean(chunk_delays),
        # ACKs
        "acks_generated":          acks_generated,
        "acks_delivered":          acks_delivered,
        "ack_delivery_rate":       acks_delivered / acks_generated if acks_generated > 0 else 0.0,
        # Network
        "connectivity_windows":    n_windows,
        "bt_windows":              bt_wins,
        "wifi_windows":            wifi_wins,
        "avg_window_duration_s":   avg_dur,
        "lp_solves":               result.stats.get("scheduler_lp_success", 0),
        "lp_fallbacks":            result.stats.get("scheduler_lp_fallback", 0),
        "total_transfers":         result.stats.get("total_transfers", 0),
    }


def _fmt(val, fmt=".1f"):
    return f"{val:{fmt}}" if not (isinstance(val, float) and math.isnan(val)) else "N/A"


def _print_summary(stats: dict) -> None:
    W = 48
    print("=" * W)
    print(f"  DTN Run Summary   seed={stats['seed']}")
    print("=" * W)
    print(f"  Destination      : {stats['dest_label']}")
    print(f"  File source node : {stats['file_source_node']}")

    print("\n  COI SIGHTINGS")
    print(f"    Generated      : {stats['sightings_generated']}")
    dr = stats['sighting_delivery_rate']
    print(f"    Delivered      : {stats['sightings_delivered']}  ({dr*100:.1f}%)")
    print(f"    Avg / P50 delay: {_fmt(stats['sighting_avg_delay_s'])}s "
          f"/ {_fmt(stats['sighting_p50_delay_s'])}s")
    print(f"    Min / Max delay: {_fmt(stats['sighting_min_delay_s'])}s "
          f"/ {_fmt(stats['sighting_max_delay_s'])}s")
    print(f"    Avg hops       : {_fmt(stats['sighting_avg_hops'])}")
    print(f"    Expired        : {stats['sightings_expired']}")
    print(f"    Time to first  : {_fmt(stats['time_to_first_sighting_s'])}s")

    print("\n  FILE CHUNKS")
    print(f"    Total / Delivered: {stats['chunks_total']} / {stats['chunks_delivered']}  "
          f"({stats['chunk_completion_pct']:.1f}%)")
    print(f"    Rate           : {_fmt(stats['chunks_per_min'])} chunks/min")
    print(f"    Avg delay      : {_fmt(stats['chunk_avg_delay_s'])}s")
    print(f"    Time to first  : {_fmt(stats['time_to_first_chunk_s'])}s")

    print("\n  FILE ACKs")
    ar = stats['ack_delivery_rate']
    print(f"    Generated      : {stats['acks_generated']}")
    print(f"    Delivered      : {stats['acks_delivered']}  ({ar*100:.1f}%)")

    print("\n  NETWORK")
    print(f"    Connectivity windows : {stats['connectivity_windows']}  "
          f"(BT: {stats['bt_windows']}, WiFi: {stats['wifi_windows']})")
    print(f"    Avg window duration  : {_fmt(stats['avg_window_duration_s'])}s")
    print(f"    LP solves / fallbacks: {stats['lp_solves']} / {stats['lp_fallbacks']}")
    print(f"    Total transfers      : {stats['total_transfers']}")
    print("=" * W)


# ── Core simulate function ─────────────────────────────────────────────────────

def simulate(
    seed:        int  = None,
    G:           nx.Graph = None,
    decode_fn          = None,
    animate:     bool = True,
    write_files: bool = True,
    verbose:     bool = False,
) -> dict:
    """
    Run one complete simulation and return a stats dict.

    Parameters
    ----------
    seed        : RNG seed; uses config.SEED if None
    G           : pre-loaded road graph (loaded from OSM if None)
    decode_fn   : NSGA-II decode function (computed if None)
    animate     : whether to build and save the MP4 animation
    write_files : whether to write connectivity CSV / JSON
    verbose     : print per-event delivery lines to stdout

    Returns
    -------
    Flat dict of scalar metrics (see _compute_stats for keys).
    """
    if seed is None:
        seed = config.SEED
    random.seed(seed)
    np.random.seed(seed)
    os.makedirs(config.OUTPUT_DIR, exist_ok=True)

    # ── Road network ──────────────────────────────────────────────────────
    if G is None:
        G = load_graph(try_osmnx=True)
    node_list = list(G.nodes())
    node_map  = name_to_node(G)

    all_x = [G.nodes[n]["x"] for n in G.nodes()]
    all_y = [G.nodes[n]["y"] for n in G.nodes()]
    road_length = max(max(all_x) - min(all_x), max(all_y) - min(all_y))

    # ── NSGA-II ───────────────────────────────────────────────────────────
    if decode_fn is None:
        _, decode_fn = nsga2()

    # ── Destination node ──────────────────────────────────────────────────
    from core.utils import dist_m as _dm
    dest_node_id = random.choice(node_list)
    mother_x     = G.nodes[dest_node_id]["x"]
    mother_y     = G.nodes[dest_node_id]["y"]
    dest_label = min(
        node_map.items(),
        key=lambda kv: _dm(G.nodes[kv[1]]["x"], G.nodes[kv[1]]["y"],
                           mother_x, mother_y) if kv[1] else float("inf")
    )[0]

    # ── Static IoT nodes ──────────────────────────────────────────────────
    from core.utils import xy2ll
    static_ids = [n for n in config.STATIC_NODE_IDS if node_map.get(n)]
    static_nodes = [
        StaticNode(
            node_id     = nid,
            x           = G.nodes[node_map[nid]]["x"],
            y           = G.nodes[node_map[nid]]["y"],
            mother_x    = mother_x,
            mother_y    = mother_y,
            decode_fn   = decode_fn,
            road_length = road_length,
        )
        for nid in static_ids
    ]

    # Edge-positioned IoT nodes — spatially stratified
    edge_list = list(G.edges())
    x_min, x_max = min(all_x), max(all_x)
    strip_w = (x_max - x_min) / config.NUM_EDGE_NODES
    for i in range(config.NUM_EDGE_NODES):
        strip_lo = x_min + i * strip_w
        strip_hi = strip_lo + strip_w
        eligible = [
            (u, v) for u, v in edge_list
            if strip_lo <= (G.nodes[u]["x"] + G.nodes[v]["x"]) / 2 < strip_hi
        ]
        if not eligible:
            eligible = edge_list
        u, v = random.choice(eligible)
        t_   = random.uniform(0.15, 0.85)
        ex   = G.nodes[u]["x"] + t_ * (G.nodes[v]["x"] - G.nodes[u]["x"])
        ey   = G.nodes[u]["y"] + t_ * (G.nodes[v]["y"] - G.nodes[u]["y"])
        static_nodes.append(StaticNode(
            node_id     = f"EDGE_{i}",
            x           = float(ex),
            y           = float(ey),
            mother_x    = mother_x,
            mother_y    = mother_y,
            decode_fn   = decode_fn,
            road_length = road_length,
        ))

    # Randomise static node radio types — 1/3 BT / 1/3 WiFi / 1/3 both
    shuffled = static_nodes[:]
    random.shuffle(shuffled)
    n_nodes  = len(shuffled)
    bt_cut   = n_nodes // 3
    wifi_cut = bt_cut + n_nodes // 3
    for i, node in enumerate(shuffled):
        node.radio_type = "bt" if i < bt_cut else ("wifi" if i < wifi_cut else "both")

    # ── COI circuit ───────────────────────────────────────────────────────
    circuit = [node_map[n] for n in config.COI_CIRCUIT if node_map.get(n)]
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
        full_circuit = node_list[:6]

    # ── Vehicles ──────────────────────────────────────────────────────────
    coi = Vehicle(
        vid        = "COI",
        x          = G.nodes[full_circuit[0]]["x"],
        y          = G.nodes[full_circuit[0]]["y"],
        route      = full_circuit,
        speed      = config.COI_START_SPEED,
        heading    = 0.0,
        is_coi     = True,
        color      = "#f72585",
        radio_type = "both",
    )
    coi.lat, coi.lon = xy2ll(coi.x, coi.y)

    escort_circuit_defs = [config.ESCORT_1_CIRCUIT, config.ESCORT_2_CIRCUIT]
    escorts = []
    for i in range(min(config.NUM_ESCORTS, len(escort_circuit_defs))):
        raw = [node_map[n] for n in escort_circuit_defs[i] if node_map.get(n)]
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
        e = Vehicle(
            vid        = f"ESC{i + 1}",
            x          = G.nodes[full_esc[0]]["x"],
            y          = G.nodes[full_esc[0]]["y"],
            route      = full_esc,
            speed      = config.COI_START_SPEED,
            heading    = 0.0,
            is_coi     = False,
            is_escort  = True,
            color      = config.ESCORT_COLORS[i % len(config.ESCORT_COLORS)],
            radio_type = "both",
        )
        e.lat, e.lon = xy2ll(e.x, e.y)
        escorts.append(e)

    traffic_colors = [
        "#4caf50", "#b5ead7", "#c77dff", "#90e0ef", "#caffbf",
        "#a8dadc", "#ffe066", "#d4a5a5", "#aed9a8",
    ]
    _rt_pool = (["bt"]   * (config.NUM_VEHICLES // 3)
              + ["wifi"] * (config.NUM_VEHICLES // 3)
              + ["both"] * (config.NUM_VEHICLES - 2 * (config.NUM_VEHICLES // 3)))
    random.shuffle(_rt_pool)

    eligible = [n for n in node_list if G.degree(n) > 1]
    traffic = []
    for i in range(config.NUM_VEHICLES):
        start     = random.choice(eligible)
        is_follow = i < config.NUM_FOLLOWERS
        color     = (config.FOLLOWER_COLORS[i % len(config.FOLLOWER_COLORS)]
                     if is_follow else traffic_colors[i % len(traffic_colors)])
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
            radio_type      = _rt_pool[i],
        )
        v.lat, v.lon = xy2ll(v.x, v.y)
        traffic.append(v)

    vehicles = [coi] + escorts + traffic

    # ── Engine ────────────────────────────────────────────────────────────
    pre_result = SimulationResult()
    pre_result.mother_x = mother_x
    pre_result.mother_y = mother_y
    result = run(G, vehicles, static_nodes, result=pre_result, verbose=verbose)

    static_pos_map = {n.node_id: (n.x, n.y) for n in static_nodes}

    # ── File outputs ──────────────────────────────────────────────────────
    if write_files:
        csv_path  = os.path.join(config.OUTPUT_DIR, "connectivity.csv")
        json_path = os.path.join(config.OUTPUT_DIR, "connectivity.json")
        write_csv(result.connectivity_log, csv_path)
        write_json(result.connectivity_log, static_nodes,
                   full_circuit, result.stats, json_path)

    # ── Animation ─────────────────────────────────────────────────────────
    if animate:
        mp4_path = os.path.join(config.OUTPUT_DIR, "simulation.mp4")
        build_animation(
            G, result, static_pos_map, full_circuit, mp4_path,
            mother_pos=(mother_x, mother_y),
            dest_label=dest_label,
            static_nodes=static_nodes,
        )

    return _compute_stats(result, seed, dest_label)


# ── CLI entry point ────────────────────────────────────────────────────────────

def main():
    print("=" * 60)
    print("DTN Georgetown Simulation")
    print("=" * 60)

    G = load_graph(try_osmnx=True)
    print("[moo] Running offline NSGA-II...")
    pareto_front, decode_fn = nsga2()
    print(f"[moo] Pareto front: {len(pareto_front)} solutions")

    stats = simulate(
        seed        = config.SEED,
        G           = G,
        decode_fn   = decode_fn,
        animate     = config.RECORD_ANIMATION,
        write_files = True,
        verbose     = False,
    )
    _print_summary(stats)
    print("\n[done] All outputs in:", config.OUTPUT_DIR)


if __name__ == "__main__":
    main()
