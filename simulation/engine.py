"""
simulation/engine.py
Main simulation loop.

Orchestrates:
  - Vehicle stepping (kinematic physics)
  - Contact detection (BT / WiFi range checks)
  - Scheduler invocation at each contact event
  - COI sighting message generation
  - Connectivity window tracking
  - Frame recording for animation
"""

import math
import random
from collections import defaultdict
from typing import List, Dict, Any, Callable

import networkx as nx

from core.utils import dist_m, xy2ll, classify_tech
from core.message import Message
from simulation.vehicle import Vehicle
from simulation.static_node import StaticNode
import config


# ── Contact window tracker ─────────────────────────────────────────────────

def _open_window(entity_a: str, ax: float, ay: float,
                 entity_b: str, bx: float, by: float,
                 dist: float, t: float) -> dict:
    la, loa = xy2ll(ax, ay)
    lb, lob = xy2ll(bx, by)
    return {
        "node_a": entity_a, "node_b": entity_b,
        "start_time": t, "end_time": -1.0, "duration": 0.0,
        "min_dist": dist, "max_dist": dist, "samples": 1,
        "tech": "unknown",          # resolved at close by classify_tech
        "start_lat_a": la, "start_lon_a": loa,
        "start_lat_b": lb, "start_lon_b": lob,
        "end_lat_a": la, "end_lon_a": loa,
        "end_lat_b": lb, "end_lon_b": lob,
    }


def _update_window(w: dict, dist: float, ax: float, ay: float,
                   bx: float, by: float) -> None:
    la, loa = xy2ll(ax, ay)
    lb, lob = xy2ll(bx, by)
    w["min_dist"]  = min(w["min_dist"], dist)
    w["max_dist"]  = max(w["max_dist"], dist)
    w["samples"]  += 1
    w["end_lat_a"] = la; w["end_lon_a"] = loa
    w["end_lat_b"] = lb; w["end_lon_b"] = lob


def _close_window(w: dict, t: float) -> None:
    w["end_time"] = t
    w["duration"] = t - w["start_time"]
    w["tech"]     = classify_tech(w["min_dist"])


# ── Simulation result container ────────────────────────────────────────────

class SimulationResult:
    def __init__(self):
        self.all_messages:     List[Message] = []
        self.delivered_msgs:   List[Message] = []
        self.connectivity_log: List[dict]    = []
        self.frames:           List[dict]    = []
        self.stats:            dict          = defaultdict(int)

    @property
    def delivery_rate(self) -> float:
        n = len(self.all_messages)
        return len(self.delivered_msgs) / n if n > 0 else 0.0

    @property
    def avg_delay(self) -> float:
        n = len(self.delivered_msgs)
        return self.stats["total_delay"] / n if n > 0 else 0.0


# ── Main engine ────────────────────────────────────────────────────────────

def run(
    G:            nx.Graph,
    vehicles:     List[Vehicle],
    static_nodes: List[StaticNode],
    result:       SimulationResult = None,
    verbose:      bool = False,
) -> SimulationResult:
    """
    Run the full simulation.

    Parameters
    ----------
    G            : Georgetown road graph
    vehicles     : list of Vehicle instances (COI first by convention)
    static_nodes : list of StaticNode instances
    result       : optional pre-allocated SimulationResult (created if None)
    verbose      : print delivery events to stdout

    Returns
    -------
    SimulationResult with messages, connectivity log, frames, and stats
    """
    if result is None:
        result = SimulationResult()

    msg_counter   = 0
    active_windows: Dict[tuple, dict] = {}

    static_pos_map = {n.node_id: (n.x, n.y) for n in static_nodes}
    coi = next((v for v in vehicles if v.is_coi), None)

    for t_int in range(0, config.SIM_DURATION, int(config.DT)):
        t = float(t_int)
        contacts_this_step: List[tuple] = []

        # ── Step vehicles ──────────────────────────────────────────────────
        for v in vehicles:
            fp = (coi.x, coi.y) if (v.is_follower and coi is not None) else None
            v.step(G, config.DT, follow_pos=fp)

        # ── COI sighting events ────────────────────────────────────────────
        if coi and t_int % config.COI_SIGHTING_INTERVAL == 0:
            for node in static_nodes:
                if dist_m(coi.x, coi.y, node.x, node.y) <= config.WIFI_RANGE:
                    msg = node.generate_sighting(
                        msg_counter, t, coi.lat, coi.lon
                    )
                    result.all_messages.append(msg)
                    msg_counter += 1

        # ── Contact detection & scheduling ─────────────────────────────────
        for node in static_nodes:
            node.expire_messages(t)
            for veh in vehicles:
                d = dist_m(veh.x, veh.y, node.x, node.y)
                if d > config.WIFI_RANGE:
                    continue

                # Estimate contact window duration
                contact_window = max(
                    2.0,
                    (config.WIFI_RANGE * 2) / max(veh.speed, 0.1)
                )

                # Run epsilon-constraint LP scheduler
                transferred, status = node.schedule_transfer(veh, contact_window, t)
                veh.receive(transferred)

                result.stats["total_transfers"]  += len(transferred)
                result.stats[f"scheduler_{status}"] += 1

                if transferred:
                    contacts_this_step.append(
                        (node.x, node.y, veh.x, veh.y,
                         "bluetooth" if d <= config.BT_RANGE else "wifi")
                    )

        # ── Vehicle-to-vehicle contact (carry and forward) ─────────────────
        for i, va in enumerate(vehicles):
            for vb in vehicles[i + 1:]:
                d = dist_m(va.x, va.y, vb.x, vb.y)
                if d > config.WIFI_RANGE:
                    continue
                # Simple spray: share messages between vehicles
                for msg in list(va.buffer):
                    if (msg.msg_id not in vb.seen_ids
                            and msg.copies_in_net < config.MAX_SPRAY_COPIES
                            and len(vb.buffer) < config.BUFFER_CAPACITY
                            and not msg.is_expired(t)):
                        vb.buffer.append(msg)
                        vb.seen_ids.add(msg.msg_id)
                        msg.copies_in_net += 1
                        msg.hop_count     += 1
                        contacts_this_step.append(
                            (va.x, va.y, vb.x, vb.y,
                             "bluetooth" if d <= config.BT_RANGE else "wifi")
                        )

        # ── Delivery check ─────────────────────────────────────────────────
        for node in static_nodes:
            for veh in vehicles:
                newly = veh.deliver_to_node(node.x, node.y, t)
                result.delivered_msgs.extend(newly)
                for m in newly:
                    delay = t - m.created_at
                    result.stats["total_delay"] += delay
                    if verbose:
                        print(f"  t={t:5.0f}s  {m.msg_id:03d} delivered  "
                              f"hops={m.hop_count}  delay={delay:.0f}s")

        # ── Connectivity window tracking ───────────────────────────────────
        all_entities = (
            [(v.vid, v.x, v.y) for v in vehicles]
            + [(sid, sx, sy) for sid, (sx, sy) in static_pos_map.items()]
        )
        n_ent = len(all_entities)
        for i in range(n_ent):
            for j in range(i + 1, n_ent):
                ia, ax, ay = all_entities[i]
                ib, bx, by = all_entities[j]
                d = dist_m(ax, ay, bx, by)
                key = (ia, ib)

                if d <= config.WIFI_RANGE:
                    if key not in active_windows:
                        active_windows[key] = _open_window(ia, ax, ay, ib, bx, by, d, t)
                    else:
                        _update_window(active_windows[key], d, ax, ay, bx, by)
                else:
                    if key in active_windows:
                        w = active_windows.pop(key)
                        _close_window(w, t)
                        if w["duration"] >= 1.0:
                            result.connectivity_log.append(w)

        # ── Frame recording ────────────────────────────────────────────────
        if t_int % config.FRAME_INTERVAL == 0:
            result.frames.append({
                "time":       t,
                "vehicles":   [v.state_dict() for v in vehicles],
                "contacts":   list(contacts_this_step),
                "n_contacts": len(contacts_this_step),
            })

    # Close any windows still open at end of simulation
    for key, w in active_windows.items():
        _close_window(w, float(config.SIM_DURATION))
        if w["duration"] >= 1.0:
            result.connectivity_log.append(w)

    result.stats["total_messages"] = len(result.all_messages)
    result.stats["delivered"]      = len(result.delivered_msgs)
    result.stats["connectivity_windows"] = len(result.connectivity_log)

    return result
