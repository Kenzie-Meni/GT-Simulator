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

from core.utils import dist_m, xy2ll, classify_tech, compatible_range, contact_tech
from core.message import Message
from simulation.vehicle import Vehicle
from simulation.static_node import StaticNode
import config


# ── Contact window tracker ─────────────────────────────────────────────────

def _open_window(entity_a: str, ax: float, ay: float,
                 entity_b: str, bx: float, by: float,
                 dist: float, t: float,
                 radio_type: str = "both") -> dict:
    la, loa = xy2ll(ax, ay)
    lb, lob = xy2ll(bx, by)
    return {
        "node_a": entity_a, "node_b": entity_b,
        "start_time": t, "end_time": -1.0, "duration": 0.0,
        "min_dist": dist, "max_dist": dist, "samples": 1,
        "tech": "unknown",          # resolved at close by classify_tech
        "radio_type": radio_type,
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
    w["tech"]     = classify_tech(w["min_dist"], w.get("radio_type", "both"))


# ── Simulation result container ────────────────────────────────────────────

class SimulationResult:
    def __init__(self):
        self.all_messages:     List[Message] = []
        self.delivered_msgs:   List[Message] = []
        self.delivered_chunks: List[Message] = []  # FILE_CHUNK messages confirmed at dest
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

    # ── File transfer setup ────────────────────────────────────────────────
    file_source = random.choice(static_nodes)
    # Chunk IDs start well above sighting IDs to avoid collisions
    msg_counter = file_source.init_file_store(start_id=10_000, now=0.0)

    # Mother position for chunk delivery (passed in via result or derived)
    mother_x = getattr(result, "mother_x", None)
    mother_y = getattr(result, "mother_y", None)

    delivered_chunk_ids: set = set()   # chunk_idx values confirmed at destination
    acks_sent: set           = set()   # thresholds already ACKed (e.g. 0.25)
    dest_pending_acks: list  = []      # FILE_ACK messages waiting at destination
    ack_id_counter           = 20_000  # unique IDs for ACK messages

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
                if dist_m(coi.x, coi.y, node.x, node.y) <= node.effective_range:
                    msg = node.generate_sighting(
                        msg_counter, t, coi.lat, coi.lon
                    )
                    result.all_messages.append(msg)
                    msg_counter += 1

        # ── Contact detection & scheduling (node → vehicle) ───────────────
        for node in static_nodes:
            node.expire_messages(t)
            for veh in vehicles:
                d   = dist_m(veh.x, veh.y, node.x, node.y)
                eff = compatible_range(node.radio_type, veh.radio_type)
                if eff == 0.0 or d > eff:
                    continue

                contact_window = max(2.0, (eff * 2) / max(veh.speed, 0.1))

                transferred, status = node.schedule_transfer(veh, contact_window, t)
                veh.receive(transferred, now=t)

                result.stats["total_transfers"]      += len(transferred)
                result.stats[f"scheduler_{status}"]  += 1

                if transferred:
                    tech = contact_tech(node.radio_type, veh.radio_type, d)
                    contacts_this_step.append(
                        (node.x, node.y, veh.x, veh.y, tech)
                    )

        # ── Vehicle-to-vehicle contact (FILE_CHUNK and FILE_ACK only) ─────
        # COI_SIGHTING messages relay through static nodes, not V2V.
        # V2V requires a shared comms channel; range is determined by that channel.
        for i, va in enumerate(vehicles):
            for vb in vehicles[i + 1:]:
                d   = dist_m(va.x, va.y, vb.x, vb.y)
                eff = compatible_range(va.radio_type, vb.radio_type)
                if eff == 0.0 or d > eff:
                    continue
                any_transferred = False
                for msg in list(va.buffer):
                    if msg.msg_type == "COI_SIGHTING":
                        continue  # sightings relay via static nodes only
                    if msg.msg_id in vb.seen_ids or msg.is_expired(t):
                        continue
                    spray_cap = (config.CHUNK_SPRAY_COPIES
                                 if msg.msg_type == "FILE_CHUNK"
                                 else config.MAX_SPRAY_COPIES)
                    if msg.copies_in_net >= spray_cap:
                        continue
                    if vb.receive([msg], now=t) > 0:
                        msg.copies_in_net += 1
                        msg.hop_count     += 1
                        any_transferred    = True
                if any_transferred:
                    tech = contact_tech(va.radio_type, vb.radio_type, d)
                    contacts_this_step.append(
                        (va.x, va.y, vb.x, vb.y, tech)
                    )

        # ── Vehicle → node relay (COI_SIGHTING store-and-forward) ─────────
        # A vehicle deposits sightings into a static node's buffer when passing,
        # enabling store-and-forward to future vehicles — requires shared channel.
        # The vehicle keeps its copy; it may still reach the destination directly.
        for node in static_nodes:
            for veh in vehicles:
                d   = dist_m(veh.x, veh.y, node.x, node.y)
                eff = compatible_range(node.radio_type, veh.radio_type)
                if eff == 0.0 or d > eff:
                    continue
                relayed_any = False
                for msg in veh.buffer:
                    if (msg.msg_type == "COI_SIGHTING"
                            and msg.msg_id not in node.seen_ids
                            and not msg.is_expired(t)):
                        node.buffer.append(msg)
                        node.seen_ids.add(msg.msg_id)
                        relayed_any = True
                if relayed_any:
                    tech = contact_tech(node.radio_type, veh.radio_type, d)
                    contacts_this_step.append(
                        (node.x, node.y, veh.x, veh.y, tech)
                    )

        # ── Delivery to destination (sightings + chunks) ───────────────────
        if mother_x is not None:
            for veh in vehicles:
                if dist_m(veh.x, veh.y, mother_x, mother_y) > config.WIFI_RANGE:
                    continue
                for msg in veh.buffer:
                    if msg.delivered:
                        continue
                    if msg.msg_type == "COI_SIGHTING" and not msg.is_expired(t):
                        msg.delivered     = True
                        msg.delivery_time = t
                        result.delivered_msgs.append(msg)
                        delay = t - msg.created_at
                        result.stats["total_delay"] += delay
                        if verbose:
                            print(f"  t={t:5.0f}s  {msg.msg_id:03d} SIGHTING delivered  "
                                  f"hops={msg.hop_count}  delay={delay:.0f}s")
                    elif msg.msg_type == "FILE_CHUNK":
                        msg.delivered     = True
                        msg.delivery_time = t
                        delivered_chunk_ids.add(msg.chunk_idx)
                        result.delivered_chunks.append(msg)

                # Vehicles near destination also pick up pending ACK messages
                if dest_pending_acks:
                    veh.receive(dest_pending_acks, now=t)
                    # Don't clear dest_pending_acks — other vehicles should get a copy too
                    # but cap spray on ACKs via copies_in_net (set high so they spread freely)

            # Generate ACK messages when completion thresholds are crossed
            completion = len(delivered_chunk_ids) / config.FILE_CHUNK_COUNT
            for thresh in config.ACK_THRESHOLDS:
                if completion >= thresh and thresh not in acks_sent:
                    acks_sent.add(thresh)
                    from core.utils import xy2ll
                    lat, lon = xy2ll(mother_x, mother_y)
                    ack = Message(
                        msg_id        = ack_id_counter,
                        msg_type      = "FILE_ACK",
                        origin        = "DEST",
                        origin_lat    = lat,
                        origin_lon    = lon,
                        created_at    = t,
                        ttl           = t + config.CHUNK_TTL,
                        benefit       = 0.9,
                        cpu_cost      = 0.01,
                        mem_cost      = 0.01,
                        bw_cost       = 0.1,
                        file_id       = 0,
                        ack_threshold = thresh,
                    )
                    dest_pending_acks.append(ack)
                    ack_id_counter += 1
                    print(f"[file] ACK generated at t={t:.0f}s  "
                          f"completion={completion:.0%}  threshold={thresh:.0%}")

            # Deliver ACK messages to the file source node
            for veh in vehicles:
                eff_ack = compatible_range(file_source.radio_type, veh.radio_type)
                if eff_ack == 0.0 or dist_m(veh.x, veh.y, file_source.x, file_source.y) > eff_ack:
                    continue
                for msg in list(veh.buffer):
                    if msg.msg_type == "FILE_ACK" and not msg.delivered:
                        msg.delivered     = True
                        msg.delivery_time = t
                        file_source.receive_ack(msg)
                        result.stats["acks_delivered_to_source"] += 1

        # ── Connectivity window tracking ───────────────────────────────────
        static_radio_map = {n.node_id: (n.radio_type, n.effective_range)
                            for n in static_nodes}
        all_entities = (
            [(v.vid, v.x, v.y, v.radio_type) for v in vehicles]
            + [(sid, sx, sy, static_radio_map[sid][0])
               for sid, (sx, sy) in static_pos_map.items()]
        )
        n_ent = len(all_entities)
        for i in range(n_ent):
            for j in range(i + 1, n_ent):
                ia, ax, ay, rta = all_entities[i]
                ib, bx, by, rtb = all_entities[j]
                d         = dist_m(ax, ay, bx, by)
                key       = (ia, ib)
                eff_range = compatible_range(rta, rtb)
                rt        = contact_tech(rta, rtb, d)

                if d <= eff_range:
                    if key not in active_windows:
                        active_windows[key] = _open_window(
                            ia, ax, ay, ib, bx, by, d, t, radio_type=rt)
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

    result.stats["total_messages"]       = len(result.all_messages)
    result.stats["delivered"]            = len(result.delivered_msgs)
    result.stats["connectivity_windows"] = len(result.connectivity_log)
    result.stats["chunks_total"]         = config.FILE_CHUNK_COUNT
    result.stats["chunks_delivered"]     = len(delivered_chunk_ids)
    result.stats["chunk_completion_pct"] = round(
        len(delivered_chunk_ids) / config.FILE_CHUNK_COUNT * 100, 1)
    result.stats["file_source_node"]     = file_source.node_id
    result.stats["acks_generated"]       = len(acks_sent)

    return result
