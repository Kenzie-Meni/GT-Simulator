"""
simulation/static_node.py
IoT static sensor node.

Responsibilities:
  - Generate COI sighting messages with MOO-derived benefit scores
  - Run the epsilon-constraint LP scheduler at each contact event
  - Expire stale messages from buffer
  - Track dynamic CPU/memory load via rolling average
"""

import math
import random
from dataclasses import dataclass, field
from collections import deque
from typing import List, Set, Callable, Tuple
import numpy as np

from core.message import Message
from core.resource import ResourceSnapshot
from core.utils import xy2ll, dist_m
from moo.scheduler import epsilon_constraint_select
import config


@dataclass
class StaticNode:
    """
    Roadside IoT sensor node.

    Parameters
    ----------
    node_id    : matches a graph node ID (e.g. 'WIS_N')
    x, y       : position in local metres
    mother_x/y : XY position of the mother/destination node
    decode_fn  : NSGA-II decode function for benefit assignment
    road_length: used to normalise distance-to-mother in benefit scoring

    File transfer fields (populated only on the randomly chosen source node)
    --------------------------------------------------------------------------
    is_file_source  : True for the one node holding the large file
    file_store      : all FILE_CHUNK messages; not subject to BUFFER_CAPACITY
    dispatched_ids  : chunk msg_ids already handed to at least one vehicle
    acked_chunk_ids : chunk indices confirmed delivered (via FILE_ACK feedback)
    """
    node_id:     str
    x:           float
    y:           float
    mother_x:    float
    mother_y:    float
    decode_fn:   Callable
    road_length: float

    buffer:   List[Message] = field(default_factory=list)
    seen_ids: Set[int]      = field(default_factory=set)
    _cpu_history: deque     = field(default_factory=lambda: deque(maxlen=5))

    # Radio capability — assigned randomly at startup (1/3 each)
    # "bt"   : Bluetooth only  (range = BT_RANGE)
    # "wifi" : WiFi only       (range = WIFI_RANGE)
    # "both" : both radios     (range = WIFI_RANGE, classifies by distance)
    radio_type: str = "both"

    # File source fields
    is_file_source:  bool      = False
    file_store:      List[Message] = field(default_factory=list)
    dispatched_ids:  Set[int]      = field(default_factory=set)
    acked_chunk_ids: Set[int]      = field(default_factory=set)

    @property
    def effective_range(self) -> float:
        """Maximum contact range given this node's radio type."""
        return config.BT_RANGE if self.radio_type == "bt" else config.WIFI_RANGE

    # ── Properties ─────────────────────────────────────────────────────────

    @property
    def lat(self) -> float:
        return xy2ll(self.x, self.y)[0]

    @property
    def lon(self) -> float:
        return xy2ll(self.x, self.y)[1]

    @property
    def dist_to_mother(self) -> float:
        return dist_m(self.x, self.y, self.mother_x, self.mother_y)

    # ── File store init ────────────────────────────────────────────────────

    def init_file_store(self, start_id: int, now: float) -> int:
        """
        Populate file_store with FILE_CHUNK_COUNT chunk messages.
        Returns the next available msg_id after the last chunk.
        Chunks use CHUNK_BASE_BENEFIT and a long TTL — they are not time-sensitive.
        """
        from core.utils import xy2ll
        lat, lon = xy2ll(self.x, self.y)
        for i in range(config.FILE_CHUNK_COUNT):
            msg = Message(
                msg_id       = start_id + i,
                msg_type     = "FILE_CHUNK",
                origin       = self.node_id,
                origin_lat   = lat,
                origin_lon   = lon,
                created_at   = now,
                ttl          = now + config.CHUNK_TTL,
                benefit      = config.CHUNK_BASE_BENEFIT,
                cpu_cost     = 0.02,
                mem_cost     = 0.01,
                bw_cost      = 0.5,
                file_id      = 0,
                chunk_idx    = i,
                total_chunks = config.FILE_CHUNK_COUNT,
            )
            self.file_store.append(msg)
        self.is_file_source = True
        print(f"[file] Source node: {self.node_id}  "
              f"({config.FILE_CHUNK_COUNT} chunks seeded, ids {start_id}–{start_id + config.FILE_CHUNK_COUNT - 1})")
        return start_id + config.FILE_CHUNK_COUNT

    # ── ACK reception ──────────────────────────────────────────────────────

    def receive_ack(self, ack_msg) -> None:
        """Record chunk indices confirmed delivered via a FILE_ACK message."""
        # ack_threshold encodes how many chunks are done as a fraction
        n_done = int(ack_msg.ack_threshold * config.FILE_CHUNK_COUNT)
        for i in range(n_done):
            self.acked_chunk_ids.add(i)

    # ── Message generation ─────────────────────────────────────────────────

    def generate_sighting(self, msg_id: int, now: float,
                          coi_lat: float, coi_lon: float) -> Message:
        """
        Create a COI sighting message.
        Benefit is computed from the NSGA-II decode function using:
          - urgency: random (0.5–1.0 — sightings are high priority)
          - dist_norm: this node's distance to mother node
          - age_ratio: 0.0 (brand new)
          - cost_level: random (0.1–0.9)
        """
        urgency    = random.uniform(0.5, 1.0)
        cost_level = random.uniform(0.1, 0.9)
        dist_norm  = min(1.0, self.dist_to_mother / max(self.road_length, 1.0))

        benefit, cpu, mem, bw = self.decode_fn(
            np.clip([urgency, dist_norm, 0.0, cost_level], 0.0, 1.0)
        )

        msg = Message(
            msg_id     = msg_id,
            msg_type   = "COI_SIGHTING",
            origin     = self.node_id,
            origin_lat = coi_lat,
            origin_lon = coi_lon,
            created_at = now,
            ttl        = now + config.MESSAGE_TTL,
            benefit    = benefit,
            cpu_cost   = cpu,
            mem_cost   = mem,
            bw_cost    = bw,
        )
        self.buffer.append(msg)
        self.seen_ids.add(msg_id)
        return msg

    # ── Resource sampling ──────────────────────────────────────────────────

    def _sample_resources(self, now: float) -> ResourceSnapshot:
        """
        Sample dynamic CPU/memory availability.
        Simulates a background workload on the IoT device with sinusoidal
        variation + Gaussian noise, blended as a rolling average.
        """
        bg_cpu = float(np.clip(
            0.3 + 0.2 * math.sin(now / 120.0) + random.gauss(0, 0.05),
            0.0, 0.9
        ))
        self._cpu_history.append(bg_cpu)
        blended_cpu = 0.7 * float(np.mean(self._cpu_history)) + 0.3 * bg_cpu
        avail_cpu = max(0.0, 1.0 - blended_cpu)
        avail_mem = max(0.0, 1.0 - min(0.8, len(self.buffer) * 0.04))
        return ResourceSnapshot(avail_cpu=avail_cpu, avail_mem=avail_mem,
                                contact_window=0.0)

    # ── Scheduler ──────────────────────────────────────────────────────────

    def schedule_transfer(
        self,
        vehicle,          # Vehicle instance
        contact_window: float,
        now: float,
    ) -> Tuple[List[Message], str]:
        """
        Run the epsilon-constraint LP to select messages for transfer.

        Steps:
        1. Sample current resources → set epsilon ceilings
        2. Filter candidates: non-expired, unseen by vehicle, below spray cap,
           vehicle has buffer space
        3. Call LP scheduler → get selected messages + status
        4. Update hop counts and copy counts

        Returns (transferred_messages, status_string)
        """
        snapshot = self._sample_resources(now)
        snapshot.contact_window = contact_window

        # Sighting candidates — no buffer-space gate; receive() handles eviction
        candidates = [
            m for m in self.buffer
            if not m.is_expired(now)
            and m.msg_id not in vehicle.seen_ids
            and m.copies_in_net < config.MAX_SPRAY_COPIES
        ]

        # Chunk candidates from file_store (source node only)
        # Only offer chunks when there is actual free space — sightings take priority
        if self.is_file_source and len(vehicle.buffer) < config.BUFFER_CAPACITY:
            # Prioritise un-dispatched chunks; fall back to dispatched-but-not-acked
            undispatched = [
                m for m in self.file_store
                if m.msg_id not in self.dispatched_ids
                and m.msg_id not in vehicle.seen_ids
                and m.copies_in_net < config.CHUNK_SPRAY_COPIES
            ]
            redispatch = [
                m for m in self.file_store
                if m.msg_id in self.dispatched_ids
                and m.chunk_idx not in self.acked_chunk_ids
                and m.msg_id not in vehicle.seen_ids
                and m.copies_in_net < config.CHUNK_SPRAY_COPIES
            ]
            chunk_pool = (undispatched or redispatch)
            # Cap how many chunks we present to the LP so sightings aren't buried
            candidates += chunk_pool[:20]

        selected, status = epsilon_constraint_select(
            candidates   = candidates,
            snapshot     = snapshot,
            decode_fn    = self.decode_fn,
            road_length  = self.road_length,
            mother_x     = self.mother_x,
            mother_y     = self.mother_y,
            origin_x     = self.x,
            origin_y     = self.y,
            now          = now,
        )

        for msg in selected:
            msg.copies_in_net += 1
            msg.hop_count     += 1
            if msg.msg_type == "FILE_CHUNK":
                self.dispatched_ids.add(msg.msg_id)

        return selected, status

    # ── Maintenance ────────────────────────────────────────────────────────

    def expire_messages(self, now: float) -> int:
        """Remove expired messages from buffer. Returns count removed."""
        before = len(self.buffer)
        self.buffer = [m for m in self.buffer if not m.is_expired(now)]
        return before - len(self.buffer)

    def __repr__(self) -> str:
        return (f"StaticNode(id={self.node_id}, "
                f"pos=({self.x:.1f},{self.y:.1f}), buf={len(self.buffer)})")
