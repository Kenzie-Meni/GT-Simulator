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

        candidates = [
            m for m in self.buffer
            if not m.is_expired(now)
            and m.msg_id not in vehicle.seen_ids
            and m.copies_in_net < config.MAX_SPRAY_COPIES
            and len(vehicle.buffer) < config.BUFFER_CAPACITY
        ]

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
