"""
simulation/vehicle.py
Kinematic vehicle model with full physics.

State: position (x,y), speed, heading, acceleration.
Physics: friction-limited accel/brake, turn-proportional slowdown.
Routing: waypoint list from the road graph; advances on proximity.
"""

import math
import random
from dataclasses import dataclass, field
from typing import List, Set, Optional
import networkx as nx

from core.message import Message
from core.utils import xy2ll
import config


@dataclass
class Vehicle:
    """
    Kinematic vehicle following a waypoint route on the road graph.

    Parameters
    ----------
    vid      : unique string identifier ('COI', 'V00', ...)
    x, y     : initial position in local metres
    route    : ordered list of graph node IDs
    speed    : initial speed (m/s)
    heading  : initial heading (radians; 0 = east)
    is_coi   : True for the Car of Interest
    color    : matplotlib color string for animation
    """
    vid:     str
    x:       float
    y:       float
    route:   List[str]
    speed:   float = 5.0
    heading: float = 0.0
    accel:   float = 0.0
    ridx:    int   = 0       # current waypoint index
    is_coi:          bool = False
    is_escort:       bool = False
    dynamic_routing: bool = False   # pick next road stochastically at intersections
    is_follower:     bool = False   # bias turns toward COI position
    color:           str  = "#4caf50"
    lat:     float = 0.0
    lon:     float = 0.0
    # DTN message buffer
    buffer:   List[Message] = field(default_factory=list)
    seen_ids: Set[int]      = field(default_factory=set)

    # ── Route helpers ──────────────────────────────────────────────────────

    def current_target(self) -> str:
        return self.route[self.ridx % len(self.route)]

    def advance_waypoint(self, G=None, follow_pos=None) -> None:
        """
        Move to the next waypoint.

        For dynamic_routing vehicles: extend the route by picking the next
        intersection probabilistically.  Followers bias that pick toward the
        COI position supplied in follow_pos.

        For fixed-route vehicles: wrap ridx as before.
        """
        if self.dynamic_routing and G is not None:
            self.ridx += 1
            if self.ridx >= len(self.route):
                nxt = self._pick_next_dynamic(G, follow_pos)
                self.route.append(nxt)
            # Trim to stop unbounded route growth
            if self.ridx > 100:
                trim       = self.ridx - 20
                self.route = self.route[trim:]
                self.ridx  = 20
        else:
            self.ridx = (self.ridx + 1) % len(self.route)

    def _pick_next_dynamic(self, G: nx.Graph, follow_pos=None) -> str:
        """
        Choose the next waypoint from the neighbors of the current node.

        - Avoids immediate reversal (U-turn).
        - Followers weight neighbors by inverse distance to follow_pos (COI).
        - Plain dynamic vehicles weight by node degree (prefer main roads).
        """
        current   = self.route[self.ridx - 1]          # node just reached
        prev      = self.route[self.ridx - 2] if self.ridx >= 2 else None

        neighbors = [n for n in G.neighbors(current) if G.degree(n) > 1]
        if not neighbors:
            neighbors = list(G.neighbors(current))
        if not neighbors:
            return current

        forward = [n for n in neighbors if n != prev] or neighbors

        if self.is_follower and follow_pos is not None:
            fx, fy = follow_pos
            scores = []
            for n in forward:
                d = math.sqrt((G.nodes[n]["x"] - fx) ** 2 +
                              (G.nodes[n]["y"] - fy) ** 2) + 1.0
                scores.append(1.0 / d)
        else:
            scores = [float(G.degree(n)) for n in forward]

        return random.choices(forward, weights=scores)[0]

    # ── Physics step ───────────────────────────────────────────────────────

    def step(self, G: nx.Graph, dt: float = None,
             follow_pos: tuple = None) -> None:
        """
        Advance vehicle physics by one timestep.

        1. Compute vector to current target waypoint.
        2. Determine desired heading and turn error.
        3. Scale target speed down for sharp turns (friction-limited cornering).
        4. Apply friction-limited acceleration toward target speed.
        5. Update heading smoothly.
        6. Integrate position.
        7. Advance waypoint if within WAYPOINT_RADIUS.
        """
        dt = dt or config.DT

        target_node = self.current_target()
        tx = G.nodes[target_node]["x"]
        ty = G.nodes[target_node]["y"]

        dx = tx - self.x
        dy = ty - self.y
        dist = math.sqrt(dx ** 2 + dy ** 2)

        # Advance waypoint if close enough
        if dist < config.WAYPOINT_RADIUS:
            self.advance_waypoint(G, follow_pos)
            target_node = self.current_target()
            tx = G.nodes[target_node]["x"]
            ty = G.nodes[target_node]["y"]
            dx = tx - self.x
            dy = ty - self.y
            dist = math.sqrt(dx ** 2 + dy ** 2) + 1e-6

        desired_heading = math.atan2(dy, dx)

        # Heading error in [-π, π]
        hdiff = desired_heading - self.heading
        while hdiff >  math.pi: hdiff -= 2 * math.pi
        while hdiff < -math.pi: hdiff += 2 * math.pi

        # Target speed: reduce for sharp turns
        turn_factor  = max(config.TURN_SLOWDOWN,
                           1.0 - abs(hdiff) / math.pi * (1.0 - config.TURN_SLOWDOWN))
        target_speed = config.MAX_SPEED * turn_factor

        # Friction-limited acceleration
        max_a = min(config.MAX_ACCEL, config.MU_FRICTION * 9.81)
        if self.speed < target_speed:
            self.accel = min(max_a, (target_speed - self.speed) / dt)
        else:
            self.accel = max(-config.MAX_BRAKE, (target_speed - self.speed) / dt)

        self.speed = float(
            max(config.MIN_SPEED, min(config.MAX_SPEED, self.speed + self.accel * dt))
        )

        # Smooth heading update (rate-limited turn)
        turn_rate    = min(abs(hdiff), 1.2 * dt) * (1 if hdiff >= 0 else -1)
        self.heading += turn_rate

        # Position integration
        self.x += self.speed * math.cos(self.heading) * dt
        self.y += self.speed * math.sin(self.heading) * dt

        # Keep lat/lon current
        self.lat, self.lon = xy2ll(self.x, self.y)

    # ── DTN buffer helpers ─────────────────────────────────────────────────

    def receive(self, messages: List[Message]) -> int:
        """
        Accept messages not already in buffer (dedup by msg_id).
        Returns number of new messages accepted.
        """
        added = 0
        for msg in messages:
            if (msg.msg_id not in self.seen_ids
                    and len(self.buffer) < config.BUFFER_CAPACITY):
                self.buffer.append(msg)
                self.seen_ids.add(msg.msg_id)
                added += 1
        return added

    def deliver_to_node(self, node_x: float, node_y: float,
                        now: float) -> List[Message]:
        """
        If within WIFI_RANGE of a node, mark all buffered messages as delivered.
        Returns list of newly delivered messages.
        """
        from core.utils import dist_m
        if dist_m(self.x, self.y, node_x, node_y) > config.WIFI_RANGE:
            return []
        delivered = []
        for msg in self.buffer:
            if not msg.delivered and not msg.is_expired(now):
                msg.delivered     = True
                msg.delivery_time = now
                delivered.append(msg)
        return delivered

    # ── Convenience ───────────────────────────────────────────────────────

    def state_dict(self) -> dict:
        """Snapshot of vehicle state for animation frame recording."""
        return {
            "vid":       self.vid,
            "x":         self.x,
            "y":         self.y,
            "lat":       self.lat,
            "lon":       self.lon,
            "speed":     self.speed,
            "heading":   self.heading,
            "is_coi":      self.is_coi,
            "is_escort":   self.is_escort,
            "is_follower": self.is_follower,
            "color":     self.color,
            "buf_len":   len(self.buffer),
        }

    def __repr__(self) -> str:
        return (f"Vehicle(vid={self.vid}, pos=({self.x:.1f},{self.y:.1f}), "
                f"speed={self.speed:.1f}m/s, buf={len(self.buffer)})")
