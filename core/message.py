"""
core/message.py
Message dataclass — the atomic unit of information in the DTN.
"""

from dataclasses import dataclass, field
from typing import Optional
import config


@dataclass
class Message:
    """
    A sighting event or data payload to be carried to a static node.

    Fields
    ------
    msg_id        : unique integer ID
    msg_type      : 'COI_SIGHTING' or 'DATA'
    origin        : vehicle ID or node ID that created the message
    origin_lat    : latitude where message was created
    origin_lon    : longitude where message was created
    created_at    : simulation time of creation (seconds)
    ttl           : absolute expiry time (created_at + MESSAGE_TTL)
    benefit       : MOO-derived priority score (0–1), set by offline NSGA-II
    cpu_cost      : fraction of CPU budget consumed per transfer (0–1)
    mem_cost      : fraction of memory budget consumed per copy (0–1)
    bw_cost       : seconds of contact window consumed per transfer
    hop_count     : number of vehicle hops taken so far
    copies_in_net : how many live copies exist across all carriers
    delivered     : True once received by a static node
    delivery_time : sim time of delivery (-1.0 if not yet delivered)
    """
    msg_id:       int
    msg_type:     str
    origin:       str
    origin_lat:   float
    origin_lon:   float
    created_at:   float
    ttl:          float
    benefit:      float
    cpu_cost:     float
    mem_cost:     float
    bw_cost:      float
    hop_count:    int   = 0
    copies_in_net: int  = 1
    delivered:    bool  = False
    delivery_time: float = -1.0

    # ── Derived properties ─────────────────────────────────────────────────

    def is_expired(self, now: float) -> bool:
        return now >= self.ttl

    def ttl_ratio(self, now: float) -> float:
        """Fraction of TTL remaining (1.0 = fresh, 0.0 = expired)."""
        elapsed = self.ttl - self.created_at
        if elapsed <= 0:
            return 0.0
        return max(0.0, (self.ttl - now) / elapsed)

    def priority_score(self, now: float) -> float:
        """
        Runtime priority: benefit × freshness × 1/(1+hops).
        Used to rank candidates before the LP solve.
        High-benefit, fresh, low-hop messages float to the top.
        """
        hop_penalty = 1.0 / (1.0 + self.hop_count)
        return self.benefit * self.ttl_ratio(now) * hop_penalty

    def age_ratio(self, now: float) -> float:
        """Fraction of lifetime elapsed (inverse of ttl_ratio)."""
        return 1.0 - self.ttl_ratio(now)

    def __repr__(self) -> str:
        return (f"Message(id={self.msg_id}, type={self.msg_type}, "
                f"origin={self.origin}, benefit={self.benefit:.3f}, "
                f"hops={self.hop_count}, delivered={self.delivered})")
