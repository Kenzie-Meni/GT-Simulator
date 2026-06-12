"""
core/message.py
Message dataclass — the atomic unit of information in the DTN.

Three message types:
  COI_SIGHTING  — observes and records the Car of Interest; high priority, time-sensitive
  FILE_CHUNK    — one fragment of a large file transfer; low individual priority, no TTL urgency
  FILE_ACK      — completion acknowledgement traveling back from destination to file source
"""

from dataclasses import dataclass, field
from typing import Optional
import config


@dataclass
class Message:
    """
    An atomic DTN message carried by vehicles across the network.

    Fields
    ------
    msg_id        : unique integer ID
    msg_type      : 'COI_SIGHTING' | 'FILE_CHUNK' | 'FILE_ACK'
    origin        : vehicle ID or node ID that created the message
    origin_lat    : latitude where message was created
    origin_lon    : longitude where message was created
    created_at    : simulation time of creation (seconds)
    ttl           : absolute expiry time (float('inf') for chunks/acks)
    benefit       : MOO-derived priority score (0–1), set by offline NSGA-II
    cpu_cost      : fraction of CPU budget consumed per transfer (0–1)
    mem_cost      : fraction of memory budget consumed per copy (0–1)
    bw_cost       : seconds of contact window consumed per transfer
    hop_count     : number of vehicle hops taken so far
    copies_in_net : how many live copies exist across all carriers
    delivered     : True once received by destination
    delivery_time : sim time of delivery (-1.0 if not yet delivered)
    file_id       : which file this chunk/ack belongs to (-1 for sightings)
    chunk_idx     : chunk index 0..total_chunks-1 (-1 for non-chunk messages)
    total_chunks  : total chunks in the file (-1 for non-chunk messages)
    ack_threshold : for FILE_ACK, the completion fraction that triggered this ACK
    """
    msg_id:        int
    msg_type:      str
    origin:        str
    origin_lat:    float
    origin_lon:    float
    created_at:    float
    ttl:           float
    benefit:       float
    cpu_cost:      float
    mem_cost:      float
    bw_cost:       float
    hop_count:     int   = 0
    copies_in_net: int   = 1
    delivered:     bool  = False
    delivery_time: float = -1.0
    # File transfer fields
    file_id:       int   = -1
    chunk_idx:     int   = -1
    total_chunks:  int   = -1
    ack_threshold: float = -1.0

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
        Runtime priority used for buffer eviction and candidate ranking.

        COI_SIGHTING : benefit × freshness × 1/(1+hops)  — high, decays with age
        FILE_ACK     : 0.85 / (1 + 0.1×hops)             — high, needs to return fast
        FILE_CHUNK   : CHUNK_BASE_BENEFIT / (1 + 0.3×hops) — low flat value
        """
        if self.msg_type == "FILE_ACK":
            return 0.85 / (1.0 + self.hop_count * 0.1)
        if self.msg_type == "FILE_CHUNK":
            return config.CHUNK_BASE_BENEFIT / (1.0 + self.hop_count * 0.3)
        # COI_SIGHTING
        hop_penalty = 1.0 / (1.0 + self.hop_count)
        return self.benefit * self.ttl_ratio(now) * hop_penalty

    def age_ratio(self, now: float) -> float:
        """Fraction of lifetime elapsed (inverse of ttl_ratio)."""
        return 1.0 - self.ttl_ratio(now)

    def __repr__(self) -> str:
        extra = f", chunk={self.chunk_idx}" if self.chunk_idx >= 0 else ""
        return (f"Message(id={self.msg_id}, type={self.msg_type}, "
                f"origin={self.origin}, benefit={self.benefit:.3f}, "
                f"hops={self.hop_count}, delivered={self.delivered}{extra})")
