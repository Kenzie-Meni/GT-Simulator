"""
core/resource.py
ResourceSnapshot — dynamic device resource state sampled at contact time.
"""

from dataclasses import dataclass
import config


@dataclass
class ResourceSnapshot:
    """
    Available resources on a node at the moment of a contact event.

    avail_cpu     : free CPU fraction right now (blended rolling avg)
    avail_mem     : free memory fraction right now
    contact_window: estimated seconds remaining in vehicle contact
    """
    avail_cpu:      float
    avail_mem:      float
    contact_window: float

    @property
    def eps_cpu(self) -> float:
        """Schedulable CPU headroom after reserving the floor."""
        return max(0.0, self.avail_cpu - config.CPU_RESERVE_FLOOR)

    @property
    def eps_mem(self) -> float:
        """Schedulable memory headroom after reserving the floor."""
        return max(0.0, self.avail_mem - config.MEM_RESERVE_FLOOR)

    @property
    def eps_bw(self) -> float:
        """Schedulable bandwidth (BW_FRACTION of contact window)."""
        return max(0.0, self.contact_window * config.BW_FRACTION)

    def budget_exhausted(self) -> bool:
        """True if any resource is below its reserve floor."""
        return (
            self.avail_cpu    < config.CPU_RESERVE_FLOOR
            or self.avail_mem < config.MEM_RESERVE_FLOOR
            or self.contact_window <= 0.0
        )

    def deduct(self, cpu: float, mem: float, bw: float) -> None:
        """Deduct resource costs after scheduling a message."""
        self.avail_cpu      -= cpu
        self.avail_mem      -= mem
        self.contact_window -= bw
