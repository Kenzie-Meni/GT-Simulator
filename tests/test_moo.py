"""
tests/test_moo.py
Unit tests for the MOO layer.
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
from moo.nsga2 import nsga2, decode, dominates, fast_non_dominated_sort
from moo.scheduler import epsilon_constraint_select
from core.message import Message
from core.resource import ResourceSnapshot


def test_decode_ranges():
    """decode() should always return values in expected ranges."""
    for _ in range(100):
        x = np.random.rand(4)
        b, c, m, w = decode(x)
        assert 0.0 < b <= 1.0,   f"benefit out of range: {b}"
        assert 0.05 <= c <= 0.20, f"cpu out of range: {c}"
        assert 0.02 <= m <= 0.10, f"mem out of range: {m}"
        assert 0.5  <= w <= 3.0,  f"bw out of range: {w}"
    print("PASS test_decode_ranges")


def test_dominates():
    a = [1, 2, 3]
    b = [2, 3, 4]
    assert dominates(a, b)
    assert not dominates(b, a)
    assert not dominates(a, a)
    print("PASS test_dominates")


def test_pareto_front_nonempty():
    pareto, _ = nsga2(n_gen=5, pop_size=20, seed=0)
    assert len(pareto) > 0
    print(f"PASS test_pareto_front_nonempty  (front size={len(pareto)})")


def _make_message(msg_id, benefit, cpu, mem, bw, now=0.0):
    import config
    return Message(
        msg_id=msg_id, msg_type="TEST", origin="node",
        origin_lat=38.9, origin_lon=-77.06,
        created_at=now, ttl=now + config.MESSAGE_TTL,
        benefit=benefit, cpu_cost=cpu, mem_cost=mem, bw_cost=bw,
    )


def test_lp_selects_best():
    """LP should select the high-benefit message when resources are tight."""
    from moo.nsga2 import decode as decode_fn
    msgs = [
        _make_message(0, 0.9, 0.08, 0.04, 1.0),  # high benefit, medium cost
        _make_message(1, 0.2, 0.08, 0.04, 1.0),  # low benefit, same cost
    ]
    snap = ResourceSnapshot(avail_cpu=0.5, avail_mem=0.4, contact_window=3.0)
    selected, status = epsilon_constraint_select(
        msgs, snap, decode_fn,
        road_length=800, mother_x=0, mother_y=0,
        origin_x=100, origin_y=100, now=0.0,
    )
    assert len(selected) >= 1
    assert selected[0].msg_id == 0, "LP should prefer high-benefit message"
    print(f"PASS test_lp_selects_best  (status={status})")


def test_lp_empty_candidates():
    from moo.nsga2 import decode as decode_fn
    snap = ResourceSnapshot(avail_cpu=0.5, avail_mem=0.4, contact_window=3.0)
    selected, status = epsilon_constraint_select(
        [], snap, decode_fn,
        road_length=800, mother_x=0, mother_y=0,
        origin_x=0, origin_y=0, now=0.0,
    )
    assert selected == []
    assert status == "no_candidates"
    print(f"PASS test_lp_empty_candidates  (status={status})")


def test_budget_exhausted():
    from moo.nsga2 import decode as decode_fn
    msgs = [_make_message(0, 0.9, 0.08, 0.04, 1.0)]
    snap = ResourceSnapshot(avail_cpu=0.1, avail_mem=0.1, contact_window=0.0)
    selected, status = epsilon_constraint_select(
        msgs, snap, decode_fn,
        road_length=800, mother_x=0, mother_y=0,
        origin_x=0, origin_y=0, now=0.0,
    )
    assert selected == []
    assert status == "budget_exhausted"
    print(f"PASS test_budget_exhausted  (status={status})")


if __name__ == "__main__":
    test_decode_ranges()
    test_dominates()
    test_pareto_front_nonempty()
    test_lp_selects_best()
    test_lp_empty_candidates()
    test_budget_exhausted()
    print("\nAll MOO tests passed.")
