"""
moo/scheduler.py
Online epsilon-constraint LP scheduler.

At every contact event, solves:
    maximize  Σ benefit_i · x_i
    subject to
        Σ cpu_i  · x_i  ≤  eps_cpu
        Σ mem_i  · x_i  ≤  eps_mem
        Σ bw_i   · x_i  ≤  eps_bw
        x_i ∈ [0, 1]   (LP relaxation — rounded down at 0.5)

Falls back to single highest-priority message if LP fails.

Benefits are re-scored using the NSGA-II decode_fn before each solve
so that message age (increasing urgency penalty) is current.
"""

from typing import List, Callable, Tuple
from scipy.optimize import linprog
import numpy as np

from core.message import Message
from core.resource import ResourceSnapshot
import config


def epsilon_constraint_select(
    candidates: List[Message],
    snapshot:   ResourceSnapshot,
    decode_fn:  Callable,
    road_length: float,
    mother_x:   float,
    mother_y:   float,
    origin_x:   float,
    origin_y:   float,
    now:        float,
) -> Tuple[List[Message], str]:
    """
    Select which candidate messages to transfer using epsilon-constraint LP.

    Parameters
    ----------
    candidates   : pre-filtered list of transferable messages
    snapshot     : current resource snapshot (sets epsilon ceilings)
    decode_fn    : NSGA-II decode function for re-scoring
    road_length  : used to normalise distance in benefit re-scoring
    mother_x/y   : XY position of the destination (mother node)
    origin_x/y   : XY position of the scheduling node
    now          : current simulation time

    Returns
    -------
    (selected_messages, status)
    status: 'lp_success' | 'lp_fallback' | 'budget_exhausted' | 'no_candidates'
    """
    if not candidates:
        return [], "no_candidates"

    if snapshot.budget_exhausted():
        return [], "budget_exhausted"

    eps_cpu = snapshot.eps_cpu
    eps_mem = snapshot.eps_mem
    eps_bw  = snapshot.eps_bw

    if eps_cpu <= 0 or eps_mem <= 0 or eps_bw <= 0:
        return [], "budget_exhausted"

    # Re-score each candidate with current age via decode_fn
    import math
    scored = []
    for m in candidates:
        age = m.age_ratio(now)
        dist_to_mother = math.sqrt((origin_x - mother_x)**2 + (origin_y - mother_y)**2)
        dist_norm = min(1.0, dist_to_mother / max(road_length, 1.0))
        # urgency proxy: use stored benefit (reflects original creation-time urgency)
        b, cpu, mem, bw = decode_fn(
            np.clip([m.benefit, dist_norm, age, m.bw_cost / 3.0], 0.0, 1.0)
        )
        scored.append((m, b, cpu, mem, bw))

    n = len(scored)

    # LP: minimize -benefit (scipy minimizes)
    c_lp = [-s[1] for s in scored]
    A_ub = [
        [s[2] for s in scored],   # CPU constraint
        [s[3] for s in scored],   # memory constraint
        [s[4] for s in scored],   # bandwidth constraint
    ]
    b_ub   = [eps_cpu, eps_mem, eps_bw]
    bounds = [(0.0, 1.0)] * n

    result = linprog(c_lp, A_ub=A_ub, b_ub=b_ub, bounds=bounds, method="highs")

    if result.success:
        selected = [scored[i][0] for i, xi in enumerate(result.x) if xi >= 0.5]
        status   = "lp_success"
    else:
        # Fallback: single highest-priority message
        best     = max(scored, key=lambda s: s[1])
        selected = [best[0]]
        status   = "lp_fallback"

    return selected, status
