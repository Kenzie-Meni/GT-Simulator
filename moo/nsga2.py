"""
moo/nsga2.py
Offline NSGA-II Pareto optimizer.

Runs once at simulation startup. Explores the 4-objective space:
  f1 = -benefit   (minimize → maximize benefit)
  f2 =  cpu_cost
  f3 =  mem_cost
  f4 =  bw_cost

Decision variables (all in [0, 1]):
  urgency     — how time-sensitive the message is
  dist_norm   — origin-to-destination distance, normalized
  age_ratio   — fraction of TTL elapsed
  cost_level  — transmission cost level

Returns a decode_fn(x) that maps a 4-vector to (benefit, cpu, mem, bw).
This function is used at message creation time and at each LP re-score.
"""

import numpy as np
from typing import Callable
import config


# ── Objective helpers ──────────────────────────────────────────────────────

def decode(x: np.ndarray) -> tuple[float, float, float, float]:
    """
    Map decision vector x = [urgency, dist_norm, age_ratio, cost_level]
    to (benefit, cpu_cost, mem_cost, bw_cost).

    Benefit model:
      - High urgency → higher benefit
      - Far from destination → slight benefit reduction (message less likely useful)
      - Old message (high age_ratio) → benefit degrades
    """
    urgency, dist_norm, age_ratio, cost_level = np.clip(x, 0.0, 1.0)
    benefit = urgency * (1.0 - dist_norm * 0.5) * (1.0 - age_ratio * 0.3)
    benefit = float(np.clip(benefit, 0.01, 1.0))
    cpu  = float(0.05 + cost_level * 0.15)
    mem  = float(0.02 + cost_level * 0.08)
    bw   = float(0.50 + cost_level * 2.50)
    return benefit, cpu, mem, bw


def objectives(x: np.ndarray) -> list[float]:
    """All four objectives (minimizing all — benefit is negated)."""
    b, c, m, w = decode(x)
    return [-b, c, m, w]


# ── NSGA-II core ───────────────────────────────────────────────────────────

def dominates(a: list, b: list) -> bool:
    """True if solution a Pareto-dominates b."""
    return (all(ai <= bi for ai, bi in zip(a, b))
            and any(ai < bi for ai, bi in zip(a, b)))


def fast_non_dominated_sort(pop_obj: list) -> list[list[int]]:
    """Return Pareto fronts as lists of indices into pop_obj."""
    n = len(pop_obj)
    dominated_by    = [[] for _ in range(n)]
    domination_count = [0] * n
    fronts = [[]]

    for i in range(n):
        for j in range(n):
            if i == j:
                continue
            if dominates(pop_obj[i], pop_obj[j]):
                dominated_by[i].append(j)
            elif dominates(pop_obj[j], pop_obj[i]):
                domination_count[i] += 1
        if domination_count[i] == 0:
            fronts[0].append(i)

    k = 0
    while fronts[k]:
        next_front = []
        for i in fronts[k]:
            for j in dominated_by[i]:
                domination_count[j] -= 1
                if domination_count[j] == 0:
                    next_front.append(j)
        k += 1
        fronts.append(next_front)

    return fronts[:-1]


def crowding_distance(front_indices: list, pop_obj: list) -> list[float]:
    """Compute crowding distance for solutions in a front."""
    n_f = len(front_indices)
    if n_f <= 2:
        return [float("inf")] * n_f

    n_obj = len(pop_obj[0])
    dist  = [0.0] * n_f

    for m in range(n_obj):
        vals = [(pop_obj[front_indices[i]][m], i) for i in range(n_f)]
        vals.sort()
        dist[vals[0][1]]  = float("inf")
        dist[vals[-1][1]] = float("inf")
        obj_range = vals[-1][0] - vals[0][0]
        if obj_range == 0:
            continue
        for k in range(1, n_f - 1):
            dist[vals[k][1]] += (vals[k + 1][0] - vals[k - 1][0]) / obj_range

    return dist


def nsga2(
    n_gen: int = None,
    pop_size: int = None,
    seed: int = None,
) -> tuple[list, Callable]:
    """
    Run offline NSGA-II and return (pareto_front, decode_fn).

    pareto_front : list of (x_vector, objectives_vector) for front-0 solutions
    decode_fn    : function (x) -> (benefit, cpu, mem, bw)
                   used by SensorNode.generate_message and the LP re-scorer
    """
    n_gen    = n_gen    or config.NSGA2_GENERATIONS
    pop_size = pop_size or config.NSGA2_POP_SIZE
    seed     = seed     or config.SEED

    rng = np.random.RandomState(seed)
    pop     = rng.rand(pop_size, 4)
    pop_obj = [objectives(p) for p in pop]

    for _ in range(n_gen):
        # Generate offspring via tournament + Gaussian mutation
        offspring = []
        fronts = fast_non_dominated_sort(pop_obj)
        rank   = {idx: fi for fi, front in enumerate(fronts) for idx in front}

        for _ in range(pop_size):
            i1, i2 = rng.randint(0, pop_size, 2)
            parent  = pop[i1] if rank.get(i1, 99) <= rank.get(i2, 99) else pop[i2]
            child   = np.clip(parent + rng.randn(4) * 0.1, 0.0, 1.0)
            offspring.append(child)

        offspring_obj = [objectives(c) for c in offspring]

        # Combine and select next generation
        combined     = list(pop) + offspring
        combined_obj = pop_obj + offspring_obj
        fronts       = fast_non_dominated_sort(combined_obj)

        new_pop = []; new_obj = []
        for front in fronts:
            if len(new_pop) + len(front) <= pop_size:
                for idx in front:
                    new_pop.append(combined[idx])
                    new_obj.append(combined_obj[idx])
            else:
                needed = pop_size - len(new_pop)
                cd     = crowding_distance(front, combined_obj)
                for _, idx in sorted(zip(cd, front), reverse=True)[:needed]:
                    new_pop.append(combined[idx])
                    new_obj.append(combined_obj[idx])
                break

        pop = new_pop; pop_obj = new_obj

    fronts       = fast_non_dominated_sort(pop_obj)
    pareto_front = [(pop[i], pop_obj[i]) for i in fronts[0]]

    return pareto_front, decode
