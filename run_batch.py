"""
run_batch.py
Batch runner — executes N independent simulation runs with different seeds
and collects aggregate statistics across all three message types.

    python run_batch.py              # 100 runs, seeds 0-99
    python run_batch.py --n 50       # 50 runs
    python run_batch.py --n 200 --base-seed 1000

Outputs
-------
    data/batch_results.csv    — per-run row for every metric
    data/batch_summary.txt    — printed aggregate table also saved to disk
"""

import argparse
import csv
import math
import os
import sys
import time

import numpy as np

import config
from network.georgetown import load_graph
from moo.nsga2 import nsga2
from run_simulation import simulate, _print_summary


# ── Aggregate helpers ──────────────────────────────────────────────────────────

_NUMERIC_METRICS = [
    # Sightings
    "sightings_generated", "sightings_delivered", "sighting_delivery_rate",
    "sighting_avg_delay_s", "sighting_min_delay_s", "sighting_max_delay_s",
    "sighting_p50_delay_s", "sighting_avg_hops", "sightings_expired",
    "time_to_first_sighting_s",
    # Chunks
    "chunks_total", "chunks_delivered", "chunk_completion_pct",
    "chunks_per_min", "time_to_first_chunk_s", "chunk_avg_delay_s",
    # ACKs
    "acks_generated", "acks_delivered", "ack_delivery_rate",
    # Network
    "connectivity_windows", "bt_windows", "wifi_windows",
    "avg_window_duration_s", "lp_solves", "lp_fallbacks", "total_transfers",
]

_SECTION_HEADERS = {
    "sightings_generated":        "COI SIGHTINGS",
    "chunks_total":                "FILE CHUNKS",
    "acks_generated":              "FILE ACKs",
    "connectivity_windows":        "NETWORK",
}

_LABELS = {
    "sightings_generated":        "Generated",
    "sightings_delivered":        "Delivered",
    "sighting_delivery_rate":     "Delivery rate",
    "sighting_avg_delay_s":       "Avg delay (s)",
    "sighting_min_delay_s":       "Min delay (s)",
    "sighting_max_delay_s":       "Max delay (s)",
    "sighting_p50_delay_s":       "P50 delay (s)",
    "sighting_avg_hops":          "Avg hops",
    "sightings_expired":          "Expired",
    "time_to_first_sighting_s":   "Time to first (s)",
    "chunks_total":                "Total chunks",
    "chunks_delivered":            "Delivered",
    "chunk_completion_pct":        "Completion (%)",
    "chunks_per_min":              "Rate (chunks/min)",
    "time_to_first_chunk_s":       "Time to first (s)",
    "chunk_avg_delay_s":           "Avg delay (s)",
    "acks_generated":              "Generated",
    "acks_delivered":              "Delivered",
    "ack_delivery_rate":           "Delivery rate",
    "connectivity_windows":        "Total windows",
    "bt_windows":                  "BT windows",
    "wifi_windows":                "WiFi windows",
    "avg_window_duration_s":       "Avg duration (s)",
    "lp_solves":                   "LP solves",
    "lp_fallbacks":                "LP fallbacks",
    "total_transfers":             "Total transfers",
}


def _aggregate(rows: list[dict]) -> dict:
    """Compute mean/std/min/P25/P50/P75/max for each numeric metric."""
    agg = {}
    for key in _NUMERIC_METRICS:
        vals = [r[key] for r in rows
                if isinstance(r.get(key), (int, float)) and not math.isnan(r[key])]
        if not vals:
            agg[key] = {s: float("nan") for s in ("mean", "std", "min", "p25", "p50", "p75", "max")}
            continue
        arr = np.array(vals, dtype=float)
        agg[key] = {
            "mean": float(np.mean(arr)),
            "std":  float(np.std(arr)),
            "min":  float(np.min(arr)),
            "p25":  float(np.percentile(arr, 25)),
            "p50":  float(np.percentile(arr, 50)),
            "p75":  float(np.percentile(arr, 75)),
            "max":  float(np.max(arr)),
            "n":    len(vals),
        }
    return agg


def _fmt_agg(v, pct=False):
    if math.isnan(v):
        return "    N/A"
    if pct:
        return f"{v*100:7.1f}%"
    if abs(v) >= 1000:
        return f"{v:8.0f}"
    if abs(v) >= 10:
        return f"{v:8.1f}"
    return f"{v:8.3f}"


def _print_aggregate(agg: dict, n_runs: int, lines: list = None) -> None:
    """Print the aggregate table. If `lines` is provided, also append to it."""
    W = 100

    def emit(s=""):
        print(s)
        if lines is not None:
            lines.append(s)

    emit("=" * W)
    emit(f"  DTN Batch Results   ({n_runs} runs)")
    emit(f"  {'Metric':<30}  {'mean':>8}  {'std':>8}  {'min':>8}  "
         f"{'P25':>8}  {'P50':>8}  {'P75':>8}  {'max':>8}  {'n':>5}")
    emit("-" * W)

    for key in _NUMERIC_METRICS:
        if key in _SECTION_HEADERS:
            emit(f"\n  [{_SECTION_HEADERS[key]}]")

        a   = agg[key]
        lbl = _LABELS.get(key, key)
        pct = key in ("sighting_delivery_rate", "ack_delivery_rate")

        mean_s = _fmt_agg(a["mean"], pct)
        std_s  = _fmt_agg(a["std"],  pct)
        min_s  = _fmt_agg(a["min"],  pct)
        p25_s  = _fmt_agg(a["p25"],  pct)
        p50_s  = _fmt_agg(a["p50"],  pct)
        p75_s  = _fmt_agg(a["p75"],  pct)
        max_s  = _fmt_agg(a["max"],  pct)
        n_s    = str(a.get("n", "?")).rjust(5)

        emit(f"  {lbl:<30}  {mean_s}  {std_s}  {min_s}  "
             f"{p25_s}  {p50_s}  {p75_s}  {max_s}  {n_s}")

    emit("=" * W)


def _write_csv(rows: list[dict], path: str) -> None:
    if not rows:
        return
    os.makedirs(os.path.dirname(path) if os.path.dirname(path) else ".", exist_ok=True)
    fieldnames = list(rows[0].keys())
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    print(f"[batch] CSV  → {path}")


def _write_summary(lines: list, path: str) -> None:
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"[batch] Summary → {path}")


# ── Main batch runner ──────────────────────────────────────────────────────────

def run_batch(n: int = 100, base_seed: int = 0, verbose_first: bool = False) -> list[dict]:
    """
    Run N simulations with seeds base_seed … base_seed+N-1.
    Loads the road graph and NSGA-II result once and reuses them.
    Returns the list of per-run stats dicts.
    """
    print("=" * 60)
    print(f"DTN Georgetown Batch  ({n} runs, seeds {base_seed}–{base_seed + n - 1})")
    print("=" * 60)

    print("[network] Loading road graph …")
    G = load_graph(try_osmnx=True)
    print("[moo] Running offline NSGA-II …")
    _, decode_fn = nsga2()

    rows      = []
    t_start   = time.time()
    col_w     = 12

    # Header line for progress table
    hdr = (f"  {'Run':>4}  {'Seed':>6}  {'Dest':<10}  "
           f"{'Sight%':>7}  {'Chunks%':>8}  {'ACK%':>6}  "
           f"{'Windows':>8}  {'Elapsed':>8}")
    print(hdr)
    print("  " + "-" * (len(hdr) - 2))

    for i in range(n):
        seed    = base_seed + i
        t_run   = time.time()
        stats   = simulate(
            seed        = seed,
            G           = G,
            decode_fn   = decode_fn,
            animate     = False,
            write_files = False,
            verbose     = (verbose_first and i == 0),
        )
        elapsed = time.time() - t_run
        rows.append(stats)

        sight_pct = stats["sighting_delivery_rate"] * 100
        chunk_pct = stats["chunk_completion_pct"]
        ack_pct   = stats["ack_delivery_rate"] * 100
        dest      = stats["dest_label"][:10]

        print(f"  {i+1:>4}  {seed:>6}  {dest:<10}  "
              f"{sight_pct:>6.1f}%  {chunk_pct:>7.1f}%  {ack_pct:>5.1f}%  "
              f"{stats['connectivity_windows']:>8}  {elapsed:>7.1f}s")
        sys.stdout.flush()

    total_elapsed = time.time() - t_start
    print(f"\n[batch] Completed {n} runs in {total_elapsed:.0f}s "
          f"({total_elapsed/n:.1f}s/run avg)")
    return rows


def main():
    parser = argparse.ArgumentParser(description="DTN batch simulation runner")
    parser.add_argument("--n",         type=int, default=100, help="Number of runs")
    parser.add_argument("--base-seed", type=int, default=0,   help="First seed value")
    parser.add_argument("--csv",       type=str,
                        default=os.path.join(config.OUTPUT_DIR, "batch_results.csv"))
    parser.add_argument("--summary",   type=str,
                        default=os.path.join(config.OUTPUT_DIR, "batch_summary.txt"))
    parser.add_argument("--verbose-first", action="store_true",
                        help="Print per-event delivery log for the first run")
    args = parser.parse_args()

    rows = run_batch(n=args.n, base_seed=args.base_seed,
                     verbose_first=args.verbose_first)

    # ── Aggregate table ────────────────────────────────────────────────────
    agg   = _aggregate(rows)
    lines = []
    print()
    _print_aggregate(agg, n_runs=len(rows), lines=lines)

    # ── Write outputs ──────────────────────────────────────────────────────
    os.makedirs(config.OUTPUT_DIR, exist_ok=True)
    _write_csv(rows, args.csv)
    _write_summary(lines, args.summary)


if __name__ == "__main__":
    main()
