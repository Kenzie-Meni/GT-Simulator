"""Create a compact connectivity report from schema-v2 simulation JSON."""

import argparse
import json
import os
from collections import Counter

import matplotlib.pyplot as plt
import numpy as np


def analyze(input_path: str, output_dir: str) -> None:
    with open(input_path) as f:
        data = json.load(f)
    if data.get("schema_version") != "2.0":
        raise ValueError("analyze_results.py requires connectivity schema 2.0")

    windows = data["windows"]
    stats = data["statistics"]
    durations = [w["duration_s"] for w in windows]
    contacts = Counter(
        node for w in windows for node in (w["node_a"], w["node_b"])
    )
    tech = Counter(w["technology"] for w in windows)
    starts = [w["start_time_s"] for w in windows]
    os.makedirs(output_dir, exist_ok=True)

    lines = [
        "DTN Connectivity Analysis", "=" * 25,
        f"Schema: {data['schema_version']}",
        f"Nodes: {len(data['nodes'])}",
        f"Contact windows: {len(windows)}",
        f"Bluetooth / WiFi: {tech['bluetooth']} / {tech['wifi']}",
        f"Mean contact duration: {np.mean(durations) if durations else 0:.1f}s",
        f"Median contact duration: {np.median(durations) if durations else 0:.1f}s",
        f"Longest contact: {max(durations, default=0):.1f}s", "",
        "Most connected nodes:",
        *[f"  {node:<12} {count:>5} windows"
          for node, count in contacts.most_common(10)], "",
        f"Sightings delivered: {stats.get('delivered', 0)} / "
        f"{stats.get('total_messages', 0)}",
        f"Chunks delivered: {stats.get('chunks_delivered', 0)} / "
        f"{stats.get('chunks_total', 0)}",
        f"Total transfers: {stats.get('total_transfers', 0)}",
    ]
    report_path = os.path.join(output_dir, "summary.txt")
    with open(report_path, "w") as f:
        f.write("\n".join(lines) + "\n")

    fig, axes = plt.subplots(2, 2, figsize=(11, 7))
    axes[0, 0].hist(durations, bins=25, color="#6c8cff")
    axes[0, 0].set(title="Contact duration", xlabel="Seconds", ylabel="Windows")

    top = contacts.most_common(10)[::-1]
    axes[0, 1].barh([x[0] for x in top], [x[1] for x in top], color="#41b883")
    axes[0, 1].set(title="Most connected nodes", xlabel="Windows")

    axes[1, 0].bar(["Bluetooth", "WiFi"],
                   [tech["bluetooth"], tech["wifi"]],
                   color=["#f4b942", "#9b5de5"])
    axes[1, 0].set(title="Contacts by technology", ylabel="Windows")

    duration = data["simulation"]["duration_s"]
    axes[1, 1].hist(starts, bins=np.arange(0, duration + 61, 60), color="#ef476f")
    axes[1, 1].set(title="Contact starts over time", xlabel="Simulation time (s)",
                   ylabel="Windows")
    fig.tight_layout()
    chart_path = os.path.join(output_dir, "connectivity.png")
    fig.savefig(chart_path, dpi=150)
    plt.close(fig)
    print(f"[analysis] Report: {report_path}")
    print(f"[analysis] Charts: {chart_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", default="data/connectivity.json")
    parser.add_argument("--output-dir", default="data/analysis")
    args = parser.parse_args()
    analyze(args.input, args.output_dir)


if __name__ == "__main__":
    main()
