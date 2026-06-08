"""
output/animator.py
MP4 animation builder.

Renders the simulation as a top-down map of Georgetown DC:
  - Road network in dark theme
  - Bluetooth and WiFi range rings around static IoT nodes
  - Destination node labelled with a gold diamond marker
  - Vehicles drawn as directional arrow-cars rotated by heading:
      COI      — large magenta arrow, labelled "COI"
      Escorts  — medium arrows in their assigned colour, labelled by vid
      Traffic  — small grey-green arrows
  - Yellow/purple contact lines flashing at active transfers
  - Live info box showing COI lat/lon and speed
  - Bottom panels: cumulative delivery curve + message status bar chart
"""

import os
import math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.animation as animation
import networkx as nx
from typing import List, Optional, Tuple

from simulation.engine import SimulationResult
import config


# ── Car drawing helper ─────────────────────────────────────────────────────

def _draw_car(
    ax,
    x: float,
    y: float,
    heading: float,
    color: str,
    size: float,
    label: Optional[str] = None,
    zorder: int = 9,
) -> list:
    """
    Draw a FancyArrow pointing in `heading` direction at (x, y).
    Returns list of matplotlib artists added.
    """
    dx = math.cos(heading) * size * 0.45
    dy = math.sin(heading) * size * 0.45
    arrow = mpatches.FancyArrow(
        x - dx, y - dy, 2 * dx, 2 * dy,
        width=size * 0.38,
        head_width=size * 0.65,
        head_length=size * 0.50,
        fc=color,
        ec="white",
        linewidth=0.6,
        length_includes_head=True,
        zorder=zorder,
    )
    ax.add_patch(arrow)
    arts = [arrow]
    if label:
        txt = ax.text(
            x + size * 0.55, y + size * 0.55, label,
            color=color, fontsize=7, fontweight="bold",
            va="bottom", zorder=zorder + 1,
        )
        arts.append(txt)
    return arts


# ── Main builder ───────────────────────────────────────────────────────────

def build_animation(
    G:               nx.Graph,
    result:          SimulationResult,
    static_positions: dict,           # {node_id: (x, y)}
    coi_circuit:     List[str],
    output_path:     str,
    mother_pos:      Optional[Tuple[float, float]] = None,
) -> None:
    """
    Build and save the simulation animation as an MP4.

    Parameters
    ----------
    G                : Georgetown road graph
    result           : SimulationResult from engine.run()
    static_positions : dict mapping node_id -> (x, y)
    coi_circuit      : ordered list of node IDs for COI route ghost
    output_path      : file path for the output MP4
    mother_pos       : (x, y) of the destination/mother node; drawn with label
    """
    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)

    frames = result.frames
    if not frames:
        print("[animator] No frames to render.")
        return

    # Axis limits
    all_x = [G.nodes[n]["x"] for n in G.nodes()]
    all_y = [G.nodes[n]["y"] for n in G.nodes()]
    pad   = 40
    xmin, xmax = min(all_x) - pad, max(all_x) + pad
    ymin, ymax = min(all_y) - pad, max(all_y) + pad

    # ── Figure layout ──────────────────────────────────────────────────────
    fig = plt.figure(figsize=(13, 11), facecolor="#0d1117")
    ax_road  = fig.add_axes([0.04, 0.30, 0.92, 0.65], facecolor="#0d1117")
    ax_stats = fig.add_axes([0.04, 0.04, 0.44, 0.22], facecolor="#161b22")
    ax_bar   = fig.add_axes([0.54, 0.04, 0.42, 0.22], facecolor="#161b22")

    for ax in [ax_road, ax_stats, ax_bar]:
        for sp in ax.spines.values():
            sp.set_edgecolor("#30363d")

    # ── Road network ───────────────────────────────────────────────────────
    ax_road.set_xlim(xmin, xmax); ax_road.set_ylim(ymin, ymax)
    ax_road.set_aspect("equal")
    ax_road.tick_params(colors="#8b949e", labelsize=6)
    ax_road.set_xlabel(
        f"Metres East  (origin: {config.LAT_ORIGIN}°N, "
        f"{abs(config.LON_ORIGIN):.3f}°W)",
        color="#8b949e", fontsize=7
    )
    ax_road.set_ylabel("Metres North", color="#8b949e", fontsize=7)

    for u, v_n in G.edges():
        x1 = G.nodes[u]["x"];  y1 = G.nodes[u]["y"]
        x2 = G.nodes[v_n]["x"]; y2 = G.nodes[v_n]["y"]
        ax_road.plot([x1, x2], [y1, y2], color="#21262d", lw=5, zorder=1,
                     solid_capstyle="round")
        ax_road.plot([x1, x2], [y1, y2], color="#3d4450", lw=2, zorder=2,
                     solid_capstyle="round")

    for nid in G.nodes():
        ax_road.plot(G.nodes[nid]["x"], G.nodes[nid]["y"], ".",
                     color="#3d4450", ms=2.5, zorder=4)

    # COI route ghost
    rx = [G.nodes[n]["x"] for n in coi_circuit if n in G.nodes()]
    ry = [G.nodes[n]["y"] for n in coi_circuit if n in G.nodes()]
    ax_road.plot(rx, ry, color="#f72585", lw=1.0, alpha=0.15, zorder=3,
                 linestyle=":")

    # Static IoT nodes
    for sid, (sx, sy) in static_positions.items():
        ax_road.add_patch(plt.Circle((sx, sy), config.WIFI_RANGE,
                                     color="#7209b7", alpha=0.07, zorder=3))
        ax_road.add_patch(plt.Circle((sx, sy), config.BT_RANGE,
                                     color="#00b4d8", alpha=0.10, zorder=4))
        ax_road.plot(sx, sy, marker="s", color="#00b4d8", ms=9, zorder=6,
                     markeredgecolor="white", markeredgewidth=0.6)
        ax_road.text(sx + 3, sy + 3, sid, color="#58a6ff", fontsize=5.5, zorder=7)

    # Destination / mother node
    if mother_pos is not None:
        mx, my = mother_pos
        ax_road.plot(mx, my, marker="D", color="#ffd700", ms=14, zorder=8,
                     markeredgecolor="white", markeredgewidth=1.0)
        ax_road.text(mx + 4, my + 5, "DEST", color="#ffd700",
                     fontsize=8, fontweight="bold", zorder=9,
                     bbox=dict(boxstyle="round,pad=0.25", facecolor="#161b22",
                               edgecolor="#ffd700", alpha=0.85))

    # ── Legend ─────────────────────────────────────────────────────────────
    legend_el = [
        mpatches.Patch(color="#f72585",  label="Car of Interest (COI)"),
        mpatches.Patch(color="#ff9800",  label="Escort 1 (P/O Street)"),
        mpatches.Patch(color="#00e5ff",  label="Escort 2 (M Street)"),
        mpatches.Patch(color="#ff6b6b",  label="Follower vehicle"),
        mpatches.Patch(color="#4caf50",  label="Traffic vehicle"),
        mpatches.Patch(color="#ffd700",  label="Destination node"),
        mpatches.Patch(color="#00b4d8",  label="IoT static node"),
        mpatches.Patch(color="#00b4d8",  alpha=0.3,
                       label=f"BT range ({config.BT_RANGE}m)"),
        mpatches.Patch(color="#7209b7",  alpha=0.3,
                       label=f"WiFi range ({config.WIFI_RANGE}m)"),
        mpatches.Patch(color="#ffe066",  label="BT contact link"),
        mpatches.Patch(color="#c77dff",  label="WiFi contact link"),
    ]
    ax_road.legend(handles=legend_el, loc="lower right", fontsize=6.5,
                   facecolor="#161b22", labelcolor="#c9d6df",
                   edgecolor="#30363d", framealpha=0.95)

    # ── Static title and info box ──────────────────────────────────────────
    title_obj = ax_road.set_title(
        "Georgetown DTN  |  t=00:00",
        color="#e6edf3", fontsize=11, pad=8
    )
    info_box = ax_road.text(
        xmin + 5, ymax - 8, "", color="#e6edf3", fontsize=7.5, va="top",
        zorder=10,
        bbox=dict(boxstyle="round,pad=0.4", facecolor="#161b22",
                  edgecolor="#30363d", alpha=0.9)
    )

    # ── Stats axes ─────────────────────────────────────────────────────────
    ax_stats.set_xlim(0, config.SIM_DURATION / 60)
    ax_stats.set_ylim(0, max(len(result.delivered_msgs) + 5, 10))
    ax_stats.set_xlabel("Time (min)", color="#9a9abf", fontsize=8)
    ax_stats.set_ylabel("Messages",   color="#9a9abf", fontsize=8)
    ax_stats.set_title("Cumulative Delivery", color="#c9d6df", fontsize=9)
    ax_stats.tick_params(colors="#9a9abf", labelsize=7)
    line_del, = ax_stats.plot([], [], color="#4caf50", lw=2, label="Delivered")
    ax_stats.legend(fontsize=7, facecolor="#161b22", labelcolor="#c9d6df",
                    edgecolor="#30363d", framealpha=0.9)

    bar_objs = ax_bar.bar(
        ["BT\ncontacts", "WiFi\ncontacts", "In-transit", "Delivered"],
        [0, 0, 0, 0],
        color=["#00b4d8", "#c77dff", "#ffe066", "#4caf50"],
        edgecolor="#0d1117"
    )
    ax_bar.set_ylim(0, max(len(result.all_messages) + 5, 10))
    ax_bar.set_ylabel("Count", color="#9a9abf", fontsize=8)
    ax_bar.set_title("Current Status", color="#c9d6df", fontsize=9)
    ax_bar.tick_params(colors="#9a9abf", labelsize=7)

    # ── Per-frame mutable state ────────────────────────────────────────────
    flash_arts: list = []   # contact link lines
    veh_arts:   list = []   # vehicle arrow patches + labels

    delivery_count_by_frame = _precompute_delivery_counts(result)

    def animate(fi: int):
        fr   = frames[fi]
        t    = fr["time"]
        mins = int(t // 60); secs = int(t % 60)

        # ── Remove previous vehicle artists ───────────────────────────────
        for a in veh_arts:
            try: a.remove()
            except: pass
        veh_arts.clear()

        # ── Draw each vehicle as a directional car arrow ───────────────────
        for vs in fr["vehicles"]:
            is_coi    = vs["is_coi"]
            is_escort = vs.get("is_escort", False)
            hdg       = vs.get("heading", 0.0)

            is_follower = vs.get("is_follower", False)
            if is_coi:
                size  = 65
                label = "COI"
                zo    = 11
            elif is_escort:
                size  = 50
                label = vs["vid"]
                zo    = 10
            elif is_follower:
                size  = 45
                label = vs["vid"]
                zo    = 10
            else:
                size  = 35
                label = None
                zo    = 9

            arts = _draw_car(
                ax_road, vs["x"], vs["y"], hdg,
                vs["color"], size, label=label, zorder=zo,
            )
            veh_arts.extend(arts)

        # ── Flash contact links ────────────────────────────────────────────
        for a in flash_arts:
            try: a.remove()
            except: pass
        flash_arts.clear()

        bt_n = wf_n = 0
        for ax_, ay_, bx_, by_, tech in fr["contacts"]:
            col = "#ffe066" if tech == "bluetooth" else "#c77dff"
            lw  = 1.0      if tech == "bluetooth" else 1.6
            ln, = ax_road.plot([ax_, bx_], [ay_, by_],
                               color=col, lw=lw, alpha=0.75, zorder=7)
            flash_arts.append(ln)
            if tech == "bluetooth": bt_n += 1
            else:                    wf_n += 1

        # ── Title and info box ─────────────────────────────────────────────
        title_obj.set_text(
            f"Georgetown DTN  |  t={mins:02d}:{secs:02d}  |  "
            f"BT links: {bt_n}  WiFi links: {wf_n}"
        )
        coi_states = [v for v in fr["vehicles"] if v["is_coi"]]
        if coi_states:
            c = coi_states[0]
            info_box.set_text(
                f"COI\n"
                f"Lat: {c['lat']:.5f}°N\n"
                f"Lon: {abs(c['lon']):.5f}°W\n"
                f"Speed: {c['speed']:.1f} m/s\n"
                f"Buffer: {c['buf_len']} msgs"
            )

        # ── Stats line ─────────────────────────────────────────────────────
        t_mins_hist = [frames[i]["time"] / 60 for i in range(fi + 1)]
        del_hist    = [delivery_count_by_frame[i] for i in range(fi + 1)]
        line_del.set_data(t_mins_hist, del_hist)

        # ── Bar chart ──────────────────────────────────────────────────────
        in_transit = sum(v["buf_len"] for v in fr["vehicles"])
        n_del      = delivery_count_by_frame[fi]
        for bar, h in zip(bar_objs, [bt_n, wf_n, in_transit, n_del]):
            bar.set_height(h)

        return [title_obj, info_box, line_del] + flash_arts + veh_arts + list(bar_objs)

    ani = animation.FuncAnimation(
        fig, animate, frames=len(frames), interval=80, blit=False
    )

    print(f"[animator] Saving {len(frames)} frames → {output_path}")
    ani.save(
        output_path,
        writer=animation.FFMpegWriter(
            fps=config.ANIMATION_FPS,
            bitrate=config.ANIMATION_BITRATE,
        ),
        dpi=config.ANIMATION_DPI,
    )
    plt.close()
    print("[animator] Done.")


def _precompute_delivery_counts(result: SimulationResult) -> list:
    """Build per-frame cumulative delivery count for the stats line."""
    delivery_times = sorted(
        m.delivery_time for m in result.delivered_msgs
        if m.delivery_time >= 0
    )
    counts = []
    di = 0
    for fr in result.frames:
        while di < len(delivery_times) and delivery_times[di] <= fr["time"]:
            di += 1
        counts.append(di)
    return counts
