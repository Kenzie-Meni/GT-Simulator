"""
output/animator.py
MP4 animation builder.

Renders the simulation as a top-down map of Georgetown DC:
  - Road network in dark theme with street name labels
  - Bluetooth and WiFi range rings around static IoT nodes
  - Destination node labelled with a gold diamond marker
  - Vehicles drawn as directional arrow-cars rotated by heading:
      COI      — large magenta arrow, labelled "COI"
      Escorts  — medium arrows in escort/follower color set, labelled by vid
      Followers— same color palette as escorts (same surveillance team)
      Traffic  — small grey-green arrows
  - Yellow/purple contact lines flashing at active transfers
  - Live info box showing COI lat/lon and speed
  - Map legend (vehicles + nodes) on main plot only
  - Three independent bottom panels with their own context:
      Left  : cumulative COI sighting delivery curve
      Middle: network activity bar chart (BT/WiFi contacts, in-transit, delivered)
      Right : live sighting log showing each report as it arrives
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

def _draw_car(ax, x, y, heading, color, size, label=None, zorder=9):
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


# ── Road label position helpers ────────────────────────────────────────────

def _node_y(G, candidates):
    """Y-coordinate of the first candidate node found in G.
    Falls back to named_xy for OSM graphs where node IDs are numeric."""
    from network.georgetown import named_xy
    for nid in candidates:
        if nid in G.nodes():
            return G.nodes[nid]["y"]
    for nid in candidates:
        pos = named_xy(nid)
        if pos is not None:
            return pos[1]
    return None


def _node_x(G, candidates):
    """X-coordinate of the first candidate node found in G.
    Falls back to named_xy for OSM graphs where node IDs are numeric."""
    from network.georgetown import named_xy
    for nid in candidates:
        if nid in G.nodes():
            return G.nodes[nid]["x"]
    for nid in candidates:
        pos = named_xy(nid)
        if pos is not None:
            return pos[0]
    return None


# ── Main builder ───────────────────────────────────────────────────────────

def build_animation(
    G:               nx.Graph,
    result:          SimulationResult,
    static_positions: dict,
    coi_circuit:     List[str],
    output_path:     str,
    mother_pos:      Optional[Tuple[float, float]] = None,
    dest_label:      Optional[str] = None,
    static_nodes:    Optional[list] = None,
) -> None:
    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)

    frames = result.frames
    if not frames:
        print("[animator] No frames to render.")
        return

    # ── Axis limits ────────────────────────────────────────────────────────
    all_x = [G.nodes[n]["x"] for n in G.nodes()]
    all_y = [G.nodes[n]["y"] for n in G.nodes()]
    pad   = 40
    xmin, xmax = min(all_x) - pad, max(all_x) + pad
    ymin, ymax = min(all_y) - pad, max(all_y) + pad

    # ── Figure — 4-panel layout ────────────────────────────────────────────
    fig = plt.figure(figsize=(14, 12), facecolor="#0d1117")
    ax_road      = fig.add_axes([0.04, 0.28, 0.92, 0.68], facecolor="#0d1117")
    ax_stats     = fig.add_axes([0.04, 0.04, 0.28, 0.20], facecolor="#161b22")
    ax_bar       = fig.add_axes([0.37, 0.04, 0.28, 0.20], facecolor="#161b22")
    ax_sightings = fig.add_axes([0.70, 0.04, 0.27, 0.20], facecolor="#161b22")

    for ax in (ax_road, ax_stats, ax_bar, ax_sightings):
        for sp in ax.spines.values():
            sp.set_edgecolor("#30363d")

    # ── Road network ───────────────────────────────────────────────────────
    ax_road.set_xlim(xmin, xmax)
    ax_road.set_ylim(ymin, ymax)
    ax_road.set_aspect("equal")
    ax_road.tick_params(colors="#8b949e", labelsize=6)
    ax_road.set_xlabel(
        f"Metres East  (origin ≈ 37th St NW & M St NW  "
        f"{config.LAT_ORIGIN}°N, {abs(config.LON_ORIGIN):.4f}°W)",
        color="#8b949e", fontsize=7,
    )
    ax_road.set_ylabel("Metres North", color="#8b949e", fontsize=7)

    for u, v_n in G.edges():
        x1, y1 = G.nodes[u]["x"],   G.nodes[u]["y"]
        x2, y2 = G.nodes[v_n]["x"], G.nodes[v_n]["y"]
        ax_road.plot([x1, x2], [y1, y2], color="#21262d", lw=5, zorder=1,
                     solid_capstyle="round")
        ax_road.plot([x1, x2], [y1, y2], color="#3d4450", lw=2, zorder=2,
                     solid_capstyle="round")

    for nid in G.nodes():
        ax_road.plot(G.nodes[nid]["x"], G.nodes[nid]["y"],
                     ".", color="#3d4450", ms=2.5, zorder=4)

    # COI route ghost
    rx = [G.nodes[n]["x"] for n in coi_circuit if n in G.nodes()]
    ry = [G.nodes[n]["y"] for n in coi_circuit if n in G.nodes()]
    ax_road.plot(rx, ry, color="#f72585", lw=1.0, alpha=0.15, zorder=3,
                 linestyle=":")

    # ── Static IoT nodes ───────────────────────────────────────────────────
    radio_map = {}
    if static_nodes:
        radio_map = {n.node_id: n.radio_type for n in static_nodes}

    _NODE_STYLE = {
        "bt":   ("^", "#00b4d8", True,  False),
        "wifi": ("s", "#c77dff", False, True),
        "both": ("D", "#4cc9f0", True,  True),
    }

    for sid, (sx, sy) in static_positions.items():
        rt = radio_map.get(sid, "both")
        marker, color, draw_bt, draw_wifi = _NODE_STYLE[rt]
        if draw_wifi:
            ax_road.add_patch(plt.Circle((sx, sy), config.WIFI_RANGE,
                                         color="#7209b7", alpha=0.07, zorder=3))
        if draw_bt:
            ax_road.add_patch(plt.Circle((sx, sy), config.BT_RANGE,
                                         color="#00b4d8", alpha=0.10, zorder=4))
        ax_road.plot(sx, sy, marker=marker, color=color, ms=9, zorder=6,
                     markeredgecolor="white", markeredgewidth=0.6)
        ax_road.text(sx + 3, sy + 3, sid, color=color, fontsize=5.5, zorder=7)

    # Destination marker
    if mother_pos is not None:
        mx, my = mother_pos
        label  = f"DEST\n({dest_label})" if dest_label else "DEST"
        ax_road.plot(mx, my, marker="D", color="#ffd700", ms=14, zorder=8,
                     markeredgecolor="white", markeredgewidth=1.0)
        ax_road.text(mx + 4, my + 5, label, color="#ffd700",
                     fontsize=7, fontweight="bold", zorder=9,
                     bbox=dict(boxstyle="round,pad=0.25", facecolor="#161b22",
                               edgecolor="#ffd700", alpha=0.85))

    # ── Map legend: vehicles + node types (no stats entries) ──────────────
    escort_color = config.ESCORT_COLORS[0]
    map_legend = [
        mpatches.Patch(color="#f72585",    label="Car of Interest (COI)"),
        mpatches.Patch(color=escort_color, label="Escort / Follower"),
        mpatches.Patch(color="#4caf50",    label="Background traffic"),
        mpatches.Patch(color="#ffd700", label="Destination node"),
        mpatches.Patch(color="#00b4d8",
                       label=f"BT-only IoT node  (▲, {config.BT_RANGE:.0f} m)"),
        mpatches.Patch(color="#c77dff",
                       label=f"WiFi-only IoT node  (■, {config.WIFI_RANGE:.0f} m)"),
        mpatches.Patch(color="#4cc9f0",   label="BT+WiFi IoT node  (◆)"),
        mpatches.Patch(color="#00b4d8",   alpha=0.3,
                       label=f"BT range ring  ({config.BT_RANGE:.0f} m)"),
        mpatches.Patch(color="#7209b7",   alpha=0.3,
                       label=f"WiFi range ring  ({config.WIFI_RANGE:.0f} m)"),
        mpatches.Patch(color="#ffe066",   label="Active BT contact link"),
        mpatches.Patch(color="#c77dff",   label="Active WiFi contact link"),
    ]
    ax_road.legend(handles=map_legend, loc="lower right", fontsize=6.5,
                   facecolor="#161b22", labelcolor="#c9d6df",
                   edgecolor="#30363d", framealpha=0.95)

    # ── Title and COI info box ─────────────────────────────────────────────
    title_obj = ax_road.set_title(
        "Georgetown DTN  |  t=00:00",
        color="#e6edf3", fontsize=11, pad=8,
    )
    info_box = ax_road.text(
        xmin + 5, ymax - 8, "", color="#e6edf3", fontsize=7.5, va="top",
        zorder=10,
        bbox=dict(boxstyle="round,pad=0.4", facecolor="#161b22",
                  edgecolor="#30363d", alpha=0.9),
    )

    # ── Bottom-left: cumulative delivery curve ─────────────────────────────
    ax_stats.set_xlim(0, config.SIM_DURATION / 60)
    ax_stats.set_ylim(0, max(len(result.delivered_msgs) + 5, 10))
    ax_stats.set_xlabel("Time (min)", color="#9a9abf", fontsize=8)
    ax_stats.set_ylabel("Sightings", color="#9a9abf", fontsize=8)
    ax_stats.set_title("Cumulative COI Sightings Delivered", color="#c9d6df", fontsize=9)
    ax_stats.tick_params(colors="#9a9abf", labelsize=7)
    line_del, = ax_stats.plot([], [], color="#4caf50", lw=2)

    # ── Bottom-mid: network activity bars ─────────────────────────────────
    bar_objs = ax_bar.bar(
        ["BT\ncontacts", "WiFi\ncontacts", "In-transit", "Delivered"],
        [0, 0, 0, 0],
        color=["#00b4d8", "#c77dff", "#ffe066", "#4caf50"],
        edgecolor="#0d1117",
    )
    ax_bar.set_ylim(0, max(len(result.all_messages) + 5, 10))
    ax_bar.set_ylabel("Count", color="#9a9abf", fontsize=8)
    ax_bar.set_title("Network Activity", color="#c9d6df", fontsize=9)
    ax_bar.tick_params(colors="#9a9abf", labelsize=7)

    # ── Bottom-right: live sighting log ────────────────────────────────────
    ax_sightings.set_xlim(0, 1)
    ax_sightings.set_ylim(0, 1)
    ax_sightings.set_xticks([])
    ax_sightings.set_yticks([])
    ax_sightings.set_title("COI Sightings Received", color="#c9d6df", fontsize=9)

    # Pre-compute sighting delivery events sorted by delivery time
    _sight_events = sorted(
        [m for m in result.delivered_msgs
         if m.msg_type == "COI_SIGHTING" and m.delivery_time >= 0],
        key=lambda m: m.delivery_time,
    )

    sight_text = ax_sightings.text(
        0.04, 0.94, "— waiting for first sighting —",
        color="#484f58", fontsize=6.5, va="top", ha="left",
        transform=ax_sightings.transAxes,
        family="monospace",
        zorder=5,
    )
    sight_count = ax_sightings.text(
        0.96, 0.06, "0 sightings",
        color="#8b949e", fontsize=7, va="bottom", ha="right",
        transform=ax_sightings.transAxes,
        zorder=5,
    )

    # ── Per-frame mutable state ────────────────────────────────────────────
    flash_arts: list = []
    veh_arts:   list = []
    delivery_count_by_frame = _precompute_delivery_counts(result)

    def animate(fi: int):
        fr   = frames[fi]
        t    = fr["time"]
        mins = int(t // 60)
        secs = int(t % 60)

        # Remove previous vehicle artists
        for a in veh_arts:
            try: a.remove()
            except: pass
        veh_arts.clear()

        # Draw vehicles
        for vs in fr["vehicles"]:
            is_coi      = vs["is_coi"]
            is_escort   = vs.get("is_escort", False)
            is_follower = vs.get("is_follower", False)
            hdg         = vs.get("heading", 0.0)

            if is_coi:
                size, label, zo = 65, "COI", 11
            elif is_escort:
                size, label, zo = 50, vs["vid"], 10
            elif is_follower:
                size, label, zo = 45, vs["vid"], 10
            else:
                size, label, zo = 35, None, 9

            veh_arts.extend(
                _draw_car(ax_road, vs["x"], vs["y"], hdg,
                          vs["color"], size, label=label, zorder=zo)
            )

        # Flash contact links
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

        # Title
        title_obj.set_text(
            f"Georgetown DTN  |  t={mins:02d}:{secs:02d}  |  "
            f"BT links: {bt_n}  WiFi links: {wf_n}"
        )

        # COI info box
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

        # Delivery curve
        t_mins_hist = [frames[i]["time"] / 60 for i in range(fi + 1)]
        del_hist    = [delivery_count_by_frame[i] for i in range(fi + 1)]
        line_del.set_data(t_mins_hist, del_hist)

        # Activity bars
        in_transit = sum(v["buf_len"] for v in fr["vehicles"])
        n_del      = delivery_count_by_frame[fi]
        for bar, h in zip(bar_objs, [bt_n, wf_n, in_transit, n_del]):
            bar.set_height(h)

        # Live sighting log — show up to 5 most recent sightings delivered by now
        visible = [e for e in _sight_events if e.delivery_time <= t]
        if visible:
            last5 = visible[-5:]
            lines = []
            for m in last5:
                tm = int(m.delivery_time)
                mm, ss = tm // 60, tm % 60
                ns    = m.origin[:8].ljust(8)
                lat_s = f"{abs(m.origin_lat):.4f}°{'N' if m.origin_lat >= 0 else 'S'}"
                lines.append(f"{mm:02d}:{ss:02d}  {ns}  {lat_s}")
            sight_text.set_text("\n".join(lines))
            sight_text.set_color("#c9d6df")
            sight_count.set_text(f"{len(visible)} sightings")
        else:
            sight_text.set_text("— waiting for first sighting —")
            sight_count.set_text("0 sightings")

        return ([title_obj, info_box, line_del, sight_text, sight_count]
                + flash_arts + veh_arts + list(bar_objs))

    ani = animation.FuncAnimation(
        fig, animate, frames=len(frames), interval=80, blit=False,
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
    """Per-frame cumulative count of delivered sighting messages."""
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
