"""
network/georgetown.py
Georgetown DC road network.

Builds a NetworkX graph from real OSM intersection coordinates.
Streets covered: M, N, O, P, Q Streets NW and Wisconsin Ave NW,
with N-S connectors at 31st, 33rd, 36th, and 37th Streets.

Optionally downloads fresh data via osmnx if available and network
access is present; falls back to the bundled GraphML otherwise.

Usage
-----
    from network.georgetown import load_graph
    G = load_graph()   # returns NetworkX Graph with 'x','y','lat','lon' on nodes
"""

import math
import os
import networkx as nx
from core.utils import ll2xy, haversine_m
import config


# ── Real OSM intersection coordinates ─────────────────────────────────────
# (lat, lon) keyed by a short human-readable ID
_NODES: dict[str, tuple[float, float]] = {
    # M Street NW (east-west)
    "M_37": (38.9041, -77.0631), "M_36": (38.9043, -77.0612),
    "M_35": (38.9045, -77.0594), "M_34": (38.9047, -77.0575),
    "M_33": (38.9049, -77.0557), "M_32": (38.9051, -77.0538),
    "M_31": (38.9053, -77.0520),
    # Wisconsin Ave NW (north-south) — intersections only (M intersection = M_35)
    "WIS_N": (38.9063, -77.0594), "WIS_O": (38.9081, -77.0594),
    "WIS_P": (38.9099, -77.0594), "WIS_Q": (38.9117, -77.0594),
    # N Street NW
    "N_37": (38.9063, -77.0631), "N_36": (38.9063, -77.0612),
    "N_34": (38.9063, -77.0575), "N_33": (38.9063, -77.0557),
    "N_32": (38.9063, -77.0538), "N_31": (38.9063, -77.0520),
    # O Street NW
    "O_37": (38.9081, -77.0631), "O_36": (38.9081, -77.0612),
    "O_34": (38.9081, -77.0575), "O_33": (38.9081, -77.0557),
    "O_32": (38.9081, -77.0538),
    # P Street NW
    "P_37": (38.9099, -77.0631), "P_36": (38.9099, -77.0612),
    "P_34": (38.9099, -77.0575), "P_33": (38.9099, -77.0557),
    "P_32": (38.9099, -77.0538),
    # Q Street NW
    "Q_37": (38.9117, -77.0631), "Q_36": (38.9117, -77.0612),
    "Q_34": (38.9117, -77.0575), "Q_33": (38.9117, -77.0557),
}

_EDGES: list[tuple[str, str]] = [
    # M Street
    ("M_37","M_36"),("M_36","M_35"),("M_35","M_34"),("M_34","M_33"),
    ("M_33","M_32"),("M_32","M_31"),
    # Wisconsin Ave
    ("M_35","WIS_N"),("WIS_N","WIS_O"),("WIS_O","WIS_P"),("WIS_P","WIS_Q"),
    # N Street
    ("N_37","N_36"),("N_36","WIS_N"),("WIS_N","N_34"),("N_34","N_33"),
    ("N_33","N_32"),("N_32","N_31"),
    # O Street
    ("O_37","O_36"),("O_36","WIS_O"),("WIS_O","O_34"),("O_34","O_33"),("O_33","O_32"),
    # P Street
    ("P_37","P_36"),("P_36","WIS_P"),("WIS_P","P_34"),("P_34","P_33"),("P_33","P_32"),
    # Q Street
    ("Q_37","Q_36"),("Q_36","WIS_Q"),("WIS_Q","Q_34"),("Q_34","Q_33"),
    # 37th Street N-S
    ("M_37","N_37"),("N_37","O_37"),("O_37","P_37"),("P_37","Q_37"),
    # 36th Street N-S
    ("M_36","N_36"),("N_36","O_36"),("O_36","P_36"),("P_36","Q_36"),
    # 33rd Street N-S
    ("M_33","N_33"),("N_33","O_33"),("O_33","P_33"),("P_33","Q_33"),
    # 32nd Street N-S
    ("M_32","N_32"),("N_32","O_32"),("O_32","P_32"),
    # 31st Street N-S
    ("M_31","N_31"),
]


def _build_graph() -> nx.Graph:
    G = nx.Graph()

    for nid, (lat, lon) in _NODES.items():
        x, y = ll2xy(lat, lon)
        G.add_node(nid, lat=float(lat), lon=float(lon),
                   x=float(round(x, 4)), y=float(round(y, 4)))

    seen: set = set()
    for u, v in _EDGES:
        if u == v:
            continue
        key = tuple(sorted([u, v]))
        if key in seen:
            continue
        seen.add(key)
        lat1, lon1 = _NODES[u]
        lat2, lon2 = _NODES[v]
        length = haversine_m(lat1, lon1, lat2, lon2)
        if length < 1.0:
            continue
        G.add_edge(u, v, length=float(round(length, 2)))

    # Remove nodes with degree ≤ 1 (dead ends that could trap vehicles)
    dead_ends = [n for n in G.nodes() if G.degree(n) <= 1]
    G.remove_nodes_from(dead_ends)

    return G


def load_graph(try_osmnx: bool = False) -> nx.Graph:
    """
    Load the Georgetown road graph.

    Parameters
    ----------
    try_osmnx : if True, attempt a live OSM download via osmnx first.
                Falls back to the bundled data if osmnx is unavailable
                or the network request fails.

    Returns
    -------
    NetworkX Graph with node attributes: lat, lon, x, y
    """
    if try_osmnx:
        try:
            import osmnx as ox
            bbox = (
                config.BBOX["lon_min"], config.BBOX["lat_min"],
                config.BBOX["lon_max"], config.BBOX["lat_max"],
            )
            G_raw = ox.graph_from_bbox(bbox=bbox, network_type="drive")
            G = nx.Graph(G_raw.to_undirected())
            for n, data in G.nodes(data=True):
                lat = data.get("y", 0.0)
                lon = data.get("x", 0.0)
                x, y = ll2xy(lat, lon)
                G.nodes[n]["lat"] = float(lat)
                G.nodes[n]["lon"] = float(lon)
                G.nodes[n]["x"]   = float(x)
                G.nodes[n]["y"]   = float(y)
            print(f"[network] OSM download: {G.number_of_nodes()} nodes, "
                  f"{G.number_of_edges()} edges")
            return G
        except Exception as e:
            print(f"[network] osmnx unavailable ({e}), using bundled graph.")

    # Load pre-built GraphML if it exists
    graphml_path = os.path.join(os.path.dirname(__file__), "georgetown.graphml")
    if os.path.exists(graphml_path):
        try:
            G = nx.read_graphml(graphml_path)
            for n in G.nodes():
                if "x" not in G.nodes[n]:
                    lat = float(G.nodes[n]["lat"])
                    lon = float(G.nodes[n]["lon"])
                    x, y = ll2xy(lat, lon)
                    G.nodes[n]["x"] = x
                    G.nodes[n]["y"] = y
            print(f"[network] Loaded GraphML: {G.number_of_nodes()} nodes, "
                  f"{G.number_of_edges()} edges")
            return G
        except Exception as e:
            print(f"[network] GraphML load failed ({e}), rebuilding from coordinates.")

    # Build from bundled coordinates
    G = _build_graph()
    try:
        nx.write_graphml(G, graphml_path)
    except Exception:
        pass
    print(f"[network] Built from coordinates: {G.number_of_nodes()} nodes, "
          f"{G.number_of_edges()} edges")
    return G


def name_to_node(G: nx.Graph) -> dict:
    """
    Map each named Georgetown intersection to the nearest node in G.

    When using the hand-coded graph the names exist directly.
    When using the real OSM graph (numeric node IDs) we find the nearest
    node by coordinate so the rest of the code can still use human-readable
    names to look up positions and build routes.

    Returns {name: node_id_in_G}.
    """
    from core.utils import ll2xy, dist_m
    out = {}
    for name, (lat, lon) in _NODES.items():
        if name in G.nodes():
            out[name] = name
            continue
        tx, ty = ll2xy(lat, lon)
        best, best_d = None, float("inf")
        for n in G.nodes():
            d = dist_m(G.nodes[n]["x"], G.nodes[n]["y"], tx, ty)
            if d < best_d:
                best, best_d = n, d
        out[name] = best
    return out


def named_xy(name: str):
    """
    Return (x, y) in local metres for a named intersection, or None.
    Works even when the name is not a node in the current graph — useful
    for placing street labels on the animation regardless of graph type.
    """
    from core.utils import ll2xy
    coords = _NODES.get(name)
    return ll2xy(*coords) if coords is not None else None


def make_random_route(G: nx.Graph, start: str, length: int = 20,
                      rng=None) -> list[str]:
    """
    Build a random walk route from start of the given length.
    Avoids dead ends and prefers not to immediately reverse direction.
    """
    import random
    rng = rng or random
    route = [start]
    for _ in range(length):
        neighbors = [n for n in G.neighbors(route[-1]) if G.degree(n) > 1]
        if not neighbors:
            neighbors = list(G.neighbors(route[-1]))
        if not neighbors:
            break
        avoid = route[-2] if len(route) > 1 else None
        choices = [n for n in neighbors if n != avoid] or neighbors
        route.append(rng.choice(choices))
    return route
