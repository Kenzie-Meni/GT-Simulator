"""
network/georgetown.py
Georgetown DC road network.

Builds a NetworkX graph from real OSM intersection coordinates.
Streets covered: M, N, O, P, Q Streets NW and Wisconsin Ave NW,
with N-S connectors at 31st, 33rd, 36th, and 37th Streets.

Optionally downloads fresh data via OSMnx and caches it as GraphML. If the
network is unavailable, the last valid OSM cache is used. Hand-built road
geometry is never used as a fallback.

Usage
-----
    from network.georgetown import load_graph
    G = load_graph()   # returns NetworkX Graph with 'x','y','lat','lon' on nodes
"""

import math
import os
import networkx as nx
from core.utils import ll2xy
import config


# Approximate landmark coordinates used only to map readable route names to
# their nearest nodes in the real OSM graph. They never define road geometry.
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


def load_graph(try_osmnx: bool = False) -> nx.Graph:
    """
    Load the Georgetown road graph.

    Parameters
    ----------
    try_osmnx : if True, refresh the graph from live OpenStreetMap first.
                Falls back to the last valid OSM-derived GraphML cache.

    Returns
    -------
    NetworkX Graph with node attributes: lat, lon, x, y
    """
    import osmnx as ox

    graphml_path = os.path.join(os.path.dirname(__file__), "georgetown.graphml")
    if try_osmnx:
        try:
            bbox = (
                config.BBOX["lon_min"], config.BBOX["lat_min"],
                config.BBOX["lon_max"], config.BBOX["lat_max"],
            )
            G_raw = ox.graph_from_bbox(bbox=bbox, network_type="drive")
            ox.save_graphml(G_raw, graphml_path)
            print(f"[network] OSM download: {G_raw.number_of_nodes()} nodes, "
                  f"{G_raw.number_of_edges()} edges")
        except Exception as e:
            print(f"[network] OSM refresh failed ({e}); using cached OSM graph.")

    if os.path.exists(graphml_path) and os.path.getsize(graphml_path) > 0:
        try:
            G_raw = ox.load_graphml(graphml_path)
        except Exception as e:
            raise RuntimeError(f"Invalid cached OSM graph: {graphml_path}") from e
    else:
        raise RuntimeError(
            "No valid Georgetown OSM cache is available. Run with "
            "load_graph(try_osmnx=True) while online."
        )

    G = nx.Graph(G_raw.to_undirected())
    for n, data in G.nodes(data=True):
        lat, lon = float(data["y"]), float(data["x"])
        x, y = ll2xy(lat, lon)
        G.nodes[n].update(lat=lat, lon=lon, x=float(x), y=float(y))
    print(f"[network] Loaded OSM GraphML: {G.number_of_nodes()} nodes, "
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
