"""
core/utils.py
Coordinate helpers and distance utilities.
"""

import math
import config


def ll2xy(lat: float, lon: float) -> tuple[float, float]:
    """Convert (lat, lon) to local XY metres from the configured origin."""
    M_LAT = 111320.0
    M_LON = 111320.0 * math.cos(math.radians(config.LAT_ORIGIN))
    x = (lon - config.LON_ORIGIN) * M_LON
    y = (lat - config.LAT_ORIGIN) * M_LAT
    return x, y


def xy2ll(x: float, y: float) -> tuple[float, float]:
    """Convert local XY metres back to (lat, lon)."""
    M_LAT = 111320.0
    M_LON = 111320.0 * math.cos(math.radians(config.LAT_ORIGIN))
    lat = config.LAT_ORIGIN + y / M_LAT
    lon = config.LON_ORIGIN + x / M_LON
    return lat, lon


def dist_m(x1: float, y1: float, x2: float, y2: float) -> float:
    """Euclidean distance in metres between two XY points."""
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance in metres between two lat/lon points."""
    R = 6_371_000
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) ** 2
         + math.cos(math.radians(lat1))
         * math.cos(math.radians(lat2))
         * math.sin(dlon / 2) ** 2)
    return R * 2 * math.asin(math.sqrt(a))


def classify_tech(min_dist: float) -> str:
    """
    Classify a contact window's technology from the minimum distance achieved.
    Classified at window close so brief close-passes are captured correctly.
    """
    if min_dist <= config.BT_RANGE:
        return "bluetooth"
    if min_dist <= config.WIFI_RANGE:
        return "wifi"
    return "none"
