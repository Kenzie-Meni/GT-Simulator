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


def _has_bt(rt: str) -> bool:
    return rt in ("bt", "both")


def _has_wifi(rt: str) -> bool:
    return rt in ("wifi", "both")


def compatible_range(rt_a: str, rt_b: str) -> float:
    """
    Effective contact range between two parties given their radio types.

    WiFi is preferred when both parties support it (longer range).
    Returns 0.0 when no shared channel exists.
    """
    if _has_wifi(rt_a) and _has_wifi(rt_b):
        return config.WIFI_RANGE
    if _has_bt(rt_a) and _has_bt(rt_b):
        return config.BT_RANGE
    return 0.0


def contact_tech(rt_a: str, rt_b: str, dist: float) -> str:
    """
    Radio technology actually used for a contact at the given distance.

    BT is used when both parties have BT and are within BT_RANGE.
    WiFi is used when both parties have WiFi and within WiFi_RANGE.
    Returns 'none' if no shared channel exists at this distance.
    """
    if _has_bt(rt_a) and _has_bt(rt_b) and dist <= config.BT_RANGE:
        return "bluetooth"
    if _has_wifi(rt_a) and _has_wifi(rt_b) and dist <= config.WIFI_RANGE:
        return "wifi"
    return "none"


def classify_tech(min_dist: float, radio_type: str = "both") -> str:
    """
    Classify a contact window's technology from the minimum distance achieved
    and the node's radio capability.

    radio_type : "bt"   — Bluetooth only (max range BT_RANGE)
                 "wifi" — WiFi only      (max range WIFI_RANGE)
                 "both" — both radios    (BT if ≤BT_RANGE, else WiFi)
    """
    if radio_type == "bt":
        return "bluetooth" if min_dist <= config.BT_RANGE else "none"
    if radio_type == "wifi":
        return "wifi" if min_dist <= config.WIFI_RANGE else "none"
    # "both" — or vehicle-to-vehicle (always both radios available)
    if min_dist <= config.BT_RANGE:
        return "bluetooth"
    if min_dist <= config.WIFI_RANGE:
        return "wifi"
    return "none"
