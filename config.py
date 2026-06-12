"""
config.py
All simulation parameters in one place.
Edit this file to adjust the simulation without touching any other code.
"""

# ── Random seed ────────────────────────────────────────────────────────────
SEED = 42

# ── Simulation timing ──────────────────────────────────────────────────────
SIM_DURATION   = 1800   # seconds (30 minutes)
DT             = 1.0    # timestep in seconds

# ── Communication ranges ───────────────────────────────────────────────────
BT_RANGE   = 40.0   # metres — Bluetooth (IoT / BLE)
WIFI_RANGE = 100.0  # metres — WiFi (wider band → more 40-100m WiFi-only contacts)

# ── Road network ───────────────────────────────────────────────────────────
# Geographic origin for local XY coordinate system
LAT_ORIGIN = 38.9041
LON_ORIGIN = -77.0631

# Georgetown DC bounding box (for OSM download if osmnx is available)
BBOX = {
    "lat_min": 38.895,
    "lat_max": 38.915,
    "lon_min": -77.075,
    "lon_max": -77.045,
}

# Path to pre-built GraphML (used if OSM download unavailable)
GRAPHML_PATH = "network/georgetown.graphml"

# ── Static IoT nodes ───────────────────────────────────────────────────────
# Names must match node IDs in the road graph
STATIC_NODE_IDS = [
    "WIS_N", "WIS_O", "WIS_P", "WIS_Q",
    "N_33",  "O_33",  "M_36",  "P_33",
]

# ── Car of Interest (COI) ──────────────────────────────────────────────────
# Ordered list of graph node IDs forming the closed circuit
COI_CIRCUIT = [
    "N_37", "WIS_N", "WIS_O", "O_33", "N_33", "N_32", "N_31",
    "M_32", "M_33",  "M_34",  "M_35", "M_36", "M_37", "N_37",
]
COI_START_SPEED = 7.0   # m/s

# ── Escort vehicles (travel near COI on adjacent streets) ─────────────────
NUM_ESCORTS = 2

# Escort 1: P/O Street loop — one block north of the COI's main N Street route
ESCORT_1_CIRCUIT = [
    "P_37", "P_36", "WIS_P", "P_34", "P_33",
    "O_33", "O_34", "WIS_O", "O_36", "O_37", "P_37",
]

# Escort 2: M Street loop — one block south, uses WIS_N as the northern connector
ESCORT_2_CIRCUIT = [
    "M_37", "M_36", "M_35", "WIS_N", "N_36", "N_37", "M_37",
]

ESCORT_COLORS = ["#ff9800", "#00e5ff"]  # orange, cyan

# ── Follower vehicles (traffic vehicles that shadow the COI) ──────────────
NUM_FOLLOWERS    = 2    # first N traffic vehicles bias their turns toward COI
FOLLOWER_COLORS  = ["#ff6b6b", "#ffd93d"]   # red, yellow

# ── Background traffic vehicles ────────────────────────────────────────────
NUM_VEHICLES      = 9
VEHICLE_SPEED_MIN = 4.0    # m/s
VEHICLE_SPEED_MAX = 9.0    # m/s
ROUTE_LENGTH      = 20     # waypoints per random route

# ── Vehicle physics ────────────────────────────────────────────────────────
MAX_SPEED     = 11.0   # m/s (~25 mph city)
MIN_SPEED     = 1.5    # m/s (never fully stop)
MAX_ACCEL     = 2.0    # m/s²
MAX_BRAKE     = 4.0    # m/s²
MU_FRICTION   = 0.7    # road surface friction coefficient
TURN_SLOWDOWN = 0.45   # fraction of max speed at sharp turns
WAYPOINT_RADIUS = 15.0  # metres — how close before advancing to next waypoint

# ── Message / DTN parameters ───────────────────────────────────────────────
MESSAGE_TTL        = 900   # seconds before a sighting message expires
MAX_SPRAY_COPIES   = 4     # max copies of one sighting in the network
BUFFER_CAPACITY    = 10    # max messages a vehicle can carry (priority queue)
COI_SIGHTING_INTERVAL = 30  # seconds between COI sighting events

# ── File transfer parameters ───────────────────────────────────────────────
FILE_CHUNK_COUNT   = 1000  # total chunks in the large file
CHUNK_BASE_BENEFIT = 0.15  # flat LP benefit for a single chunk (vs sighting ~0.6-0.9)
CHUNK_SPRAY_COPIES = 3     # max copies of one chunk in the network
CHUNK_TTL          = 7200  # chunks are valid for 2 hours (not time-sensitive)
# Completion fractions that trigger a FILE_ACK message back toward the source
ACK_THRESHOLDS     = [0.25, 0.50, 0.75, 1.00]

# ── Resource budget (scheduler) ────────────────────────────────────────────
CPU_RESERVE_FLOOR = 0.20   # keep 20% CPU free for other tasks
MEM_RESERVE_FLOOR = 0.15   # keep 15% memory free
BW_FRACTION       = 0.80   # use at most 80% of contact window bandwidth

# ── Offline NSGA-II ────────────────────────────────────────────────────────
NSGA2_GENERATIONS = 40
NSGA2_POP_SIZE    = 60

# ── Output / animation ────────────────────────────────────────────────────
OUTPUT_DIR       = "data"
RECORD_ANIMATION = True
FRAME_INTERVAL   = 3      # seconds between captured animation frames
ANIMATION_FPS    = 15
ANIMATION_DPI    = 120
ANIMATION_BITRATE = 1800
