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
DEFAULT_ALTITUDE_M = 0.0  # fixed altitude exported for all simulated nodes

# ── Communication ranges ───────────────────────────────────────────────────
BT_RANGE   = 45.0   # metres — Bluetooth (IoT / BLE)
WIFI_RANGE = 65.0   # metres — WiFi

# ── Road network ───────────────────────────────────────────────────────────
# Geographic origin for local XY coordinate system
LAT_ORIGIN = 38.9041
LON_ORIGIN = -77.0631

# Georgetown DC bounding box (for OSM download if osmnx is available)
BBOX = {
    "lat_min": 38.903,   # just south of M Street NW (38.9041)
    "lat_max": 38.913,   # just north of Q Street NW (38.9117)
    "lon_min": -77.064,  # just west  of 37th St NW  (-77.0631)
    "lon_max": -77.051,  # just east  of 31st St NW  (-77.0520)
}

# Path to pre-built GraphML (used if OSM download unavailable)
GRAPHML_PATH = "network/georgetown.graphml"

# ── Static IoT nodes ───────────────────────────────────────────────────────
# Names must match node IDs in the road graph (intersection-based nodes)
STATIC_NODE_IDS = [
    "WIS_N", "WIS_O", "WIS_P", "WIS_Q",
    "N_33",  "O_33",  "M_36",  "P_33",
]
# Additional IoT nodes placed at random positions along road *edges*
# (between intersections).  Set to 0 to use intersection nodes only.
NUM_EDGE_NODES = 4

# Destination landmark (for example "M_33") or exact OSM node ID.
# None selects a random OSM node using SEED.
DESTINATION_NODE = None

# ── Car of Interest (COI) ──────────────────────────────────────────────────
# Coarse waypoints — gaps are filled with nx.shortest_path at startup.
# Route: Q Street east → 33rd Street south → M Street east →
#        31st Street north → N Street to Wisconsin → Wisconsin north →
#        Q Street west back to start.
COI_CIRCUIT = [
    "Q_37",   # NW corner
    "Q_33",   # Q Street east end
    "M_33",   # 33rd Street south to M Street
    "M_31",   # M Street far east
    "N_31",   # 31st Street north
    "WIS_N",  # Wisconsin / N Street intersection
    "WIS_Q",  # Wisconsin / Q Street intersection
    "Q_37",   # back to start
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

ESCORT_COLORS  = ["#ff6b6b"]  # red — same for all escorts and followers
FOLLOWER_COLORS = ["#ff6b6b"]

# ── Follower vehicles (traffic vehicles that shadow the COI) ──────────────
NUM_FOLLOWERS = 2    # first N traffic vehicles bias their turns toward COI

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
