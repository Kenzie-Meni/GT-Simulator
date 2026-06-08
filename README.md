# DTN Anytime Scheduler — Georgetown

**A Delay-Tolerant Network simulation over a real Georgetown DC road network, combining multi-objective offline optimization with an online epsilon-constraint LP scheduler.**

![Python](https://img.shields.io/badge/python-3.10%2B-blue)
![License](https://img.shields.io/badge/license-MIT-green)
![Status](https://img.shields.io/badge/status-active-brightgreen)

---

## Demo

![Simulation demo](assets/demo.gif)

*30-minute Georgetown DC simulation: COI (magenta), escorts (orange/cyan), followers (red/yellow), traffic (green), and IoT static nodes (blue squares). Contact links flash yellow (Bluetooth) or purple (WiFi) during active transfers.*

---

## Overview

This project models a **store-carry-forward vehicular DTN** in Georgetown, Washington DC. A vehicle of interest — the **Car of Interest (COI)** — circulates through the neighborhood carrying data from IoT static nodes (traffic sensors, environmental monitors). Other vehicles act as opportunistic data mules, relaying messages toward a destination when contact windows open.

The scheduling problem is solved in two phases:

| Phase | Algorithm | When |
|---|---|---|
| Offline | NSGA-II Pareto optimizer | Once at startup |
| Online | Epsilon-constraint LP | At every contact event |

The simulation is built on a **real OSM road graph** (M, N, O, P, Q Streets NW and Wisconsin Ave NW), with kinematic vehicles obeying friction-limited acceleration and turn-proportional speed reduction.

---

## Features

- **Realistic road network** — 31 intersections, 46 edges from real Georgetown OSM coordinates
- **Kinematic physics** — friction-limited acceleration, heading-error turn slowdown, waypoint-following
- **Heterogeneous vehicle fleet**
  - COI on a fixed closed circuit
  - Escort vehicles on parallel adjacent-street circuits
  - Follower vehicles with reactive turn bias toward the COI
  - Background traffic with probabilistic intersection routing
- **Two-tier DTN scheduling**
  - Offline NSGA-II explores the 4-objective Pareto front (benefit ↑, CPU ↓, memory ↓, bandwidth ↓)
  - Online epsilon-constraint LP maximizes message benefit at each contact, respecting hard resource ceilings
- **Spray-and-wait forwarding** — bounded message replication (configurable copies)
- **Dual-radio contact model** — Bluetooth (≤40m) and WiFi (≤100m) with per-window technology classification
- **Full connectivity logging** — every contact window logged with duration, min/max distance, lat/lon, and technology
- **Animated MP4 output** — dark-theme top-down map with directional car icons, range rings, contact links, delivery curve, and live status

---

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                        run_simulation.py                    │
│  ┌──────────────┐  ┌──────────────────┐  ┌──────────────┐  │
│  │  network/    │  │     moo/         │  │ simulation/  │  │
│  │  georgetown  │  │  nsga2 (offline) │  │  engine.py   │  │
│  │  .py         │  │  scheduler (LP)  │  │  vehicle.py  │  │
│  │  Road graph  │  │  Pareto front    │  │  static_node │  │
│  └──────┬───────┘  └────────┬─────────┘  └──────┬───────┘  │
│         │                   │                    │          │
│         └───────────────────┴────────────────────┘          │
│                             │                               │
│                     ┌───────▼────────┐                      │
│                     │   output/      │                      │
│                     │  writer.py     │                      │
│                     │  animator.py   │                      │
│                     └───────────────-┘                      │
└─────────────────────────────────────────────────────────────┘
```

### Offline NSGA-II

Runs once at startup over a synthetic population of scheduling decisions. Each individual encodes a message priority ordering; fitness is evaluated on four objectives:

1. **Benefit** — urgency × recency × inverse-distance score (maximize)
2. **CPU usage** — fraction of compute budget consumed (minimize)
3. **Memory usage** — fraction of memory budget consumed (minimize)
4. **Bandwidth** — fraction of contact window bandwidth consumed (minimize)

The Pareto front is retained and wrapped in a `decode_fn` used by the online scheduler to score incoming messages.

### Online Epsilon-Constraint LP

Triggered at every contact event between a vehicle and a static IoT node. Formulates:

```
maximize   Σ benefit_i · x_i
subject to Σ cpu_i     · x_i  ≤  CPU_RESERVE_FLOOR  (hard ceiling)
           Σ mem_i     · x_i  ≤  MEM_RESERVE_FLOOR
           Σ bw_i      · x_i  ≤  BW_FRACTION · contact_window
           x_i ∈ {0, 1}
```

Solved via `scipy.optimize.linprog` with LP relaxation and greedy rounding.

### Kinematic Vehicle Physics

Each vehicle maintains `(x, y, speed, heading, accel)` and updates every timestep:

1. Compute vector to current waypoint
2. Derive desired heading; compute turn error `Δh ∈ [−π, π]`
3. Target speed = `MAX_SPEED × max(TURN_SLOWDOWN, 1 − |Δh|/π × (1 − TURN_SLOWDOWN))`
4. Friction-limited acceleration: `a ≤ min(MAX_ACCEL, μg)`
5. Rate-limited heading update
6. Integrate position; advance waypoint when within `WAYPOINT_RADIUS`

### Dynamic & Reactive Routing

- **All traffic vehicles** use probabilistic intersection routing: at each junction, next hop is sampled by node degree (favors main roads).
- **Follower vehicles** bias their turn weights by `1/distance_to_COI`, causing them to gradually converge toward the COI's side of the network.
- Escort vehicles follow fixed closed circuits on streets parallel and adjacent to the COI's route.

### Spray-and-Wait DTN

Messages propagate via spray-and-wait: each message may be replicated up to `MAX_SPRAY_COPIES` times across the network. Delivery is confirmed when a vehicle carrying the message comes within `WIFI_RANGE` of a static node.

---

## Road Network

Georgetown, Washington DC — real OSM intersection coordinates:

```
Q Street  ·——·    ·——·    ·——·——·
          |        |        |
P Street  ·——·——WIS_P——·——·——·
          |        |        |
O Street  ·——·——WIS_O——·——·——·
          |        |        |
N Street  ·——·——WIS_N——·——·——·  ← COI route
          |        |        |
M Street  ·——·——M_35——·——·——·
         37th    Wisc    33rd  31st
```

**Nodes:** 31 intersections  
**Edges:** 46 road segments  
**Static IoT nodes:** `WIS_N`, `WIS_O`, `WIS_P`, `WIS_Q`, `N_33`, `O_33`, `M_36`, `P_33`

---

## Installation

```bash
# Python 3.10+ required
git clone https://github.com/yourusername/dtn-georgetown.git
cd dtn-georgetown

python3 -m venv venv
source venv/bin/activate        # Windows: venv\Scripts\activate

pip install -r requirements.txt

# ffmpeg required for MP4 animation output
# macOS:  brew install ffmpeg
# Ubuntu: sudo apt install ffmpeg
```

---

## Usage

```bash
# Run the full simulation (30 min simulated time, ~3 min wall time)
python run_simulation.py
```

Outputs are written to `data/`:

| File | Description |
|---|---|
| `connectivity.csv` | One row per contact window — node pair, technology, duration, distance, lat/lon |
| `connectivity.json` | Same data as JSON with summary statistics |
| `simulation.mp4` | Animated top-down map of the full simulation |

---

## Configuration

All parameters live in [`config.py`](config.py). Nothing else needs to be edited for most experiments.

### Timing

| Parameter | Default | Description |
|---|---|---|
| `SIM_DURATION` | `1800` | Simulation length (seconds) |
| `DT` | `1.0` | Physics timestep (seconds) |
| `FRAME_INTERVAL` | `3` | Seconds between captured animation frames |

### Communication Ranges

| Parameter | Default | Description |
|---|---|---|
| `BT_RANGE` | `40.0` | Bluetooth contact range (metres) |
| `WIFI_RANGE` | `100.0` | WiFi contact range (metres) |

### Vehicle Fleet

| Parameter | Default | Description |
|---|---|---|
| `NUM_VEHICLES` | `9` | Total background traffic vehicles |
| `NUM_ESCORTS` | `2` | Escort vehicles on parallel circuits |
| `NUM_FOLLOWERS` | `2` | Traffic vehicles that bias turns toward COI |
| `COI_START_SPEED` | `7.0` | COI initial speed (m/s, ≈ 15.7 mph) |
| `VEHICLE_SPEED_MIN` | `4.0` | Traffic min speed (m/s, ≈ 9 mph) |
| `VEHICLE_SPEED_MAX` | `9.0` | Traffic max speed (m/s, ≈ 20 mph) |
| `ROUTE_LENGTH` | `20` | Seed route length for dynamic-routing vehicles |

### Physics

| Parameter | Default | Description |
|---|---|---|
| `MAX_SPEED` | `11.0` | Hard speed cap (m/s, ≈ 25 mph) |
| `MIN_SPEED` | `1.5` | Minimum speed — vehicles never fully stop |
| `MAX_ACCEL` | `2.0` | Maximum acceleration (m/s²) |
| `MAX_BRAKE` | `4.0` | Maximum deceleration (m/s²) |
| `MU_FRICTION` | `0.7` | Road surface friction coefficient |
| `TURN_SLOWDOWN` | `0.45` | Speed fraction retained at 180° turn |
| `WAYPOINT_RADIUS` | `15.0` | Capture radius for waypoint advance (metres) |

### DTN / Messaging

| Parameter | Default | Description |
|---|---|---|
| `MESSAGE_TTL` | `900` | Seconds before a message expires |
| `MAX_SPRAY_COPIES` | `4` | Max copies of one message in the network |
| `BUFFER_CAPACITY` | `10` | Max messages a vehicle can carry |
| `COI_SIGHTING_INTERVAL` | `30` | Seconds between COI sighting events at nearby nodes |

### Scheduler Resources

| Parameter | Default | Description |
|---|---|---|
| `CPU_RESERVE_FLOOR` | `0.20` | Keep 20% CPU free |
| `MEM_RESERVE_FLOOR` | `0.15` | Keep 15% memory free |
| `BW_FRACTION` | `0.80` | Use at most 80% of contact window bandwidth |

### NSGA-II

| Parameter | Default | Description |
|---|---|---|
| `NSGA2_GENERATIONS` | `40` | Offline optimizer generations |
| `NSGA2_POP_SIZE` | `60` | Population size per generation |

### Animation

| Parameter | Default | Description |
|---|---|---|
| `RECORD_ANIMATION` | `True` | Save MP4 (set to False to skip) |
| `ANIMATION_FPS` | `15` | Output frame rate |
| `ANIMATION_DPI` | `120` | Output resolution |

---

## Project Structure

```
dtn-georgetown/
├── core/
│   ├── message.py          # Message dataclass — urgency scoring, expiry, hop tracking
│   ├── resource.py         # ResourceSnapshot — CPU/memory/bandwidth budget
│   └── utils.py            # ll2xy, xy2ll, haversine_m, dist_m, classify_tech
├── simulation/
│   ├── vehicle.py          # Kinematic vehicle — physics, dynamic routing, DTN buffer
│   ├── static_node.py      # IoT static node — sighting generation, LP scheduler
│   └── engine.py           # Main loop — stepping, contact detection, frame recorder
├── moo/
│   ├── nsga2.py            # Offline NSGA-II Pareto optimizer
│   └── scheduler.py        # Online epsilon-constraint LP (scipy linprog)
├── network/
│   ├── georgetown.py       # Road graph — real OSM coords, random route generator
│   └── georgetown.graphml  # Pre-built graph (auto-regenerated if missing)
├── output/
│   ├── writer.py           # CSV + JSON connectivity log writers
│   └── animator.py         # MP4 builder — FancyArrow cars, range rings, stats panels
├── tests/
│   ├── test_moo.py
│   ├── test_vehicle.py
│   └── test_connectivity.py
├── assets/
│   └── demo.gif            # Animation preview (first 20s)
├── data/                   # Simulation outputs (gitignored except demo assets)
├── run_simulation.py       # Entry point
├── config.py               # All tunable parameters
└── requirements.txt
```

---

## Sample Results

Running with default config on a 1800s (30-minute) simulation:

```
Messages generated  : 22
Delivered           : 22  (100.0%)
Avg delivery delay  : 8s
Connectivity windows: 818   (BT=711, WiFi=107)
LP solves           : 49
LP fallbacks        : 0
```

---

## Dependencies

| Package | Purpose |
|---|---|
| `numpy` | Numerical arrays, NSGA-II fitness evaluation |
| `scipy` | `linprog` for the online LP scheduler |
| `matplotlib` | Animation rendering (FuncAnimation + FancyArrow) |
| `networkx` | Road graph, shortest-path, neighbor traversal |
| `ffmpeg` | MP4 encoding (system dependency, not pip) |

Optional:
- `osmnx` — live OSM graph download (falls back to bundled GraphML if unavailable)
- `jupyter` / `ipykernel` — notebook exploration

---

## License

MIT
