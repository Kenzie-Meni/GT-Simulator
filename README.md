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

This project models a **store-carry-forward vehicular DTN** in Georgetown, Washington DC. The network has two competing missions running simultaneously over the same vehicle fleet and radio bandwidth:

**Mission 1 — COI surveillance.** A **Car of Interest (COI)** circulates through the neighborhood. Static IoT nodes (intersections, sensors) observe the COI when it passes within range and generate small, time-sensitive *sighting messages* recording its position and timestamp. Background vehicles act as **data mules**, picking up these sighting reports and carrying them to a destination node.

**Mission 2 — Large file transfer.** One randomly chosen edge node holds a 256 MB file pre-divided into **1,000 chunks** (~256 KB each). The same data mules must also carry chunks toward the destination. Because no vehicle can carry the full file in one trip, chunks spread gradually through the network via spray-and-wait — but they compete directly with sighting messages for the same limited buffer space on every vehicle.

The scheduler resolves this tension in two phases:

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
- **Competing message types** — three types with strict priority ordering
  - `COI_SIGHTING` — small, high priority, time-sensitive (TTL 900s)
  - `FILE_CHUNK` — one fragment of a 1,000-chunk large file; low individual priority, no TTL urgency
  - `FILE_ACK` — completion acknowledgement traveling back from destination to source
- **Priority queue vehicle buffer** — type-ranked eviction (ACK > Sighting > Chunk); a sighting will never be bumped to make room for a chunk regardless of age
- **Large file transfer** — source node randomly assigned at startup; 1,000 chunks dispersed via spray-and-wait; un-dispatched chunks prioritized over already-dispatched ones
- **ACK feedback loop** — destination generates `FILE_ACK` messages at 25/50/75/100% completion thresholds; ACKs ride passing vehicles back to the source, which stops redundantly resending confirmed chunks
- **Spray-and-wait forwarding** — bounded replication with separate caps for sightings (`MAX_SPRAY_COPIES`) and chunks (`CHUNK_SPRAY_COPIES`)
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

### Competing Message Types & Priority Scheduling

Three message types share the same vehicle buffers and radio bandwidth, creating the core scheduling tension:

| Type | Priority rank | TTL | Spray cap | Delivered to |
|---|---|---|---|---|
| `FILE_ACK` | 3 (highest) | 2 hours | — | File source node |
| `COI_SIGHTING` | 2 | 15 min | `MAX_SPRAY_COPIES` | Any static node |
| `FILE_CHUNK` | 1 (lowest) | 2 hours | `CHUNK_SPRAY_COPIES` | Destination only |

**Buffer eviction** is type-ranked: a `FILE_CHUNK` is always evicted before a `COI_SIGHTING`, regardless of the sighting's age or hop count. Within the same type, `priority_score()` breaks ties.

**Sighting priority score:** `benefit × ttl_ratio × 1/(1+hops)` — decays with age and hops  
**Chunk priority score:** `CHUNK_BASE_BENEFIT / (1 + 0.3×hops)` — flat low value, slight decay with hops  
**ACK priority score:** `0.85 / (1 + 0.1×hops)` — high and stable, designed to return to source quickly

**File transfer flow:**
```
Source node seeds 1,000 chunks
        ↓
LP scheduler dispatches un-dispatched chunks first,
then re-dispatches un-acked chunks for redundancy
        ↓
Chunks spread vehicle-to-vehicle (spray-and-wait, cap=3)
        ↓
Vehicles near destination deliver chunks (dest only)
        ↓
At 25/50/75/100% completion → FILE_ACK generated
        ↓
ACK rides mules back to source → source stops
resending already-delivered chunks
```

In a 30-minute simulation: sightings achieve 100% delivery in seconds; chunks reach ~6–7% completion — the competing objective in practice.

### Spray-and-Wait DTN

Messages propagate via spray-and-wait: each message may be replicated up to its type-specific spray cap across the network. Sightings and chunks use separate caps to prevent chunk flooding from starving sighting replication.

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
| `MESSAGE_TTL` | `900` | Seconds before a sighting expires |
| `MAX_SPRAY_COPIES` | `4` | Max copies of one sighting in the network |
| `BUFFER_CAPACITY` | `10` | Max messages a vehicle can carry (priority queue) |
| `COI_SIGHTING_INTERVAL` | `30` | Seconds between COI sighting events at nearby nodes |

### File Transfer

| Parameter | Default | Description |
|---|---|---|
| `FILE_CHUNK_COUNT` | `1000` | Total chunks in the large file (~256 MB ÷ ~256 KB) |
| `CHUNK_BASE_BENEFIT` | `0.15` | Flat LP benefit per chunk (vs sighting ~0.6–0.9) |
| `CHUNK_SPRAY_COPIES` | `3` | Max copies of one chunk in the network |
| `CHUNK_TTL` | `7200` | Chunk validity window (2 hours — not time-sensitive) |
| `ACK_THRESHOLDS` | `[0.25, 0.50, 0.75, 1.00]` | Completion fractions that trigger a return ACK message |

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
Avg delivery delay  : 0s
Connectivity windows: 842   (BT=726, WiFi=116)
LP solves           : 64
LP fallbacks        : 0

File transfer results:
  Source node         : O_33  (randomly chosen each run)
  Chunks delivered    : 65 / 1000  (6.5%)
  ACKs generated      : 0
```

The competing objective is visible in these numbers: COI sightings achieve instant 100% delivery because they dominate the priority queue, while file chunks — 1,000 of them competing for the same 10-slot vehicle buffers — accumulate slowly at ~6–7% per 30 minutes. Full file delivery would require several hours of simulated time, a longer sim duration, more vehicles, or relaxed chunk spray limits.

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
