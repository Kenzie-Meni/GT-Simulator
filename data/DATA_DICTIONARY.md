# Data Dictionary

Describes every field in the three output files produced by this simulation.

---

## connectivity.csv

One row per **contact window** — a continuous period during which two entities
(vehicle–vehicle or vehicle–node) were within radio range of each other.
Written by a single `run_simulation.py` run.

| Column | Type | Description |
|---|---|---|
| `node_a` | string | ID of the first entity in the contact pair (e.g. `COI`, `V03`, `WIS_N`, `EDGE_2`) |
| `node_b` | string | ID of the second entity |
| `tech` | string | Radio technology used: `bluetooth` (≤45 m, shared BT channel) or `wifi` (≤65 m, shared WiFi channel) |
| `start_time` | float (s) | Simulation time when the two entities first came into range |
| `end_time` | float (s) | Simulation time when they moved out of range |
| `duration` | float (s) | Length of the contact window (`end_time − start_time`) |
| `min_dist_m` | float (m) | Closest distance between the two entities during this window |
| `max_dist_m` | float (m) | Farthest distance during this window |
| `start_lat_a` | float (°N) | Latitude of entity A at window open |
| `start_lon_a` | float (°E) | Longitude of entity A at window open |
| `start_lat_b` | float (°N) | Latitude of entity B at window open |
| `start_lon_b` | float (°E) | Longitude of entity B at window open |
| `end_lat_a` | float (°N) | Latitude of entity A at window close |
| `end_lon_a` | float (°E) | Longitude of entity A at window close |
| `end_lat_b` | float (°N) | Latitude of entity B at window close |
| `end_lon_b` | float (°E) | Longitude of entity B at window close |

**Entity ID conventions**

| Prefix | Meaning |
|---|---|
| `COI` | Car of Interest |
| `ESC1`, `ESC2` | Escort vehicles |
| `V00`–`V08` | Background traffic (first two are followers) |
| `WIS_N`, `N_33`, etc. | Named static IoT nodes at Georgetown intersections |
| `EDGE_0`–`EDGE_3` | Randomly placed edge IoT nodes (between intersections) |

---

## batch_results.csv

One row per **simulation run**. Written by `run_batch.py` after N runs.
Each run uses a different random seed, producing a different destination,
file source, vehicle placement, and radio assignment.

### Run metadata

| Column | Type | Description |
|---|---|---|
| `seed` | int | Random seed used for this run (controls all randomness) |
| `dest_label` | string | Name of the nearest named intersection to the randomly chosen destination node (e.g. `WIS_O`) |
| `file_source_node` | string | Static node that was randomly chosen to seed the 1,000 file chunks |

### COI sighting metrics

| Column | Type | Description |
|---|---|---|
| `sightings_generated` | int | Total COI sighting messages created (one per node that observed the COI passing within range, every 30 s) |
| `sightings_delivered` | int | Sightings that reached the destination node |
| `sighting_delivery_rate` | float (0–1) | `sightings_delivered / sightings_generated` |
| `sighting_avg_delay_s` | float (s) | Mean delivery delay across all delivered sightings (`delivery_time − created_at`) |
| `sighting_min_delay_s` | float (s) | Fastest sighting delivery in this run |
| `sighting_max_delay_s` | float (s) | Slowest sighting delivery in this run |
| `sighting_p50_delay_s` | float (s) | Median delivery delay |
| `sighting_avg_hops` | float | Mean number of vehicle hops a sighting took to reach the destination |
| `sightings_expired` | int | Sightings whose 15-minute TTL expired before delivery |
| `time_to_first_sighting_s` | float (s) | Simulation time when the first sighting was delivered; `NaN` if none delivered |

### File chunk metrics

| Column | Type | Description |
|---|---|---|
| `chunks_total` | int | Always 1,000 — total chunks the file was divided into |
| `chunks_delivered` | int | Distinct chunks that reached the destination within the 30-minute sim |
| `chunk_completion_pct` | float (%) | `chunks_delivered / chunks_total × 100` |
| `chunks_per_min` | float | `chunks_delivered / 30` — average delivery throughput |
| `time_to_first_chunk_s` | float (s) | Simulation time when the first chunk arrived at the destination; `NaN` if none delivered |
| `chunk_avg_delay_s` | float (s) | Mean delivery delay for delivered chunks; `NaN` if none delivered |

### File ACK metrics

ACK messages are generated at the destination when chunk completion crosses
25%, 50%, 75%, and 100% thresholds, then carried back to the file source node.
In a 30-minute run with ~3–4% chunk completion, no run crosses the 25%
threshold, so all ACK fields are 0 in the current default configuration.

| Column | Type | Description |
|---|---|---|
| `acks_generated` | int | Number of FILE_ACK messages created (0–4, one per threshold crossed) |
| `acks_delivered` | int | ACKs that made it back to the file source node |
| `ack_delivery_rate` | float (0–1) | `acks_delivered / acks_generated`; `0` if no ACKs generated |

### Network / scheduler metrics

| Column | Type | Description |
|---|---|---|
| `connectivity_windows` | int | Total contact windows logged (vehicle–vehicle + vehicle–node, both directions) |
| `bt_windows` | int | Windows that used Bluetooth (both parties had BT radio, distance ≤45 m) |
| `wifi_windows` | int | Windows that used WiFi (both parties had WiFi radio, distance ≤65 m) |
| `avg_window_duration_s` | float (s) | Mean duration of a contact window |
| `lp_solves` | int | Times the online epsilon-constraint LP found a valid transfer schedule |
| `lp_fallbacks` | int | Times the LP had no feasible solution and fell back to greedy scheduling |
| `total_transfers` | int | Total individual message transfers across all contact events |

---

## connectivity.json (schema version 2.0)

JSON v2 stores each entity's time-indexed path once. Contact windows reference
those entities by ID, avoiding duplicated endpoint coordinates. Positions
between adjacent path points may be linearly interpolated. Mobile paths are
sampled once per simulation timestep; stationary paths contain only the start
and end of the simulation. Altitude currently uses `DEFAULT_ALTITUDE_M`.

```
{
  "schema_version": "2.0",
  "simulation": {
    "duration_s":        simulation length in seconds
    "area":              human-readable area name
    "bbox":              geographic bounding box {lat_min, lat_max, lon_min, lon_max}
    "bluetooth_range_m": BT radio range (metres)
    "wifi_range_m":      WiFi radio range (metres)
    "num_nodes":         total vehicles, static nodes, and destinations
    "default_altitude_m": fixed exported altitude
    "coi_circuit":       ordered list of node IDs on the COI's route
  },
  "nodes": {
    "V00": {
      "node_type": "vehicle",       // vehicle | static | destination
      "radio_type": "both",
      "mobile": true,
      "path": [
        { "time_s": 0.0, "lat": 38.9, "lon": -77.06, "alt_m": 0.0 }
      ]
    }
  },
  "statistics": {
    "total_windows":           total contact windows
    "bluetooth_windows":       BT-technology windows
    "wifi_windows":            WiFi-technology windows
    "total_contact_time_s":    sum of all window durations
    "avg_window_duration_s":   mean window duration
    "longest_window_s":        longest single window
    "total_transfers":         total message transfers
    "scheduler_lp_success":    LP solves
    "total_delay":             sum of sighting delivery delays (s)
    "total_messages":          sighting messages generated
    "delivered":               sightings delivered
    "connectivity_windows":    same as total_windows
    "chunks_total":            1000
    "chunks_delivered":        chunks that reached destination
    "chunk_completion_pct":    percentage completion
    "acks_generated":          FILE_ACK messages generated
    "file_source_node":        node ID that seeded the chunks
  },
  "windows": [
    {                          // one object per contact window
      "node_a", "node_b",      // entity IDs (same conventions as CSV)
      "start_time_s", "end_time_s", "duration_s",
      "min_distance_m", "max_distance_m",
      "technology"             // "bluetooth" or "wifi"
    }
  ]
}
```
