# F1TENTH Project – Part 2: Obstacles and Overtaking

Second part of the F1TENTH project on the **Sepang** circuit (ROS 2 Humble, F1TENTH Gym). The same **Follow the Gap** controller from part 1 was extended to solve two tests:

1. **Static obstacles:** 5 obstacles placed on the vehicle's path; 10 consecutive laps without collision.
2. **Two vehicles:** a second robot starts 5 m ahead of the main vehicle and must be overtaken. Both robots use the part-1 algorithm. The goal is 10 consecutive laps without collision.

**Final results:**

| Test | Laps | Best lap | Collisions | AEB |
|---|---|---|---|---|
| Obstacles | 10/10 | **51.07 s** | 0 | 0 |
| Two vehicles (main) | 10/10 | **49.21 s** | 0 | 0 |
| Two vehicles (opponent) | 10/10 | 57.74 s | 0 | 0 |

**Video evidence:**
- Test 1: [`evidencia/PROYECTOPARTE2_obstaculos.webm`](evidencia/PROYECTOPARTE2_obstaculos.webm)
- Test 2: [`evidencia/PROYECTOPARTE2_2robots.webm`](evidencia/PROYECTOPARTE2_2robots.webm)

*Spanish version: [README_parte2_es.md](README_parte2_es.md) · Part 1: [README.md](README.md)*


---

## 1. Description

Part 1 was not replaced. Both tests run the same node (`gap_node.py`) with the same Follow the Gap pipeline: LiDAR preprocessing, disparity extension, safety bubble, gap selection, steering and speed control, and AEB.

All the new functions are **off by default** and are switched on only with parameters in each test's launch script. This means part 1 (`./iniciar_sepang.sh`) still behaves exactly as before.

The controller remains **reactive**. It uses no map, no localization, and no precomputed trajectory. The only inputs are the LiDAR and the vehicle's own odometry.

## 2. Test 1 – Static Obstacles

### 2.1. Obstacle placement
The obstacles are drawn directly onto a copy of the map with `crear_mapa_obstaculos.py`:
- It takes `Sepang_map.png` and paints 5 squares of 0.40 m side.
- It saves the result as `Sepang_obstaculos.png/.yaml`.
- The simulator treats black pixels as walls, so the LiDAR sees the obstacles like any other wall.

Each obstacle was placed **on the path the vehicle follows without obstacles**, shifted 0.25 m to one side, alternating sides. Each one blocks half of the vehicle's normal line, so the vehicle is forced to steer around it. Without the shift, an obstacle in the exact center would make the vehicle hesitate between both sides.

| # | Position (x, y) [m] | Location |
|---|---|---|
| 1 | (0.17, −9.44) | Long straight, ~66 m after the start |
| 2 | (−9.70, −31.77) | ~160 m |
| 3 | (20.18, −11.16) | ~228 m |
| 4 | (−25.59, 36.90) | ~362 m |
| 5 | (−30.84, −2.43) | Finish straight, ~30 m before the line |

### 2.2. Extra margin only near obstacles
A first attempt raised the global safety margin. That made the vehicle hit the wall in the hairpin, because the free space in tight corners became too narrow.

The adopted solution applies the extra margin **only when a small obstacle is detected**:
- The scan is split into objects, separated by jumps in distance.
- An object counts as an obstacle when it is small (≤ 0.85 m), lies within 8 m and ±45°, and the background behind it is clearly farther on both sides (contrast).
- In that case an extra margin `obs_margin_extra = 0.08 m` is added to the inflation, and it is held for 0.6 s after the obstacle is no longer seen.
- On the circuit without obstacles this margin is never activated, so the lap time does not get worse.

## 3. Test 2 – Two Vehicles and Overtaking

### 3.1. Setup
- `sim_2robots.yaml`: `num_agent: 2`. The opponent starts at (5.0, 0.0), 5 m ahead and with the same heading.
- **Opponent:** the same `gap_node` with remapped topics (`/opp_scan`, `/opp_drive`, `/opp_racecar/odom`). It runs with a lower top speed (`max_speed = 8.7 m/s`, ~58 s per lap) and a small lateral offset (`lane_bias = −0.4`), so it can be caught and overtaken. It also uses `obs_margin_extra`, so it treats the main vehicle as an obstacle.
- **Main vehicle:** the same `gap_node` with `overtake_enabled = true`.

### 3.2. Detecting the other vehicle
The same object detection from test 1 is used, with two extra checks:
- **At least 4 points and a "facing" ratio ≥ 0.5.** This rejects pieces of wall seen at a steep angle, which can look small but are not facing the vehicle.
- **Speed of the object.** It is estimated from how fast its distance changes. A new object is first assumed to be standing still. An object counts as a car ahead only when it is moving faster than 1.5 m/s and is inside the track.

When a car ahead is detected, a timer of 1.5 s is started. "Car near" mode stays active while that timer is running, which avoids switching on and off at every scan.

### 3.3. Overtaking logic
While a car is near:
1. **Passing side:** always on the **left** (`ot_pass_side = 1`). The reference line moves 0.40 m to that side.
2. **Removing the car from Follow the Gap:** the moving car is removed from the gap search, so it does not "close" the gap. A walls-only scan is used for centering and braking.
3. **Wait or pass:** the vehicle only goes for the pass when the passing lane is free far enough ahead (car distance + 12 m + 1 s × speed). Otherwise it stays behind at a safe distance, with a speed limit that leaves room to brake (0.8 m + 0.3 s × speed, at 3 m/s²).
4. **Commit:** once the car is less than 1.0 m ahead, the vehicle does not abort the pass.

In practice the pass happens on the long straight, about 9 s after the start. After that the main vehicle laps the opponent several times without contact.

### 3.4. End of race
Each vehicle stops by itself after 10 laps (`race_laps = 10`). When the main vehicle finishes, it stops on the track, and the opponent keeps going and avoids it as an obstacle.

## 4. Tools Added in Part 2

| File | Role |
|---|---|
| `crear_mapa_obstaculos.py` | Creates the map with the 5 obstacles |
| `iniciar_sepang_obstaculos.sh` | Launches test 1 |
| `iniciar_sepang_2robots.sh` | Launches test 2 (two vehicles, two lap counters) |
| `resumen_carrera.py` | Waits for the 10 laps and prints the summary table |
| `config/sim_obstaculos.yaml` | Simulator setup for test 1 |
| `config/sim_2robots.yaml` | Simulator setup for test 2 |

- **Choosing the configuration:** `gym_bridge_launch.py` reads the environment variable `F1TENTH_SIM_CONFIG`. Each script sets it to its own YAML. When it is not set, the original `sim.yaml` is used (part 1).
- **Lap counter for both cars:** a second `lap_counter` runs, subscribed to `/opp_racecar/odom`, and saves its logs in `~/f1tenth_logs/oponente`.
- **RViz:** each vehicle draws its own trajectory and its own text with laps and times (the opponent in blue).
- **Automatic summary and shutdown:** `resumen_carrera.py` reads `~/f1tenth_logs/vueltas.csv` until every vehicle has completed 10 laps. It then prints a table with the time, average and maximum speed, minimum clearance and AEB of each lap, followed by totals. After that the script closes every process by itself. With Ctrl+C everything also closes cleanly.

## 5. Main Parameters of Part 2

| Parameter | Value | Description |
|---|---|---|
| `obs_margin_extra` | 0.08 m | Extra margin near small obstacles |
| `obs_range` / `obs_angle_deg` | 8 m / 45° | Obstacle search zone |
| `obs_hold` | 0.6 s | Time the extra margin is held |
| `overtake_enabled` | true (main) | Enables overtaking |
| `ot_range` | 9 m | Car search distance |
| `ot_max_size` | 0.85 m | Maximum object size |
| `ot_min_points` / `ot_min_facing` | 4 / 0.5 | Filter against wall fragments |
| `ot_moving_speed` | 1.5 m/s | Minimum speed to count as a car |
| `ot_hold` | 1.5 s | Duration of "car near" mode |
| `ot_pass_side` / `ot_pass_offset` | left / 0.40 m | Passing side and offset |
| `ot_lane_extra` / `ot_lane_time` | 12 m / 1.0 s | Free lane needed to pass |
| `ot_follow_gap` / `ot_follow_time` | 0.8 m / 0.30 s | Following distance |
| `ot_commit` | 1.0 m | Distance after which the pass is not aborted |
| `max_speed` (opponent) | 8.7 m/s | Opponent top speed |
| `lane_bias` (opponent) | −0.4 | Opponent lateral offset |
| `race_laps` | 10 | Laps before stopping |

## 6. Project Structure (Part 2 Files)

```text
F1Tenth-Repository/
├── README_parte2.md
├── README_parte2_es.md
├── crear_mapa_obstaculos.py
├── iniciar_sepang_obstaculos.sh
├── iniciar_sepang_2robots.sh
├── resumen_carrera.py
├── evidencia/
│   ├── PROYECTOPARTE2_obstaculos.webm
│   └── PROYECTOPARTE2_2robots.webm
└── src/
    ├── controllers/controllers/gap_node.py
    └── f1tenth_gym_ros/
        ├── config/
        │   ├── sim_obstaculos.yaml
        │   └── sim_2robots.yaml
        ├── launch/gym_bridge_launch.py
        └── maps/
            ├── Sepang_obstaculos.png
            └── Sepang_obstaculos.yaml
```

## 7. Building and Running

```bash
source /opt/ros/humble/setup.bash
cd ~/F1Tenth-Repository
colcon build --packages-select controllers f1tenth_gym_ros --symlink-install
source install/setup.bash
```

**Test 1 – obstacles:**
```bash
source /opt/ros/humble/setup.bash
cd ~/F1Tenth-Repository
./iniciar_sepang_obstaculos.sh
```

**Test 2 – two vehicles:**
```bash
source /opt/ros/humble/setup.bash
cd ~/F1Tenth-Repository
./iniciar_sepang_2robots.sh
```

Each script closes by itself after 10 laps and prints the summary table. The table can also be printed again manually:
```bash
python3 resumen_carrera.py --tags OBSTACULOS --laps 10 --since 2026-11-22T13:45:00
```

## 8. Results

### Test 1 – Obstacles (10 consecutive laps)

| Metric | Value |
|---|---|
| Completed laps | **10/10** |
| Best lap | **51.07 s** |
| Average lap | 51.17 s |
| Lap-to-lap deviation | 0.08 s |
| Minimum clearance | 0.281 m |
| AEB activations | **0** |
| Reversals | 0 |
| Collisions | **0** |

The cost of avoiding the 5 obstacles is about 2 s per lap compared with part 1 (49.00 s).

### Test 2 – Two vehicles (10 consecutive laps each)

| Metric | Main | Opponent |
|---|---|---|
| Completed laps | **10/10** | **10/10** |
| Best lap | **49.21 s** | 57.74 s |
| Average lap | 49.57 s | 57.92 s |
| Lap-to-lap deviation | 0.45 s | 0.15 s |
| Minimum clearance | 0.337 m | 0.331 m |
| AEB activations | **0** | **0** |
| Reversals | 0 | 0 |
| Collisions | **0** | **0** |

The main vehicle overtakes the opponent at the start and then laps it again during the race. Each pass costs about 1 s, which explains the higher deviation of the main vehicle. A previous run with the opponent at 8.0 m/s (average 62.10 s) also completed 10/10 laps without collision.

### Problems found and solved
- **Global safety margin:** raising `safety_margin` made the vehicle crash in the hairpin. It was replaced by the extra margin that applies only near obstacles.
- **Obstacle in the exact center of the line:** the vehicle hesitated between both sides and crashed. The obstacles were shifted 0.25 m to one side.
- **Walls detected as cars:** wall pieces seen at a steep angle moved the main vehicle's line without reason. The point and facing filter and the "moving object" condition reduced these false detections from 41% of the time to 0.
- **Opponent crashing in the hairpin:** caused by turning off its racing line (`line_gain = 0`). The default value was restored.
- **Contact while passing in the first corner:** the passing-lane check and the "car near" timer were added, so the pass now happens on the straight.

## 9. Conclusions

- Follow the Gap handles obstacles and another vehicle without a map. The obstacles are simply part of the free-space search.
- A safety margin applied everywhere makes the car slower and less safe in tight corners. Applying it **only near obstacles** keeps the part-1 performance where there is nothing to avoid.
- For overtaking, detection quality matters more than aggressiveness. Most of the problems came from walls taken for cars, and they were solved with simple geometric and speed filters.
- Waiting until the passing lane is free makes the pass happen on the straight and avoids contact in corners.
- The automatic summary table at the end of each run made it possible to compare runs with the same metrics and to justify each change with data.
- Future work: choosing the passing side dynamically (left or right) and predicting the other vehicle's path.