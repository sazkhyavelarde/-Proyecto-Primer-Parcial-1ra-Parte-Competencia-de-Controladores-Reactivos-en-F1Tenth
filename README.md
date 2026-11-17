# F1TENTH Project – Reactive Controller Competition

**Follow the Gap** reactive controller for an autonomous F1TENTH vehicle on the **Sepang** circuit, developed with ROS 2 Humble and evaluated in the F1TENTH Gym simulator.

**Final result:** best lap of **49.00 s**, 10 consecutive laps with no collisions and no emergency braking activations.

**Video evidence:** [`evidencia/PROYECTOPARTE1.webm`](evidencia/PROYECTOPARTE1.webm)

*Spanish version: [README_es.md](README_es.md)*

---

## 1. Project Description

The goal of this project is for the vehicle to drive the Sepang circuit autonomously, as fast as possible and **without colliding**, using only LiDAR and odometry measurements.

The **Follow the Gap** algorithm was implemented as a **reactive** controller: it uses no map or precomputed trajectory. At every instant it looks for the most suitable free space and steers the vehicle toward it. On top of this, speed and stability control was added based on the vehicle's dynamic model, which is what made it possible to reduce lap time without losing stability.

A timing node was also developed (`lap_counter.py`) that records lap and sector times and generates statistics to compare test runs.

## 2. Development Tools

- **Ubuntu** – operating system for the simulation environment.
- **ROS 2 Humble** – communication between nodes through topics.
- **Python** – language used for the nodes.
- **F1TENTH Gym** – vehicle simulator (single-track dynamic model).
- **RViz** – visualization of the vehicle, map, trajectory and lap times.
- **Visual Studio Code** – code editor.

## 3. Architecture

The system follows the classic autonomous vehicle loop: **perception → planning → control**.

| Node | File | Subscribes | Publishes | Role |
|---|---|---|---|---|
| `reactive_node` | `gap_node.py` | `/scan`, `/ego_racecar/odom` | `/drive`, `/trayectoria` | Follow the Gap controller |
| `lap_counter` | `lap_counter.py` | `/ego_racecar/odom` | — | Lap and sector timing |

Commands are sent as `AckermannDriveStamped` messages (speed and steering angle).

## 4. Controller Operation (`gap_node.py`)

The controller runs on every LiDAR scan and performs the following stages:

### 4.1. LiDAR Preprocessing
- Median filter to remove spurious readings.
- Interpolation of invalid readings (NaN).
- Cropping to the forward field of view (±90°).

### 4.2. Obstacle Inflation
- **Disparity extension:** at every wall edge (sudden jump in distance), the obstacle is extended by the vehicle's half-width plus a margin, so every remaining free space is actually drivable.
- **Safety bubble** of 0.30 m around the closest point.
- The safety margin **increases with speed**.

### 4.3. Gap Selection
- Continuous free-space segments (gaps) are identified.
- Each gap receives a dimensionless score based on its **depth**, **width**, **alignment with the heading** and **change from the previous gap**.
- **Hysteresis** is applied: the gap only changes if the new one is clearly better, which prevents oscillation (zigzag).

### 4.4. Target Point
- The target is the **center of the deepest segment** of the selected gap, not a single ray, which gives a stable reference.
- **Lateral centering** is applied on straights, and a slight **inside line** in curves (maximum 0.5 m offset, never closer than 0.45 m to the wall).

### 4.5. Steering Calculation
- **Arc geometry** (bicycle model): δ = atan(L · 2·sin(α) / Ld), with look-ahead Ld = 1.2 m + 0.25 · v.
- **Understeer compensation:** using the steady-state yaw gain of the dynamic model, the steering angle is increased with speed (×1.30 at 6 m/s, ×2.22 at 12 m/s).

### 4.6. Speed Control
The target speed is the **minimum** of several limits:
- **Curvature:** v = √(a_lat / κ).
- **Braking:** speed from which the vehicle can brake stably within the free distance measured **along the planned arc**.
- **Turning space** up to the corner exit.
- **Proximity to the side wall.**
- **Adaptive corner speed:** 8.0 m/s in open corners and 6.25 m/s in hairpins, based on the free distance.

### 4.7. Braking Stability
- In the simulator's model, hard braking at high speed shifts load to the front axle and makes the yaw dynamics unstable (spin-out).
- At startup, the node computes a **stable deceleration table** from a stability analysis of the dynamic model, and never requests more deceleration than is stable at the current speed.
- **Yaw-rate feedback** is active only during hard braking, compared against a reference model that includes the steering delay.

### 4.8. Safety
- **Automatic emergency braking (AEB)** based on time to collision: if TTC < 0.20 s, it brakes at the maximum stable rate.
- **Anti-stuck recovery** with a reverse maneuver.
- **End of race:** after completing 10 laps, the vehicle brakes progressively and stops.

## 5. Lap Counter (`lap_counter.py`)

- Detects the finish line crossing (x = `finish_x`, between `finish_y_min` and `finish_y_max`) in the positive direction, re-arming after moving away from the line.
- The timer starts **when the vehicle begins to move**.
- The crossing instant is **interpolated** between two odometry messages.
- Per lap: time, difference from the best lap, distance, average and maximum speed, and **3 sectors**.
- Final summary: best lap, average, consistency (standard deviation) and **ideal lap** (sum of best sectors).
- History saved to `~/f1tenth_logs/sesiones_sepang.csv`.

## 6. RViz Visualization

The controller publishes on the `/trayectoria` topic (`MarkerArray`):
- the driven trajectory, colored by speed;
- the previous lap in gray;
- the gap target point;
- a text with the current, last and best lap.

To display it in RViz: **Add → By topic → /trayectoria → MarkerArray**.

## 7. Main Parameters

| Parameter | Value | Description |
|---|---|---|
| `max_speed` | 12.0 m/s | Maximum speed |
| `corner_min_speed` | 8.0 m/s | Minimum target speed in open corners |
| `corner_min_sharp` | 6.25 m/s | Target speed in hairpins |
| `a_lat_max` | 10.0 m/s² | Maximum lateral acceleration |
| `ld_min` / `ld_gain` | 1.2 m / 0.25 s | Look-ahead distance |
| `fov_deg` | 90° | Field of view used |
| `car_half_width` | 0.155 m | Vehicle half-width |
| `bubble_radius` | 0.30 m | Safety bubble |
| `line_max_offset` | 0.5 m | Maximum inside-line offset |
| `ttc_aeb` | 0.20 s | Emergency braking threshold |
| `race_laps` | 10 | Race laps |

## 8. Project Structure

```text
F1Tenth-Repository/
│
├── README.md
├── README_es.md
├── iniciar_sepang.sh
├── evidencia/
│   └── PROYECTOPARTE1.webm
│
└── src/
    ├── controllers/
    │   ├── controllers/
    │   │   ├── gap_node.py
    │   │   └── lap_counter.py
    │   ├── package.xml
    │   └── setup.py
    │
    └── f1tenth_gym_ros/
        ├── config/
        ├── launch/
        └── maps/
            ├── Sepang_map.png
            └── Sepang_map.yaml
```

## 9. Building

```bash
source /opt/ros/humble/setup.bash
cd ~/F1Tenth-Repository
colcon build --packages-select controllers f1tenth_gym_ros --symlink-install
source install/setup.bash
```

## 10. Running

```bash
source /opt/ros/humble/setup.bash
cd ~/F1Tenth-Repository
source install/setup.bash
./iniciar_sepang.sh
```

To stop the simulation, press **Ctrl+C once** and wait for the shutdown to finish before launching again. If processes from a previous run are still alive, they can be closed with:

```bash
pkill -f gym_bridge; pkill -f reactive_node; pkill -f lap_counter; pkill -f rviz2
```

Logs from each run are saved in `~/f1tenth_logs/`.

## 11. Results

### Final run (10 consecutive laps)

| Metric | Value |
|---|---|
| Best lap | **49.00 s** |
| Average flying lap | 49.23 s |
| Consistency (lap-to-lap deviation) | 0.11 s |
| Minimum wall clearance | 0.32 – 0.43 m |
| AEB activations | **0** |
| Collisions | **0** |
| Distance per lap | ~486 m |

### Evaluated and discarded alternatives
- **Higher hairpin speed (6.5 m/s):** saved ~0.2 s per lap, but in one lap the clearance dropped to 0.22 m and the AEB was triggered.
- **Maximum speed above 12 m/s:** increased lap time, because stable braking distances become longer.
- **Iterative learning of the speed profile:** the vehicle learned lap by lap where to brake earlier and where to accelerate. It reached 48.64 s, but increased lap-to-lap variability (0.48 s) and caused a near-collision, so reliability was prioritized.

## 12. Conclusions

- Follow the Gap makes it possible to drive the circuit using only the LiDAR, without a map or predefined trajectory.
- Lap time was not limited by straight-line speed but by **braking and cornering stability**. Incorporating the vehicle's dynamic model (stable braking, understeer compensation) is what made it possible to reach a 49.00 s lap time while keeping the vehicle stable.
- The recorded metrics (minimum clearance, AEB, consistency) made it possible to justify every change with data and to discard improvements that compromised safety.
- Future work includes a global planner with localization and model predictive control (MPC) to optimize the full racing line.
