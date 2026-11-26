# F1TENTH Project – Part 2: Race with obstacles and overtaking

Second part of the F1TENTH project on the **Sepang** circuit (ROS 2 Humble, F1TENTH Gym simulator). It uses the same **Follow the Gap** controller from part 1, extended to drive on a track with **5 static obstacles** and **a second vehicle** that starts 5 m ahead and must be overtaken. Both conditions are tested **together**: two vehicles and five obstacles at the same time, for 10 consecutive laps.

**Final result (10 consecutive laps per vehicle):**

| Vehicle | Laps | Best lap | Average | Collisions | AEB |
|---|---|---|---|---|---|
| Main | 10/10 | **49.79 s** | 50.71 s | **0** | 0 |
| Opponent | 10/10 | 58.11 s | 58.27 s | **0** | 0 |

**Videos:**
- Part 1 (free track): https://youtu.be/4MpEtsWnOw8
- Part 2 (two vehicles and obstacles): https://youtu.be/fDcNhzK-DyY
  
**Run data (rosbag converted to CSV):**
- Part 1 CSV: [Drive folder](https://drive.google.com/drive/folders/1DT7dDvZwqSaI45-Ijq5tW_a5sXRcozir?usp=drive_link)
- Part 2 CSV: [Drive folder](https://drive.google.com/drive/folders/1KrcHtUalAmRdsR-9ACsPp33oC9yzHlQM?usp=drive_link)
- Full folder (original rosbag + CSV): [Drive](https://drive.google.com/drive/folders/1efkPdL0n2lDer6oQz9Afr0IN6jOFYXye?usp=sharing)

*Versión en español: [README_parte2_es.md](README_parte2_es.md) · Part 1: [README.md](README.md)*

---

## 1. Summary

- **A single script** launches the whole test: `./iniciar_sepang_parte2.sh`. It opens the simulator with both vehicles and the obstacle map, RViz, two lap counters, both controllers and the rosbag recording.
- **Both vehicles use the same algorithm** (`gap_node.py`). The opponent drives slower (8.7 m/s max) so it can be caught and overtaken.
- Each vehicle **stops by itself after 10 laps**. When both are done, the script closes everything, prints a summary table and converts the recording to CSV.
- In RViz, each vehicle carries a text with its lap number and times. When the main vehicle overtakes, an **"REBASE!"** banner appears and the position changes (P2 → P1).

## 2. How it works

### 2.1. Static obstacles
- `crear_mapa_obstaculos.py` takes `Sepang_map.png` and draws 5 squares of 0.40 m on the vehicle's normal path, shifted to one side so the vehicle has to avoid them. The result is `Sepang_obstaculos.png/.yaml`.
- The LiDAR sees obstacles just like a wall, so Follow the Gap avoids them with no change to its logic.
- When a small object (an obstacle or the other vehicle) is detected, an **extra margin** is added to the obstacle inflation (`obs_margin_extra`). On the rest of the track the margin is the same as in part 1, so no time is lost.
- To avoid braking too much next to obstacles, braking considers the walls and only what is inside the corridor the vehicle drives through (`obs_brake_walls`). If the other vehicle is near, it brakes for everything it sees.

| # | Position (x, y) [m] | Location |
|---|---|---|
| 1 | (0.17, −9.44) | Long straight, ~66 m after the start |
| 2 | (−9.70, −31.77) | ~160 m |
| 3 | (20.10, −10.87) | ~228 m |
| 4 | (−25.59, 36.90) | ~362 m |
| 5 | (−30.84, −2.43) | Main straight, ~30 m before the line |

### 2.2. The other vehicle as a dynamic obstacle
- **Opponent:** the same `gap_node` with its own topics (`/opp_scan`, `/opp_drive`, `/opp_racecar/odom`), 8.7 m/s max speed and kept to the right (`lane_bias = −0.4`). It starts at (5.0, 0.0), 5 m ahead.
- **Main vehicle:** sees the opponent with the LiDAR as one more obstacle. It also reads the **opponent's odometry** (`/opp_racecar/odom`), which in the simulator gives its position and speed, so it does not hit it (`safe_rival`):
  - **In corners and obstacle zones** (less than 12 m free ahead), if the opponent is ahead, the main vehicle stays about 3 m behind and does not go faster than it. While following, the LiDAR points that fall on the opponent are not used to choose the gap, so it does not block the passage between obstacles.
  - **On straights**, Follow the Gap sees the opponent and looks for the free gap beside it to overtake.
  - If the opponent is right ahead in the same lane, the main vehicle limits its speed so it does not run into it (`ot_speed_only`).
- With this, the main vehicle overtakes the opponent at the start and later laps it during the race with no contact. The lap where it laps the opponent is slower (55.6 s), because it waits behind it through the corner section.

### 2.3. Rosbag recording and CSV
- `/scan`, `/ego_racecar/odom`, `/drive`, `/opp_scan`, `/opp_racecar/odom` and `/opp_drive` are recorded to `~/f1tenth_bags/PARTE2_<date>/`.
- When the race ends, `bag_a_csv.py` converts each topic to a CSV in the recording's `csv/` folder:

| File | Columns |
|---|---|
| `ego_racecar_odom.csv`, `opp_racecar_odom.csv` | time, x, y, heading, linear velocity x/y, angular velocity z |
| `drive.csv`, `opp_drive.csv` | time, commanded speed, steering angle |
| `scan.csv`, `opp_scan.csv` | time, start angle, increment, r0 … rN (LiDAR ranges) |

- The conversion takes a few minutes because the LiDAR has a lot of data. **Do not press Ctrl+C while it converts.**
- If the race is stopped early with Ctrl+C, the recording is left unconverted and can be converted later by hand (see section 4).

## 3. Part 2 files

| File | Purpose |
|---|---|
| `iniciar_sepang_parte2.sh` | Launches the full test (two vehicles + obstacles) |
| `crear_mapa_obstaculos.py` | Creates the map with the 5 obstacles |
| `resumen_carrera.py` | Waits for the 10 laps of each vehicle and prints the summary table |
| `bag_a_csv.py` | Converts a rosbag recording to CSV |
| `src/controllers/controllers/gap_node.py` | Follow the Gap controller (the same for both vehicles) |
| `src/f1tenth_gym_ros/config/sim_2robots.yaml` | Simulator with 2 vehicles and the obstacle map |
| `src/f1tenth_gym_ros/maps/Sepang_obstaculos.png/.yaml` | Obstacle map |

All new controller features are **off by default** and are turned on with parameters in `iniciar_sepang_parte2.sh`. That is why part 1 (`./iniciar_sepang.sh`) works the same as before.

## 4. Build and run

**Build:**
```bash
source /opt/ros/humble/setup.bash
cd ~/F1Tenth-Repository
colcon build --packages-select controllers f1tenth_gym_ros --symlink-install
source install/setup.bash
```

**Create the obstacle map** (first time only):
```bash
source /opt/ros/humble/setup.bash
cd ~/F1Tenth-Repository
python3 crear_mapa_obstaculos.py
```

**Run part 2:**
```bash
source /opt/ros/humble/setup.bash
cd ~/F1Tenth-Repository
source install/setup.bash
./iniciar_sepang_parte2.sh
```
The script ends by itself. The race finished correctly when you see the summary table with **"Carrera terminada."**, then **"CSV guardados en ..."** with the 6 files, and the terminal prompt comes back.

**If processes from a previous run are still open:**
```bash
pkill -f gym_bridge; pkill -f gap_node; pkill -f lap_counter; pkill -f rviz2; pkill -f "ros2 bag"
```

**Convert a recording to CSV by hand:**
```bash
source /opt/ros/humble/setup.bash
cd ~/F1Tenth-Repository
python3 bag_a_csv.py ~/f1tenth_bags/PARTE2_<date>
```

**Print the summary table again:**
```bash
python3 resumen_carrera.py --tags PRINCIPAL OPONENTE --laps 10 --since 2026-11-26T17:00:00
```

## 5. Main parameters

| Parameter | Value | Description |
|---|---|---|
| `race_laps` | 10 | Laps before stopping |
| `obs_margin_extra` | 0.15 m (main) / 0.08 m (opponent) | Extra margin near small objects |
| `obs_brake_walls` | true (main) | Braking ignores obstacles outside the corridor |
| `obs_angle_deg` | 100° | Angle on each side where obstacles are searched |
| `safe_rival` | true (main) | Uses the opponent's odometry to avoid hitting it |
| `sr_straight` | 12 m | Minimum free straight ahead to overtake |
| `sr_gap_twisty` | 3 m | Distance behind the opponent in corners and obstacles |
| `sr_lat` | 0.8 m | Lateral separation below which both are in the same lane |
| `ot_speed_only` | true (both) | Limits speed behind the other vehicle in the same lane |
| `max_speed` | 12.0 m/s (main) / 8.7 m/s (opponent) | Maximum speed |
| `lane_bias` | −0.4 (opponent) | Lateral offset of the opponent |

## 6. Results

| Metric | Main | Opponent |
|---|---|---|
| Laps completed | **10/10** | **10/10** |
| Best lap | **49.79 s** | 58.11 s |
| Average (without lap 1) | 50.71 s | 58.27 s |
| Lap-to-lap deviation | 1.85 s | 0.17 s |
| Top speed | 11.96 m/s | 8.70 m/s |
| Minimum clearance | 0.255 m | 0.254 m |
| AEB activations | **0** | **0** |
| Reversals | 0 | 0 |
| Collisions | **0** | **0** |
| Total time | 509.1 s | 583.5 s |

- The main vehicle's best lap (49.79 s) is only 0.7 s slower than part 1 without obstacles (49.09 s).
- The main vehicle's deviation is higher because in one lap (lap 8) it waits behind the opponent to lap it safely.

### Problems found and solved
- **Safety margin on the whole track:** made the vehicle crash in the hairpin. It was replaced by an extra margin applied only near small objects.
- **Obstacle with two almost equal passages:** the vehicle hesitated between both sides. Obstacle 3 was moved 0.3 m to leave a single clear passage.
- **Too much braking next to obstacles:** braking now considers the walls and the vehicle's corridor, not the obstacles on the sides. This lowered the lap time by ~1 s.
- **Contact with the opponent next to obstacles:** the LiDAR sometimes did not recognize the opponent as a vehicle, and it blocked the passage between obstacles. This was solved with its odometry: in corners and obstacles the main vehicle stays behind, and it overtakes only on straights.

## 7. Conclusions

- Follow the Gap handles static obstacles with no change to its logic: for the LiDAR, obstacles are part of the track.
- Applying the extra margin **only near small objects** keeps almost the same lap time as part 1.
- A moving vehicle is harder than a static obstacle, because it changes position and can block the only free passage. Overtaking only on straights and waiting behind in corners avoids contact.
- The automatic summary table and the CSV recording make it possible to compare runs with the same metrics and to justify each change with data.
- Future work: detect and predict the other vehicle's path using only the LiDAR, without its odometry.
