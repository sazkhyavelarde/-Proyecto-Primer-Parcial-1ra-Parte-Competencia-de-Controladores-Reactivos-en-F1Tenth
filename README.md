# F1TENTH Project – Reactive Controller Competition

## 1. Project Description

This project focuses on the implementation of a reactive controller for an autonomous F1TENTH vehicle using ROS 2 Humble and the F1TENTH Gym simulator.

The main objective is to enable the vehicle to navigate the Sepang circuit autonomously by processing measurements from a LiDAR sensor. For this purpose, the **Follow the Gap** algorithm was implemented to identify free spaces and select a suitable driving direction based on the obstacles detected in the environment.

In addition to the controller, a lap counter node was implemented to record lap times during the simulations. This makes it possible to evaluate the vehicle's performance and compare the results obtained from different tests.

## 2. Development Tools

The following tools were used during the development of the project:

- **Ubuntu:** operating system used to run the simulation environment.
- **ROS 2 Humble:** framework used for communication between the system nodes.
- **Python:** programming language used to develop the controllers.
- **F1TENTH Gym:** simulator used to evaluate the vehicle's behavior.
- **RViz:** tool used to visualize the vehicle, the map, and information about the environment.
- **Visual Studio Code:** code editor used to develop and modify the project files.

The selected circuit for testing was **Sepang**.

## 3. Follow the Gap Controller

The main controller is implemented in the `gap_node.py` file and uses the Follow the Gap algorithm to determine the vehicle's driving direction.

The general operation of the controller is described below.

### 3.1. LiDAR Data Reception

The vehicle receives LiDAR measurements through the `/scan` topic.

These measurements represent the distances between the vehicle and obstacles in its surroundings.

### 3.2. Data Processing

Once the measurements are received, the controller analyzes the available distances to identify areas where the vehicle can move.

Safety margins around obstacles are also considered to reduce the risk of collisions.

### 3.3. Free Space Detection

The algorithm searches for free spaces between the detected obstacles.

Based on these spaces, a target direction is selected to allow the vehicle to continue moving along the track.

### 3.4. Steering Calculation

After selecting the target point, the controller calculates the steering angle required to guide the vehicle toward the available free space.

The steering angle is adjusted according to the track conditions and the vehicle's response.

### 3.5. Speed Control

The vehicle's speed does not remain constant throughout the circuit.

The controller adjusts the speed depending on the surrounding conditions. Higher speeds can be reached on straight sections, while speed is reduced in curves to improve stability.

Braking conditions are also considered when the available space is insufficient for safe movement.

### 3.6. Command Publication

Finally, the controller publishes steering and speed commands through the `/drive` topic.

This allows the vehicle to continuously adjust its movement based on LiDAR measurements.

## 4. Lap Counter

An additional node called `lap_counter.py` was implemented to evaluate the controller's performance.

This node uses odometry information published through the `/ego_racecar/odom` topic to track vehicle movement and detect when the vehicle crosses a configured finish line.

Its main functions include:

- Counting completed laps.
- Recording individual lap times.
- Identifying the fastest lap.
- Calculating average and maximum speed.
- Dividing each lap into three sectors to record partial times.
- Calculating lap statistics.
- Saving results in CSV files for further analysis.

The finish line location must be configured according to the selected circuit.

## 5. Project Structure

The main project files are organized as follows:

```text
F1Tenth-Repository/
│
├── README.md
├── README_ES.md
├── iniciar_sepang.sh
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

The `gap_node.py` file contains the reactive controller, while `lap_counter.py` is responsible for recording lap results.

The `iniciar_sepang.sh` file is the script used to start the simulation.

## 6. Building the Project

To build the ROS 2 packages, open an Ubuntu terminal and execute:

```bash
source /opt/ros/humble/setup.bash
cd ~/F1Tenth-Repository
colcon build --packages-select controllers f1tenth_gym_ros --symlink-install
source install/setup.bash
```

## 7. Running the Simulation

Once the project has been built, the simulation can be started using:

```bash
source /opt/ros/humble/setup.bash
cd ~/F1Tenth-Repository
source install/setup.bash
./iniciar_sepang.sh
```

The script starts the simulation according to the project configuration.

Both the controller and the lap counter must be running to complete the circuit and record results. If the script does not launch one of these nodes, it must be started separately.

## 8. Performance Evaluation

The controller's performance is mainly evaluated through lap times and vehicle behavior during the simulation.

The evaluation considers the ability to complete the circuit, stability in curves, achieved speed, and response to obstacles.

The records generated by the lap counter make it possible to compare different runs and identify potential improvements in the controller configuration.

## 9. Conclusions

The implementation of the Follow the Gap algorithm allows the F1TENTH vehicle to navigate using LiDAR measurements without requiring a predefined complete trajectory.

Adjusting speed and steering according to the surrounding conditions makes it possible to evaluate the vehicle's behavior in different sections of the Sepang circuit.

Additionally, the lap counter provides useful information for recording and analyzing results, making it easier to compare the controller's performance across different simulation runs.