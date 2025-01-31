# Autonomous Drone Mission Planner with Dynamic Waypoint Insertion

This project is an autonomous drone mission planner that creates a 15-waypoint mission, dynamically inserts a perpendicular waypoint after the 10th point, and provides real-time mission statistics. It also includes 2D and 3D trajectory visualization using Python libraries such as `dronekit`, `pymavlink`, and `matplotlib`.

---

## Table of Contents

1. [Overview](#overview)
2. [Features](#features)
3. [Usage](#Usage)
4. [Prerequisites](#prerequisites)
5. [Installation](#installation)

---

## Overview

This project demonstrates how to plan and execute an autonomous drone mission using Python. It includes:

- Creation of a 15-waypoint mission.
- Dynamic insertion of a perpendicular waypoint after the 10th waypoint.
- Real-time mission monitoring and statistics.
- 2D and 3D visualization of the mission trajectory.

The project uses `dronekit` for high-level mission planning and `pymavlink` for low-level MAVLink communication. It also includes a simulated drone environment using `dronekit-sitl` for testing without a physical drone.

---

## Features

- **Waypoint Planning**: Generates 15 waypoints with increasing altitude.
- **Dynamic Waypoint Insertion**: Inserts a perpendicular waypoint after the 10th waypoint.
- **Real-Time Monitoring**: Tracks mission progress, remaining distance, and estimated time of arrival (ETA).
- **2D and 3D Visualization**: Plots the original, modified, and actual paths using `matplotlib`.
- **Simulation Support**: Uses `dronekit-sitl` for testing without a physical drone.

---

## Usage

The mission planner requires a running SITL (Software In The Loop) simulator.

### 1. Start the Simulator

Open a terminal window and run the following command:

```bash
dronekit-sitl copter-3.3 --home=12.979327,77.590920,584,353
```

This command will:

Start a simulated Copter (version 3.3).
Set the home location to Bengaluru (12.979327°N, 77.590920°E) with an altitude of 584 meters and a yaw of 353 degrees. (Note: The yaw parameter might not be strictly necessary for this mission)
Establish a local connection at tcp:127.0.0.1:5760.
Important: Keep this terminal window open while running the mission planner. The simulator must be running for the script to connect.

2. Run the Mission Planner
   Open a second terminal window (preferably within your activated virtual environment if you created one) and execute the following command:

```bash
python mission_plan.py
```

The script will:

Connect to the simulator
Generate and upload the mission
Arm and launch the drone
Display real-time updates in the terminal
Show a trajectory plot after mission completion

## Prerequisites

Before running the project, ensure you have the following installed:

- Python 3.8 or higher.
- Required Python libraries:
  ```bash
  pip install dronekit dronekit-sitl pymavlink matplotlib
  ```

```

```
