# Autonomous Drone Mission Planner with Dynamic Waypoint Insertion

This project is an autonomous drone mission planner that creates a 15-waypoint mission, dynamically inserts a perpendicular waypoint after the 10th point, and provides real-time mission statistics. It also includes 2D and 3D trajectory visualization using Python libraries such as `dronekit`, `pymavlink`, and `matplotlib`.

---

## Table of Contents

1. [Overview](#overview)
2. [Features](#features)
3. [Prerequisites](#prerequisites)
4. [Installation](#installation)

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

## Prerequisites

Before running the project, ensure you have the following installed:

- Python 3.8 or higher.
- Required Python libraries:
  ```bash
  pip install dronekit dronekit-sitl pymavlink matplotlib
  ```
