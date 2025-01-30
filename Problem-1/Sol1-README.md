# 3D Pathfinding with Conflict Avoidance

Welcome to the **3D Pathfinding with Conflict Avoidance** project! This project simulates and visualizes the movement of multiple agents (e.g., drones, robots) in a 3D environment, navigating obstacles and avoiding collisions. This README provides a detailed guide to the project.

---

## Table of Contents

1. [Introduction](#introduction)
2. [How It Works](#how-it-works)
   - [2.1 3D Grid Environment](#21-3d-grid-environment)
   - [2.2 Pathfinding with A\*](#22-pathfinding-with-a)
   - [2.3 Conflict Avoidance](#23-conflict-avoidance)
   - [2.4 Visualization](#24-visualization)
3. [Key Features](#key-features)
4. [Installation](#installation)
5. [Usage](#usage)
6. [Visualization Details](#visualization-details)
7. [Applications](#applications)
8. [Contributing](#contributing)
9. [License](#license)

---

## 1. Introduction <a name="introduction"></a>

This project simulates 3D pathfinding for multiple agents (e.g., drones) within a grid-based 3D environment. The agents must navigate from their designated starting points to their respective destinations, adhering to the following constraints:

- **Obstacle Avoidance:** Agents must avoid static obstacles present in the environment (e.g., buildings, restricted areas).
- **Collision Avoidance:** Agents must not collide with each other during their movement.

The project utilizes the A\* search algorithm for individual agent path planning and incorporates a conflict detection and resolution mechanism to prevent inter-agent collisions. The resulting agent paths are visualized using an interactive 3D plot.

---

## 2. How It Works <a name="how-it-works"></a>

The project's functionality can be broken down into the following key components:

### 2.1 3D Grid Environment <a name="21-3d-grid-environment"></a>

The environment is represented as a 3D grid, specifically a NumPy array.

- **Grid Dimensions:** The default grid dimensions are 101x101x101, effectively spanning coordinates from (0, 0, 0) to (100, 100, 100) inclusive. Each cell in the grid represents a discrete location in the 3D space.
- **Obstacles:** A certain percentage (default 10%) of the grid points are randomly designated as obstacles. These obstacles are assigned a higher "cost" value (10) compared to free space (0). This cost is considered by the A\* algorithm to discourage paths from passing through obstacles. The obstacle locations are determined randomly at the start of the simulation.

### 2.2 Pathfinding with A\* <a name="22-pathfinding-with-a"></a>

Each agent individually plans its path from its starting point to its goal using the A\* search algorithm.

- **Chebyshev Distance Heuristic:** The A\* algorithm uses the Chebyshev distance as its heuristic function. The Chebyshev distance is appropriate for movement that allows all 26 possible directions (including diagonals in 3D). It calculates the maximum absolute difference in any single coordinate between two points.
- **26-Directional Movement:** Agents are allowed to move in 26 directions from any given cell (all possible combinations of dx, dy, and dz being -1, 0, or 1, excluding no movement).
- **Cost Calculation:** The cost to move from one cell to a neighboring cell is 1 for free space. The cost to move into an obstacle cell is significantly higher (10), making the algorithm prefer paths that avoid obstacles.

### 2.3 Conflict Avoidance <a name="23-conflict-avoidance"></a>

To prevent collisions between agents, a conflict avoidance mechanism is implemented.

- **Agent Speed:** Agents are assumed to move at a constant speed (1 grid cell per unit of time).
- **Buffer Zone:** A buffer zone is defined around each agent. This is a region around the agent's current location. The default buffer radius is 2 grid cells. This means that two agents cannot occupy cells within each other's buffer zones at the same time step. This helps to prevent near misses.
- **Conflict Detection:** The conflict detection system checks if multiple agents are trying to occupy the same grid cell (or cells within each other's buffer zones) at the same (or very close in time) time step.
- **Conflict Resolution:** If a conflict is detected, currently, one of the agents involved is stopped, and its path is replanned later. More sophisticated conflict resolution methods could be implemented, such as prioritized planning or cooperative pathfinding.

### 2.4 Visualization <a name="24-visualization"></a>

The generated paths for each agent are visualized in 3D using Matplotlib.

- **3D Plot:** The environment grid and the planned paths are displayed in a 3D plot.
- **Agent Paths:** Each agent's path is plotted as a line in a distinct color, making it easy to distinguish between different agent trajectories.
- **Obstacles:** Obstacles are not explicitly shown in the visualization, their effect is that the paths will avoid them.

---

## 3. Key Features <a name="key-features"></a>

- **3D Grid Environment:** Simulates a realistic 3D space for agent navigation.
- **A\* Pathfinding with Chebyshev Heuristic:** Efficiently finds near-optimal paths considering 26-directional movement.
- **Conflict Detection and Avoidance:** Prevents collisions between multiple agents using buffer zones.
- **Interactive 3D Visualization:** Provides a clear visual representation of the agent paths.
- **Flexible Configuration:** Parameters like grid size, obstacle density, buffer radius, and the number of agents can be adjusted.

---

## 4. Installation <a name="installation"></a>

### Requirements

- Python 3.8 or higher
- Required Python Libraries:
  - `numpy`
  - `matplotlib`
  - `heapq` (usually included with Python)

### Steps

1. **Clone the Repository:**
   ```bash
   git clone [https://github.com/tejasprasadd/VECROS-Assignment.git](https://github.com/tejasprasadd/VECROS-Assignment.git)  # Replace with your repository URL
   cd VECROS-Assignment
   ```
