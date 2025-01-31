"""
3D Pathfinding with Conflict Avoidance 
"""
import numpy as np
from heapq import heappush, heappop
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use('Qt5Agg')

# 1. Create Grid with Random Obstacles

grid_size = 101  # 0-100 in all dimensions
grid = np.zeros((grid_size, grid_size, grid_size))

# Add obstacles to 10% of grid points
obstacle_indices = np.random.choice(
    grid_size**3,
    size=int(0.1 * grid_size**3),
    replace=False
)
x, y, z = np.unravel_index(obstacle_indices, (grid_size, grid_size, grid_size))
grid[x, y, z] = 10  # High weight for obstacles


# 2. A* Pathfinding Algorithm
def find_path(start, end):
    """
    Simplified A* algorithm with 26-direction movement and Chebyshev distance heuristic.
    Returns path as list of points or empty list if no path.
    """
    # Chebyshev distance heuristic (optimal for 26-directional movement)
    def heuristic(a, b):
        return max(abs(a[0]-b[0]), abs(a[1]-b[1]), abs(a[2]-b[2]))

    # Precompute all 26 possible movement directions
    directions = [
        (dx, dy, dz)
        for dx in [-1, 0, 1]
        for dy in [-1, 0, 1]
        for dz in [-1, 0, 1]
        if not (dx == 0 and dy == 0 and dz == 0)  # Exclude no-movement
    ]

    # Priority queue: (priority, node)
    heap = [(0, start)]
    came_from = {}  # Tracks the path
    cost_so_far = {start: 0}  # Tracks the cost to reach each node

    while heap:
        _, current = heappop(heap)  # Get the node with the lowest priority

        if current == end:  # Reached the goal
            # Reconstruct the path
            path = []
            while current != start:
                path.append(current)
                current = came_from[current]
            return [start] + path[::-1]  # Reverse to get start-to-end order

        # Explore all 26 neighbors
        for dx, dy, dz in directions:
            neighbor = (current[0] + dx, current[1] + dy, current[2] + dz)

            # Skip if neighbor is outside the grid
            if any(not (0 <= coord < grid_size) for coord in neighbor):
                continue

            # Calculate new cost to reach the neighbor
            new_cost = cost_so_far[current] + grid[neighbor]

            # Update if this path to the neighbor is better
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + heuristic(neighbor, end)  # f(n) = g(n) + h(n)
                heappush(heap, (priority, neighbor))
                came_from[neighbor] = current

    return []  # No path found



# 3. Conflict Checking (Improved with Buffer)

def check_conflicts(paths, speed=1, buffer_radius=1):  # Added buffer radius
    """
    Conflict detection with buffer zone.
    Returns list of safe paths.
    """
    occupied = {}
    safe_paths = []

    for path in paths:
        conflict = False
        for step, point in enumerate(path):
            time = step / speed

            # Check current point and neighbors (buffer zone)
            for dx in range(-buffer_radius, buffer_radius + 1):
                for dy in range(-buffer_radius, buffer_radius + 1):
                    for dz in range(-buffer_radius, buffer_radius + 1):
                        check_point = (point[0] + dx, point[1] + dy, point[2] + dz)
                        if all(0 <= coord < grid_size for coord in check_point): # Check bounds!
                            if check_point in occupied:
                                if any(abs(t - time) < 1/speed for t in occupied[check_point]):
                                    conflict = True
                                    break
                if conflict:
                    break
            if conflict:
                break

        if not conflict:
            # Reserve the points (including buffer zone)
            for step, point in enumerate(path):
                time = step / speed
                for dx in range(-buffer_radius, buffer_radius + 1):
                    for dy in range(-buffer_radius, buffer_radius + 1):
                        for dz in range(-buffer_radius, buffer_radius + 1):
                            reserve_point = (point[0] + dx, point[1] + dy, point[2] + dz)
                            if all(0 <= coord < grid_size for coord in reserve_point): # Check bounds!
                                if reserve_point not in occupied:
                                    occupied[reserve_point] = []
                                occupied[reserve_point].append(time)
            safe_paths.append(path)
    return safe_paths



# 4. Visualization 

def plot_paths(paths):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    colors = ['red', 'blue', 'green', 'purple', 'orange','pink','yellow','black']  # Add more colors
    for i, path in enumerate(paths):
        if not path:
            continue
        x, y, z = zip(*path)
        ax.plot(x, y, z, color=colors[i % len(colors)], marker='o', markersize=2, label=f'Path {i+1}')  # Cycle colors

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    plt.show()


# 5. Main Execution

if __name__ == "__main__":
    routes = [
        ((0, 0, 0), (100, 100, 100)),
        ((3, 1, 5), (56, 65, 90)),
        ((8, 2, 7), (13, 18, 28)),
        ((23, 12, 25), (75, 83,85)),
        ((48, 24, 4), (1, 3, 25)),
        ((100, 0, 0), (0, 100, 100)),
        ((0, 100, 0), (100, 0, 100)),  # Added a third path
        ((50, 0, 0), (50, 100, 100)) # Added a fourth path
    ]

    all_paths = []
    for start, end in routes:
        path = find_path(start, end)
        if path:
            all_paths.append(path)

    safe_paths = check_conflicts(all_paths, speed=1, buffer_radius=2)  # Increased buffer radius

    print(f"Found {len(safe_paths)} conflict-free paths")
    plot_paths(safe_paths)