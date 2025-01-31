import numpy as np

grid_size = 101  # 0-100 in all dimensions

def create_environment():
    """Create 3D grid environment with random obstacles"""
    grid = np.zeros((grid_size, grid_size, grid_size))
    
    # Add obstacles to 10% of grid points
    obstacle_indices = np.random.choice(
        grid_size**3,
        size=int(0.1 * grid_size**3),
        replace=False
    )
    x, y, z = np.unravel_index(obstacle_indices, (grid_size, grid_size, grid_size))
    grid[x, y, z] = 10  # High weight for obstacles
    return grid