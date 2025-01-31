from environment import create_environment
from pathfinding import find_path
from conflict_avoidance import check_conflicts
from visualisation import plot_paths

ROUTES = [
    ((0, 0, 0), (100, 100, 100)),
    ((3, 1, 5), (56, 65, 90)),
    ((8, 2, 7), (13, 18, 28)),
    ((23, 12, 25), (75, 83, 85)),
    ((48, 24, 4), (1, 3, 25)),
    ((100, 0, 0), (0, 100, 100)),
    ((0, 100, 0), (100, 0, 100)),
    ((50, 0, 0), (50, 100, 100))
]

def main():
    # Initialize environment
    grid = create_environment()
    
    # Calculate initial paths
    all_paths = []
    for start, end in ROUTES:
        if path := find_path(start, end, grid):
            all_paths.append(path)
    
    # Resolve conflicts
    safe_paths = check_conflicts(all_paths, speed=1, buffer_radius=2)
    
    # Display results
    print(f"Found {len(safe_paths)} conflict-free paths out of {len(ROUTES)} requested")
    plot_paths(safe_paths)

if __name__ == "__main__":
    main()