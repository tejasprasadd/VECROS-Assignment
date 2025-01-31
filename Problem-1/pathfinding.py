from heapq import heappush, heappop

def find_path(start, end, grid):
    """A* algorithm with 26-direction movement and Chebyshev distance heuristic"""
    def heuristic(a, b):
        return max(abs(a[0]-b[0]), abs(a[1]-b[1]), abs(a[2]-b[2]))

    directions = [
        (dx, dy, dz)
        for dx in [-1, 0, 1]
        for dy in [-1, 0, 1]
        for dz in [-1, 0, 1]
        if not (dx == 0 and dy == 0 and dz == 0)
    ]

    heap = [(0, start)]
    came_from = {}
    cost_so_far = {start: 0}

    while heap:
        _, current = heappop(heap)

        if current == end:
            path = []
            while current != start:
                path.append(current)
                current = came_from[current]
            return [start] + path[::-1]

        for dx, dy, dz in directions:
            neighbor = (current[0]+dx, current[1]+dy, current[2]+dz)
            
            if any(not (0 <= coord < len(grid)) for coord in neighbor):
                continue

            new_cost = cost_so_far[current] + grid[neighbor]
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + heuristic(neighbor, end)
                heappush(heap, (priority, neighbor))
                came_from[neighbor] = current

    return []