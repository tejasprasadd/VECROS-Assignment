from environment import grid_size

def check_conflicts(paths, speed=1, buffer_radius=1):
    """Conflict detection with spatial and temporal buffer"""
    occupied = {}
    safe_paths = []

    for path in paths:
        conflict = False
        for step, point in enumerate(path):
            time = step / speed
            for dx in range(-buffer_radius, buffer_radius+1):
                for dy in range(-buffer_radius, buffer_radius+1):
                    for dz in range(-buffer_radius, buffer_radius+1):
                        check_point = (point[0]+dx, point[1]+dy, point[2]+dz)
                        if all(0 <= coord < grid_size for coord in check_point):
                            if check_point in occupied:
                                if any(abs(t-time) < 1/speed for t in occupied[check_point]):
                                    conflict = True
                                    break
                    if conflict: break
                if conflict: break
            if conflict: break

        if not conflict:
            for step, point in enumerate(path):
                time = step / speed
                for dx in range(-buffer_radius, buffer_radius+1):
                    for dy in range(-buffer_radius, buffer_radius+1):
                        for dz in range(-buffer_radius, buffer_radius+1):
                            reserve_point = (point[0]+dx, point[1]+dy, point[2]+dz)
                            if all(0 <= coord < grid_size for coord in reserve_point):
                                if reserve_point not in occupied:
                                    occupied[reserve_point] = []
                                occupied[reserve_point].append(time)
            safe_paths.append(path)
    return safe_paths