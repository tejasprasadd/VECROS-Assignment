import math

def create_waypoints(base_lat, base_lon, num_waypoints=15, distance_between_points=100):
    """Create waypoints with accurate meter-to-degree conversion"""
    waypoints = []
    for i in range(num_waypoints):
        # Convert meters to degrees (latitude)
        delta_lat = distance_between_points / 111320
        
        # Convert meters to degrees (longitude) using current latitude
        delta_lon = distance_between_points / (111320 * math.cos(math.radians(base_lat)))
        
        waypoints.append({
            'lat': base_lat + i * delta_lat,
            'lon': base_lon + i * delta_lon,
            'alt': 10 + i * 5  # Increased altitude step for better visibility
        })
    return waypoints

def calculate_perpendicular(waypoints, wp_index=10, distance=100):
    """Enhanced perpendicular waypoint calculation"""
    current = waypoints[wp_index]
    next_wp = waypoints[wp_index+1]

    # Calculate bearing between points
    y = math.sin(math.radians(next_wp['lon'] - current['lon'])) * math.cos(math.radians(next_wp['lat']))
    x = math.cos(math.radians(current['lat'])) * math.sin(math.radians(next_wp['lat'])) - \
        math.sin(math.radians(current['lat'])) * math.cos(math.radians(next_wp['lat'])) * \
        math.cos(math.radians(next_wp['lon'] - current['lon']))
    bearing = math.degrees(math.atan2(y, x))
    
    # Calculate perpendicular direction
    perp_bearing = (bearing + 90) % 360
    
    # Convert distance to degrees
    delta_lat = (distance * math.cos(math.radians(perp_bearing))) / 111320
    delta_lon = (distance * math.sin(math.radians(perp_bearing))) / (111320 * math.cos(math.radians(current['lat'])))
    
    return {
        'lat': current['lat'] + delta_lat,
        'lon': current['lon'] + delta_lon,
        'alt': current['alt']
    }