# Fix for Python 3.10+ compatibility
import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping

from dronekit import connect, VehicleMode, Command
from pymavlink import mavutil
import matplotlib.pyplot as plt
import math
import time

# 1. Waypoint Configuration (Improved)
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
            'alt': 10 + i * 2
        })
    return waypoints

# 2. Mission Planning Logic (Unchanged)
def add_mission(vehicle, waypoints):
    cmds = vehicle.commands
    cmds.clear()
    for wp in waypoints:
        cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                    wp['lat'], wp['lon'], wp['alt'])
        cmds.add(cmd)
    land_cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0,
                    waypoints[-1]['lat'], waypoints[-1]['lon'], 0)
    cmds.add(land_cmd)
    cmds.upload()

# 3. Fixed Perpendicular Waypoint Calculation
def calculate_perpendicular(waypoints, wp_index=10, distance=100):
    """Accurate perpendicular waypoint calculation in meters"""
    current = waypoints[wp_index]
    next_wp = waypoints[wp_index+1]

    # Convert degrees to meters
    dx_m = (next_wp['lon'] - current['lon']) * 111320 * math.cos(math.radians(current['lat']))
    dy_m = (next_wp['lat'] - current['lat']) * 111320

    # Calculate perpendicular vector
    perp_dx_m = -dy_m
    perp_dy_m = dx_m

    # Normalize and scale to desired distance
    length_m = math.hypot(perp_dx_m, perp_dy_m)
    scale = distance / length_m if length_m != 0 else 0

    # Convert back to degrees
    delta_lon = (perp_dx_m * scale) / (111320 * math.cos(math.radians(current['lat'])))
    delta_lat = (perp_dy_m * scale) / 111320

    return {
        'lat': current['lat'] + delta_lat,
        'lon': current['lon'] + delta_lon,
        'alt': current['alt']
    }

# 4. Mission Monitoring Class (Improved)
class MissionMonitor:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.remaining_distance = 0
        self.speed = 10  # m/s
        self.actual_path = []
        self.last_update = time.time()
        self.update_interval = 5
        
        self.vehicle.add_attribute_listener('location', self.location_callback)

    def location_callback(self, vehicle, name, location):
        if not vehicle.armed:
            return

        self.actual_path.append((location.global_frame.lat, location.global_frame.lon))
        
        # Calculate distance to next waypoint only
        next_wp = vehicle.commands.next
        if next_wp < vehicle.commands.count:
            cmd = vehicle.commands[next_wp]
            if cmd.command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
                self.remaining_distance = self.haversine(
                    location.global_frame.lat,
                    location.global_frame.lon,
                    cmd.x,
                    cmd.y
                )
        
        if time.time() - self.last_update >= self.update_interval:
            print(f"Remaining: {self.remaining_distance:.1f}m | ETA: {self.remaining_distance/self.speed:.1f}s")
            self.last_update = time.time()

    def haversine(self, lat1, lon1, lat2, lon2):
        """Accurate distance calculation between two points"""
        R = 6371000  # Earth radius in meters
        φ1 = math.radians(lat1)
        φ2 = math.radians(lat2)
        Δφ = math.radians(lat2 - lat1)
        Δλ = math.radians(lon2 - lon1)

        a = math.sin(Δφ/2)**2 + math.cos(φ1)*math.cos(φ2)*math.sin(Δλ/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        return R * c

# 5. Visualization Function (Unchanged)
def plot_mission_2d(original_wps, modified_wps, actual_path=None):
    plt.figure(figsize=(12, 8))
    plt.plot([wp['lon'] for wp in original_wps], [wp['lat'] for wp in original_wps], 'b--', label='Original Plan')
    if len(modified_wps) > len(original_wps):
        plt.plot([wp['lon'] for wp in modified_wps], [wp['lat'] for wp in modified_wps], 'r-', label='Modified Path')
    if actual_path:
        plt.plot([p[1] for p in actual_path], [p[0] for p in actual_path], 'g-', label='Actual Path')
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.legend()
    plt.grid(True)
    plt.show()

# Main Execution with Error Handling
if __name__ == "__main__":
    try:
        print("Connecting to vehicle...")
        vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True, timeout=60)
        
        # Bengaluru coordinates
        base_lat = 12.979327
        base_lon = 77.590920
        
        # Generate waypoints with 100m spacing
        original_waypoints = create_waypoints(base_lat, base_lon, distance_between_points=100)
        
        # Add perpendicular waypoint
        perp_wp = calculate_perpendicular(original_waypoints)
        modified_waypoints = original_waypoints[:11] + [perp_wp] + original_waypoints[11:]
        
        # Upload mission
        add_mission(vehicle, modified_waypoints)
        vehicle.commands.wait_ready()
        
        # Arming sequence
        print("Arming vehicle...")
        vehicle.mode = VehicleMode("GUIDED")
        vehicle.armed = True
        while not vehicle.armed:
            time.sleep(1)
        vehicle.mode = VehicleMode("AUTO")
        
        print("Mission started!")
        monitor = MissionMonitor(vehicle)
        
        # Mission loop
        while vehicle.commands.next < vehicle.commands.count:
            time.sleep(1)
        
        print("Mission complete!")
        plot_mission_2d(original_waypoints, modified_waypoints, monitor.actual_path)
        vehicle.close()
        
    except Exception as e:
        print(f"Error: {e}")
        if 'vehicle' in locals():
            vehicle.close()