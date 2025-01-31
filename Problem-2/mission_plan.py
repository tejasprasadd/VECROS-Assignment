# Fix for Python 3.10+ compatibility
import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping

# Rest of your code
from dronekit import connect, VehicleMode, Command
from pymavlink import mavutil
import matplotlib.pyplot as plt
import math
import time

# 1. Waypoint Configuration
def create_waypoints():
    """Create dictionary of 15 waypoints"""
    waypoints = []
    base_lat = 12.979327228659189 # Starting latitude (example)
    base_lon = 77.59092091365582  # Starting longitude (example)
    
    for i in range(15):
        waypoints.append({
            'lat': base_lat + i*0.0001,  # ~11 meters per degree
            'lon': base_lon + i*0.0001,
            'alt': 10 + i*2  # Altitude increases by 2 meters per waypoint
        })
    return waypoints

# 2. Mission Planning Logic
def add_mission(vehicle, waypoints):
    """Add mission to vehicle"""
    cmds = vehicle.commands
    cmds.clear()
    
    # Add MAV_CMD_NAV_WAYPOINT commands
    for wp in waypoints:
        cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                      mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                      wp['lat'], wp['lon'], wp['alt'])
        cmds.add(cmd)
    
    # Add landing command at last waypoint
    land_cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                       mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0,
                       waypoints[-1]['lat'], waypoints[-1]['lon'], 0)
    cmds.add(land_cmd)
    cmds.upload()

# 3. Perpendicular Waypoint Calculation
def calculate_perpendicular(waypoints, wp_index=10, distance=100):
    """Calculate perpendicular waypoint at specified distance"""
    # Get current and next waypoint
    current = waypoints[wp_index]
    next_wp = waypoints[wp_index+1]
    
    # Calculate direction vector
    dx = next_wp['lon'] - current['lon']
    dy = next_wp['lat'] - current['lat']
    
    # Calculate perpendicular vector (rotate 90 degrees)
    perp_dx = -dy
    perp_dy = dx
    
    # Normalize and scale
    length = math.sqrt(perp_dx**2 + perp_dy**2)
    scale = distance / (111320 * 0.0001)  # Approx meters to degrees
    new_lon = current['lon'] + (perp_dx/length)*scale
    new_lat = current['lat'] + (perp_dy/length)*scale
    
    return {
        'lat': new_lat,
        'lon': new_lon,
        'alt': current['alt']
    }

# 4. Mission Monitoring Class
class MissionMonitor:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.total_distance = 0
        self.remaining_distance = 0
        self.speed = 10  # m/s
        self.actual_path = []  # Stores the actual path traveled
        
        # Register listeners
        self.vehicle.add_attribute_listener('location', self.location_callback)
    
    def location_callback(self, vehicle, name, location):
        """Update mission stats on location change"""
        if len(vehicle.commands) == 0:
            return
        
        # Record actual path
        self.actual_path.append((vehicle.location.global_frame.lat, vehicle.location.global_frame.lon))
        
        # Calculate remaining distance
        self.remaining_distance = 0
        cmds = vehicle.commands
        current_seq = vehicle.commands.next
        
        for cmd in cmds[current_seq:]:
            if cmd.command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
                self.remaining_distance += self.distance_between_points(
                    vehicle.location.global_frame,
                    cmd
                )
        
        # Calculate estimated time
        est_time = self.remaining_distance / self.speed
        
        print(f"Remaining: {self.remaining_distance:.1f}m | ETA: {est_time:.1f}s")
    
    def distance_between_points(self, current, cmd):
        """Calculate distance between two points in meters"""
        dlat = cmd.x - current.lat
        dlon = cmd.y - current.lon
        return math.sqrt((dlat*111320)**2 + (dlon*111320*math.cos(math.radians(current.lat)))**2)

# 5. Plotting Functions
def plot_mission_2d(original_wps, modified_wps, actual_path=None):
    """Plot 2D mission path"""
    plt.figure(figsize=(10, 6))
    
    # Original path
    lon = [wp['lon'] for wp in original_wps]
    lat = [wp['lat'] for wp in original_wps]
    plt.plot(lon, lat, 'b--', label='Original Plan')
    
    # Modified path
    if len(modified_wps) > len(original_wps):
        lon_mod = [wp['lon'] for wp in modified_wps]
        lat_mod = [wp['lat'] for wp in modified_wps]
        plt.plot(lon_mod, lat_mod, 'r-', label='Modified Path')
    
    # Actual path
    if actual_path:
        actual_lon = [point[1] for point in actual_path]
        actual_lat = [point[0] for point in actual_path]
        plt.plot(actual_lon, actual_lat, 'g-', label='Actual Path')
    
    plt.scatter(lon, lat, c='blue')
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.title('2D Mission Path')
    plt.legend()
    plt.grid(True)
    plt.show()

# Main Execution
if __name__ == "__main__":
    # Connect to vehicle
    print("Connecting to vehicle...")
    vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True)
    
    # Create initial mission
    original_waypoints = create_waypoints()
    
    # Add perpendicular waypoint after 10th point
    new_wp = calculate_perpendicular(original_waypoints)
    modified_waypoints = original_waypoints[:11] + [new_wp] + original_waypoints[11:]
    
    # Upload mission
    add_mission(vehicle, modified_waypoints)
    
    # Initialize mission monitoring
    monitor = MissionMonitor(vehicle)
    
    # Arm and launch
    print("Arming vehicle...")
    vehicle.mode = VehicleMode("AUTO")
    vehicle.armed = True
    
    # Wait for arming
    while not vehicle.armed:
        time.sleep(1)
    
    print("Mission started!")
    
    # Wait for mission completion
    while vehicle.commands.next < len(vehicle.commands):
        time.sleep(1)
    
    print("Mission complete!")
    
    # Plot mission in 2D
    plot_mission_2d(original_waypoints, modified_waypoints, monitor.actual_path)
    
    vehicle.close()