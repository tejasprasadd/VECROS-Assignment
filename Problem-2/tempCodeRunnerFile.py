# Fix for Python 3.10+ compatibility
import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping

# Mission Planning with DroneKit
from dronekit import connect, VehicleMode, Command
from pymavlink import mavutil
import matplotlib.pyplot as plt
import math
import time

# 1. Waypoint Configuration (Improved)
def create_waypoints(base_lat, base_lon, num_waypoints=15, distance_between_points=100):
    """
    Create a list of waypoint dictionaries with a specified distance between them.
    Distance between points is in meters. The altitude increases gradually.
    """
    waypoints = []
    for i in range(num_waypoints):
        # Convert distance in meters to degrees (approximation)
        delta_lat = distance_between_points / 111320  # meters per degree latitude
        # Adjust delta_lon based on the cosine of the latitude for better approximation
        mid_lat = base_lat + (i * delta_lat) / 2
        delta_lon = distance_between_points / (111320 * math.cos(math.radians(mid_lat)))
        waypoints.append({
            'lat': base_lat + i * delta_lat,
            'lon': base_lon + i * delta_lon,
            'alt': 10 + i * 2  # Altitude increases with each waypoint
        })
    return waypoints

# 2. Mission Planning Logic
def add_mission(vehicle, waypoints):
    """Clear any existing mission and upload new waypoints to the vehicle."""
    cmds = vehicle.commands
    cmds.clear()

    # Add waypoints to the mission
    for wp in waypoints:
        cmd = Command(0, 0, 0,
                      mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                      mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                      0, 0, 0, 0, 0, 0,
                      wp['lat'], wp['lon'], wp['alt'])
        cmds.add(cmd)

    # Add a landing command at the last waypoint's location
    land_cmd = Command(0, 0, 0,
                       mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                       mavutil.mavlink.MAV_CMD_NAV_LAND,
                       0, 0, 0, 0, 0, 0,
                       waypoints[-1]['lat'], waypoints[-1]['lon'], 0)
    cmds.add(land_cmd)
    cmds.upload()

# 3. Perpendicular Waypoint Calculation
def calculate_perpendicular(waypoints, wp_index=10, distance=100):
    """
    Generate a new waypoint that is perpendicular to the line connecting the given
    waypoint (at wp_index) to the next waypoint.
    """
    current = waypoints[wp_index]
    next_wp = waypoints[wp_index + 1]

    # Compute differences in latitude and longitude
    dx = next_wp['lon'] - current['lon']
    dy = next_wp['lat'] - current['lat']

    # Compute perpendicular direction
    perp_dx = -dy
    perp_dy = dx

    # Normalize the perpendicular vector
    length = math.sqrt(perp_dx**2 + perp_dy**2)
    if length == 0:
        raise ValueError("Cannot calculate perpendicular for identical waypoints.")

    # Convert desired distance in meters to an approximate degree offset.
    # Here we use a rough conversion: 1 degree ~ 111320 meters. Adjust as needed.
    degree_offset = distance / 111320

    return {
        'lat': current['lat'] + (perp_dy / length) * degree_offset,
        'lon': current['lon'] + (perp_dx / length) * degree_offset,
        'alt': current['alt']
    }

# 4. Mission Monitoring Class
class MissionMonitor:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.remaining_distance = 0
        self.speed = 10  # m/s
        self.actual_path = []
        self.last_update = 0
        self.update_interval = 5  # seconds

        # Register the location listener callback
        self.vehicle.add_attribute_listener('location', self.location_callback)

    def location_callback(self, vehicle, attr_name, location):
        """Callback to update mission status and estimate time remaining."""
        # Stop processing if the vehicle is not armed
        if not vehicle.armed:
            return

        current_time = time.time()
        # Record the actual flight path (latitude, longitude)
        self.actual_path.append((location.global_frame.lat, location.global_frame.lon))

        # Retrieve remaining commands by filtering commands with sequence >= current
        remaining_commands = [cmd for cmd in vehicle.commands if cmd.seq >= vehicle.commands.next]

        # Sum distances for only NAV_WAYPOINT commands
        self.remaining_distance = sum(
            self.distance_between_points(location.global_frame, cmd)
            for cmd in remaining_commands
            if cmd.command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        )

        if current_time - self.last_update >= self.update_interval:
            est_time = self.remaining_distance / self.speed if self.speed > 0 else 0
            print(f"Remaining Distance: {self.remaining_distance:.1f} m | ETA: {est_time:.1f} s")
            self.last_update = current_time

    def distance_between_points(self, current, cmd):
        """Calculate the Haversine distance between the current location and a command waypoint."""
        lat1 = math.radians(current.lat)
        lon1 = math.radians(current.lon)
        # The Command object stores latitude and longitude in x and y respectively.
        lat2 = math.radians(cmd.x)
        lon2 = math.radians(cmd.y)

        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return 6371000 * c  # Earth radius in meters

# 5. Enhanced Visualization Function
def plot_mission_2d(original_wps, modified_wps, actual_path=None):
    """Generate a 2D plot showing the original mission plan, modified path, and actual flight path."""
    plt.figure(figsize=(12, 8))

    # Plot original mission plan
    plt.plot([wp['lon'] for wp in original_wps],
             [wp['lat'] for wp in original_wps],
             'b--', marker='o', label='Original Plan')

    # Plot modified mission plan if it has more waypoints
    if len(modified_wps) > len(original_wps):
        plt.plot([wp['lon'] for wp in modified_wps],
                 [wp['lat'] for wp in modified_wps],
                 'r-', marker='s', label='Modified Path')

    # Plot the actual flight path if available
    if actual_path:
        plt.plot([p[1] for p in actual_path],
                 [p[0] for p in actual_path],
                 'g-', linewidth=2, label='Actual Flight Path')

    plt.xlabel('Longitude', fontsize=12)
    plt.ylabel('Latitude', fontsize=12)
    plt.title('Mission Trajectory Analysis', fontsize=14)
    plt.legend(loc='best')
    plt.grid(True, linestyle='--', alpha=0.7)

    # Add a scale bar (~11 meters)
    scale_lon = 0.0001  # Approximate degrees for ~11 meters at mid-latitude
    plt.plot([original_wps[0]['lon'], original_wps[0]['lon'] + scale_lon],
             [original_wps[0]['lat'], original_wps[0]['lat']],
             'k-', linewidth=2)
    plt.text(original_wps[0]['lon'] + scale_lon / 2, original_wps[0]['lat'] - 0.00005,
             '11m', ha='center', va='top')

    plt.tight_layout()
    plt.show()

# Main Execution
if __name__ == "__main__":
    vehicle = None
    try:
        print("Connecting to vehicle...")
        # Adjust the connection string as needed for your setup (e.g., SITL, serial, TCP)
        vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True)

        base_lat = 12.979327228659189
        base_lon = 77.59092091365582

        # Generate original mission waypoints
        original_waypoints = create_waypoints(
            base_lat=base_lat,
            base_lon=base_lon,
            num_waypoints=15,
            distance_between_points=100
        )

        # Insert a perpendicular waypoint into the original mission plan
        perp_wp = calculate_perpendicular(original_waypoints)
        # For example, insert the perpendicular waypoint after the 11th waypoint
        modified_waypoints = original_waypoints[:11] + [perp_wp] + original_waypoints[11:]

        # Upload the modified mission to the vehicle
        add_mission(vehicle, modified_waypoints)
        vehicle.commands.wait_ready()  # Ensure commands have been uploaded

        # Wait until the vehicle is ready to be armed
        while not vehicle.is_armable:
            print("Waiting for vehicle to initialize...")
            time.sleep(1)

        # Begin arming sequence
        print("Arming vehicle...")
        vehicle.mode = VehicleMode("GUIDED")  # Set mode to GUIDED before arming
        vehicle.armed = True
        while not vehicle.armed:
            print("Arming...")
            time.sleep(1)

        # Switch to AUTO mode to start mission execution
        vehicle.mode = VehicleMode("AUTO")
        print("Mission started!")
        monitor = MissionMonitor(vehicle)

        # Mission loop: wait until all mission commands have been executed.
        # Note: vehicle.commands.count provides the total number of commands.
        while vehicle.commands.next < vehicle.commands.count:
            time.sleep(1)

        print("Mission complete!")
        plot_mission_2d(original_waypoints, modified_waypoints, monitor.actual_path)
        vehicle.close()

    except KeyboardInterrupt:
        print("Mission aborted by user!")
        if vehicle:
            vehicle.close()
    except Exception as e:
        print(f"An error occurred: {e}")
        if vehicle:
            vehicle.close()
