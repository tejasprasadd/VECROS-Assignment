# Fix for Python 3.10+ compatibility
import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping

import time
import threading
from dronekit import connect, VehicleMode
from waypoints import create_waypoints, calculate_perpendicular
from mission_planning import add_mission
from mission_monitor import MissionMonitor
from visualisation import plot_mission_2d

# Global variables for connection management
connection_lost = False
reconnect_flag = False
vehicle_instance = None
lock = threading.Lock()

if __name__ == "__main__":
    vehicle = None
    reconnect_attempts = 0
    max_reconnect_attempts = 3
    
    while reconnect_attempts < max_reconnect_attempts:
        try:
            if reconnect_flag:
                print(f"Attempting reconnect ({reconnect_attempts+1}/{max_reconnect_attempts})...")
                if vehicle is not None:
                    try:
                        with lock:
                            vehicle.close()
                    except Exception as e:
                        print(f"Error closing vehicle: {e}")
                # New connection handler
                vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True, timeout=60, rate=10)
                if vehicle is not None:
                    print("Reconnected successfully.")
                    # Reset flags and restart mission
                    with lock:
                        connection_lost = False
                        reconnect_flag = False
                    # Re-initialize mission components here
                    break
            else:
                print("Connecting to vehicle...")
                vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True, timeout=60, rate=10)
                if vehicle is not None:
                    print("Connected successfully.")
                    # Reset flags
                    with lock:
                        connection_lost = False
                        reconnect_flag = False
                    break
        except Exception as e:
            print(f"Connection attempt failed: {e}")
            reconnect_attempts += 1
            time.sleep(5)
    
    if vehicle is None:
        print("Failed to connect after multiple attempts.")
        exit()

    # Rest of mission execution
    try:
        # Set base coordinates
        base_lat = 12.979327
        base_lon = 77.590920
        
        # Generate mission waypoints
        original_waypoints = create_waypoints(base_lat, base_lon)
        perp_wp = calculate_perpendicular(original_waypoints)
        modified_waypoints = original_waypoints[:11] + [perp_wp] + original_waypoints[11:]
        
        # Upload mission
        add_mission(vehicle, modified_waypoints)
        
        # Arming sequence
        print("Arming vehicle...")
        vehicle.mode = VehicleMode("GUIDED")
        vehicle.parameters['ARMING_CHECK'] = 0
        vehicle.armed = True
        
        # Wait for arming with timeout
        arm_timeout = 30
        start_time = time.time()
        while not vehicle.armed:
            if time.time() - start_time > arm_timeout:
                raise RuntimeError("Arming timeout")
            print("Arming...")
            time.sleep(1)
        
        print("Starting mission...")
        vehicle.mode = VehicleMode("AUTO")
        monitor = MissionMonitor(vehicle)
        
        # Mission loop
        while vehicle.armed and vehicle.commands.next < vehicle.commands.count:
            time.sleep(1)
        
        if vehicle.armed:
            print("Mission complete! Landing...")
            vehicle.mode = VehicleMode("LAND")
            while vehicle.armed:
                time.sleep(1)
        
        print("Mission finished successfully!")
        plot_mission_2d(original_waypoints, modified_waypoints, monitor.actual_path)
        
    except KeyboardInterrupt:
        print("Mission aborted by user!")
    finally:
        if vehicle is not None:
            try:
                vehicle.close()
            except Exception as e:
                print(f"Error closing connection: {e}")