import math
import time
import threading
from pymavlink import mavutil
from dronekit import VehicleMode

class MissionMonitor:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.remaining_distance = 0
        self.speed = 10  # m/s
        self.actual_path = []
        self.last_update = time.time()
        self.update_interval = 3
        self.last_heartbeat = time.time()
        
        self.vehicle.add_attribute_listener('location', self.location_callback)
        self.vehicle.add_attribute_listener('heartbeat', self.heartbeat_callback)

    def heartbeat_callback(self, vehicle, attr_name, value):
        """Handle heartbeat updates"""
        self.last_heartbeat = time.time()

    def location_callback(self, vehicle, attr_name, location):
        """Handle position updates"""
        if not vehicle.armed:
            return

        self.actual_path.append((location.global_frame.lat, location.global_frame.lon))
        
        # Update remaining distance
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
        """Precise distance calculation"""
        R = 6371000  # Earth radius in meters
        φ1 = math.radians(lat1)
        φ2 = math.radians(lat2)
        Δφ = math.radians(lat2 - lat1)
        Δλ = math.radians(lon2 - lon1)

        a = math.sin(Δφ/2)**2 + math.cos(φ1)*math.cos(φ2)*math.sin(Δλ/2)**2
        return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1-a))