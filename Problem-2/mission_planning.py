from dronekit import Command
from pymavlink import mavutil

def add_mission(vehicle, waypoints):
    """Upload mission with proper takeoff sequence"""
    cmds = vehicle.commands
    cmds.clear()
    
    # Add takeoff command first
    cmds.add(Command(0, 0, 0, 
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 10))
    
    # Add waypoints
    for wp in waypoints:
        cmds.add(Command(0, 0, 0,
                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                        0, 0, 0, 0, 0, 0,
                        wp['lat'], wp['lon'], wp['alt']))
    
    # Add landing command
    cmds.add(Command(0, 0, 0,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_LAND,
                    0, 0, 0, 0, 0, 0,
                    waypoints[-1]['lat'], waypoints[-1]['lon'], 0))
    
    cmds.upload()
    cmds.wait_ready()