from doctest import master
from pymavlink import mavutil
import time


def move_forward(lat, lon, alt, speed):
    # Send MAVLink message to set the vehicle's mode to AUTO
    master.mav.command_long_send(
        master.target_system,  # Target system ID
        master.target_component,  # Target component ID
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,  # Command ID
        0,  # Confirmation
        mavutil.mavlink.MAV_MODE_AUTO,  # Mode
        0,  # Custom mode
        0,  # Custom submode
        0,  # Parameters 1-7
        0
    )

    # Send MAVLink message to set the vehicle's home position to current location
    master.mav.command_long_send(
        master.target_system,  # Target system ID
        master.target_component,  # Target component ID
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,  # Command ID
        0,  # Confirmation
        0,  # Waypoint index
        0,  # Is current home flag
        0,  # Is set position flag
        0,  # Params 1-4 unused
        0,
        0,
        0
    )

    # Send MAVLink message to set the vehicle's mission item to move forward
    master.mav.mission_item_send(
        master.target_system,  # Target system ID
        master.target_component,  # Target component ID
        0,  # Sequence number
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # Frame
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  # Command ID
        0,  # Current waypoint
        0,  # Autocontinue
        0,  # Parameters 4-7 unused
        0,
        0,
        0,
        lat,  # Latitude
        lon,  # Longitude
        alt  # Altitude
    )

    # Send MAVLink message to set the vehicle's speed
    master.mav.command_long_send(
        master.target_system,  # Target system ID
        master.target_component,  # Target component ID
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,  # Command ID
        0,  # Confirmation
        1,  # Speed type (airspeed)
        speed,  # Speed value (m/s)
        0,  # Throttle value
        0,  # Parameters 4-7 unused
        0,
        0,
        0
    )


master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Heartbeat from APM (system %u component %u)" %
      (master.target_system, master.target_system))
master.arducopter_arm()																																																																																																																					
print("Arming motors")
master.motors_armed_wait()
print("ARmed")
time.sleep(3)

while True:
	move_forward(lat=11.500032, log=77.276880, alt=0, speed=2)






