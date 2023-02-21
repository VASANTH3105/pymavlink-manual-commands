import time
from pymavlink import mavutil

# Connect to the vehicle
connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
connection.wait_heartbeat()

# Define the target position (10 meters forward, same east and down coordinates)
target_north = 10
target_east = 0
target_down = 0

# Send the SET_POSITION_TARGET_LOCAL_NED command
msg = connection.mav.set_position_target_local_ned_encode(
    0,  # time_boot_ms (not used)
    0, 0,  # target system, target component
    mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
    0b0000111111111000,  # type_mask (only position fields)
    target_east, target_north, -target_down,  # x, y, z positions
    0, 0, 0,  # x, y, z velocity (not used)
    0, 0, 0,  # x, y, z acceleration (not used)
    0, 0)  # yaw, yaw_rate (not used)
connection.mav.send(msg)

# Wait for the vehicle to move to the target position
time.sleep(5)	

