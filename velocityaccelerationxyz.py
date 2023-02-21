import time
from pymavlink import mavutil

# Connect to the vehicle's autopilot
connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Heartbeat from APM (system %u component %u)" %
      (master.target_system, master.target_system))
master.arducopter_arm()																																																																																																																					
print("Arming motors")
master.motors_armed_wait()
print("ARmed")
# Wait for the connection to be established
time.sleep(3)


# Set the target velocity in the x-direction (forward)
vx = 5  # m/s

# Set the other velocity components to 0 (not moving in y or z directions)
vy = 0
vz = 0

# Create the SET_POSITION_TARGET_LOCAL_NED message
msg = connection.message_factory.set_position_target_local_ned_encode(
    0,  # time_boot_ms (not used)
    0, 0,  # target system and target component (not used)
    mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
    0b0000111111000111,  # type_mask (only using velocity components)
    0, 0, 0,  # x, y, and z positions (not used)
    vx, vy, vz,  # x, y, and z velocity components
    0, 0, 0,  # x, y, and z acceleration (not used)
    0, 0)  # yaw and yaw rate (not used)

# Send the message to the vehicle's autopilot
connection.send(msg)

