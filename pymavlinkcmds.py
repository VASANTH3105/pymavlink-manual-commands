import time
from pymavlink import mavutil

# create a MAVLink connection object
master = mavutil.mavlink_connection('/dev/ttyACM0')
# wait for the heartbeat message to be received
master.wait_heartbeat()
print("Heartbeat connected")
# set the current position as the home position
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0, 0, 0, 0, 0, 0, 0, 0)
print("At home position")
# set the mode to guided
master.mav.set_mode_send(
    master.target_system, mavutil.mavlink.MAV_MODE_GUIDED_ARMED, 0)
print("Guided")
# move forward for 10 meters
master.mav.guided_limit_velocity_send(
    master.target_system, 10, 0, 0, -1)
print("Moving forward 10 meters")
time.sleep(10)

# turn right
master.mav.guided_limit_yaw_send(
    master.target_system, 0, 0, 1, -1)
print("Turned right")
time.sleep(10)

# stop
master.mav.guided_limit_velocity_send(
    master.target_system, 0, 0, 0, -1)

# Disarm
# master.arducopter_disarm() or:
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0)

# wait until disarming confirmed
master.motors_disarmed_wait()
print("Disarmed")
