# Import mavutil
from pymavlink import mavutil

master = mavutil.mavlink_connection('/dev/ttyACM0')
print("Port")
# Disarm
# master.arducopter_disarm() or:
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0)
print("Function definition")
# wait until disarming confirmed
master.motors_disarmed_wait()
print("Disarmed")
