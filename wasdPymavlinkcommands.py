#importing Libraries
from pymavlink import mavutil
import time
import threading
import keyboard
import math
# Create a MAVLink connection
master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=115200)
print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Heartbeat from APM (system %u component %u)" %
      (master.target_system, master.target_system))

# Arm the motors
master.arducopter_arm()
print("Arming motors")
master.motors_armed_wait()
print("Armed")

# Define constants for directions
FORWARD = 1
BACKWARD = 2
LEFT = 3
RIGHT = 4
UP = 5
DOWN = 6


# Define a function to send a manual control message
def manual_control(x=0, y=0, z=0, yaw=0, buttons=0):
    master.mav.manual_control_send(
        master.target_system,
        x,
        y,
        z,
        yaw,
        buttons
    )


# Define a function to move the vehicle in a specific direction
def move_direction(direction, distance):
    # Convert distance to centimeters
    distance_cm = distance * 100

    # Define a dictionary to map directions to manual control inputs
    direction_inputs = {
        FORWARD: (300, 0, 0, 0),
        BACKWARD: (-300, 0, 0, 0),
        LEFT: (0, -300, 0, 0),
        RIGHT: (0, 300, 0, 0),
        UP: (0, 0, -500, 0),
        DOWN: (0, 0, 500, 0)
    }

    # Get the manual control input for the direction
    input = direction_inputs.get(direction)
    if input is None:
        print('Invalid direction')
        return

    # Send manual control messages in a loop
    def send_control_loop():
        for i in range(50):
            manual_control(*input)
            time.sleep(0.1)

    thread = threading.Thread(target=send_control_loop)
    thread.start()


# Define a function to turn the vehicle
def turn(degrees):
    # Convert degrees to radians
    radians = degrees * math.pi / 180.0

    # Define a dictionary to map turn directions to manual control inputs
    direction_inputs = {
        LEFT: (0, 0, 0, 1000),
        RIGHT: (0, 0, 0, -1000)
    }

    # Get the manual control input for the turn direction
    input = direction_inputs.get(degrees / abs(degrees))
    if input is None:
        print('Invalid turn direction')
        return

    # Send manual control messages in a loop
    def send_control_loop():
        for i in range(int(abs(radians * 50))):
            manual_control(*input)
            time.sleep(0.02)

    thread = threading.Thread(target=send_control_loop)
    thread.start()


# Define a function to stop the vehicle
def stop():
    manual_control()


# Define a function to handle keyboard input
def handle_keyboard_input():
    while True:
        if keyboard.is_pressed('w'):
            move_direction(FORWARD, 1)
        elif keyboard.is_pressed('s'):
            move_direction(BACKWARD, 1)
        elif keyboard.is_pressed('a'):
            turn(90)
        elif keyboard.is_pressed('d'):
            turn(90)
