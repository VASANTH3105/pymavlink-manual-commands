from pymavlink import mavutil
import time
import math
import threading

class VyuhaController:
    def __init__(self, connection_string):
        self.master = mavutil.mavlink_connection(connection_string, baud=115200)
        self.buttons = 1 + 1 << 3 + 1 << 7
        self.running = False

    def wait_heartbeat(self):
        print("Waiting for heartbeat...")
        self.master.wait_heartbeat()
        print("Heartbeat from APM (system %u component %u)" % (self.master.target_system, self.master.target_system))
    
    def arm_motors(self):
        self.master.arducopter_arm()
        print("Arming motors")
        self.master.motors_armed_wait()
        print("Armed")

    def manual_control(self, x=0, y=0, z=500, yaw=0):
        self.master.mav.manual_control_send(
            self.master.target_system,
            x,
            y,
            z,  # 500 means neutral throttle
            yaw,
            self.buttons
        )

    def move_frd(self):
        self.running = True
        while self.running:
            self.manual_control(x=500) #initially 300
            time.sleep(2)

    def move_rev(self):
        self.running = True
        while self.running:
            self.manual_control(x=-500)
            time.sleep(2)

    def turn_rgt(self):
        self.running = True
        while self.running:
            self.manual_control(y=500)
            time.sleep(2)
                                                                                 
    def turn_lft(self):
        self.running = True
        while self.running:
            self.manual_control(y=-500)
            time.sleep(2)

    def start_movement(self, movExists):
        if movExists == 1:
            self.thread = threading.Thread(target=self.move_frd)
            self.thread.start()
        elif movExists == 2:
            self.thread = threading.Thread(target=self.move_rev)
            self.thread.start()
        elif movExists == 3:
            self.thread = threading.Thread(target=self.turn_rgt)
            self.thread.start()
        elif movExists == 4:
            self.thread = threading.Thread(target=self.turn_lft)
            self.thread.start()
        else:
            print("Null_Options")
         
    def stop_movement(self):
        self.running = False
        if self.thread:
            self.thread.join()
            self.thread = None

    def is_busy(self):
        return self.running or (self.thread and self.thread.is_alive())

# Usage example
controller = VyuhaController('/dev/ttyACM0')
controller.wait_heartbeat()
controller.arm_motors()

while True:
    print("1.Forward \n2.Reverse \n3.Turn right \n4.Turn left")
    movExists = int(input("Input goes here:"))
    controller.start_movement(movExists)
    time.sleep(2)  # wait for the movement to start
    while controller.is_busy():
        if input("Stop movement? (y/n) ") == 'y':
            controller.stop_movement()
            break
        time.sleep(0.1)
    print("Task completed.")

