import pymavobj as pym
import time
import math

pym = VyuhaController('/dev/ttyACM1')
pym.wait_heartbeat()
pym.arm_motors()

while True:
    print("1.Forward \n2.Reverse \n3.Turn right \n4.Turn left")
    movExists = int(input("Input goes here:"))
    pym.start_movement(movExists)
    while pym.is_busy():
        time.sleep(0.1)
    print("Task completed.")




'''import pymavobj as po
import time
import math

controller = VyuhaController('/dev/ttyACM1')
controller.wait_heartbeat()
controller.arm_motors()
print("1.Forward \n2.Reverse \n3.Turn right \n4.Turn left")
movExists = int(input("Input goes here:"))
controller.start_movement(movExists)
'''
