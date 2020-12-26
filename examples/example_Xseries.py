import PyDynamixel as pd
from time import sleep
import math
import sys

n = sys.argv[1]
print(f"Will open at USB{n}")

pd.setDefaultCtrlTable("X-SERIES")

#port = pd.DxlComm("/dev/ttyUSB" + n, baudrate=2000000, protocol_version=2)
port = pd.DxlComm("/dev/ttyUSB" + n, baudnum=0, protocol_version=2)

#dyn = pd.Joint(21)
joints = [pd.Joint(20), pd.Joint(21)]
port.attach_joints(joints)

print("Enabling torques.")
port.enable_torques()

print("Set goal")
joints[0].set_goal_value(math.pi)
joints[1].set_goal_value(math.pi)
print("Sync write")
port.sync_write_goal_position()
sleep(2)
print("Set goal")
joints[0].set_goal_value(math.pi + math.pi/4)
joints[1].set_goal_value(math.pi + math.pi/4)
print("Sync write")
port.sync_write_goal_position()
sleep(2)
joints[0].set_goal_value(math.pi)
joints[1].set_goal_value(math.pi)
print("Sync write")
port.sync_write_goal_position()
sleep(2)
joints[0].set_goal_value(math.pi - math.pi/4)
joints[1].set_goal_value(math.pi - math.pi/4)
print("Sync write")
port.sync_write_goal_position()
'''
while True:
    for j in port.joints:
        j.set_goal_value(0.0)

    port.sync_write_goal_position()
    sleep(2)

    for j in port.joints:
        j.set_goal_value(5)

    port.sync_write_goal_position()
    sleep(2)
'''
