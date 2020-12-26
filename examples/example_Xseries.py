import PyDynamixel as pd
from time import sleep
import math

pd.setDefaultCtrlTable("X-SERIES")

port = pd.DxlComm("/dev/ttyUSB0", baudrate=2000000, protocol_version=2)

#dyn = pd.Joint(21)
joints = [pd.Joint(20), pd.Joint(21)]
port.attach_joints(joints)

port.enable_torques()

joints[0].set_goal_value(0.0)
port.sync_write_goal_position()
sleep(2)
joints[0].set_goal_value(math.pi/2)
port.sync_write_goal_position()
sleep(2)
joints[0].set_goal_value(math.pi)
port.sync_write_goal_position()
sleep(2)
joints[0].set_goal_value(math.pi + math.pi/2)
port.sync_write_goal_position()
sleep(2)
joints[0].set_goal_value(2*math.pi - 0.05)
port.sync_write_goal_position()
sleep(2)
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