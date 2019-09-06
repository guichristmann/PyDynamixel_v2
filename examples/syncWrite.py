import PyDynamixel as pd
from time import sleep

pd.setDefaultCtrlTable("RH8D-1")

port = pd.DxlComm("/dev/ttyUSB0", baudnum=1, protocol_version=1)

port.attach_joints([pd.Joint(103), pd.Joint(105), pd.Joint(107), pd.Joint(109), pd.Joint(111)])

while True:
    for j in port.joints:
        j.set_goal_value(0.0)

    port.sync_write_goal_position()
    sleep(2)

    for j in port.joints:
        j.set_goal_value(5)

    port.sync_write_goal_position()
    sleep(2)
