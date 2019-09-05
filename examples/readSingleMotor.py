import sys
import PyDynamixel as pd

if len(sys.argv) != 4:
    print(f"Usage: python {sys.argv[0]} <baudnum> <protocol> <id>")
    sys.exit(1)

# Use this so you don't need to specify it every time Joint() is called
pd.setDefaultCtrlTable("RH8D-1")

baudnum = int(sys.argv[1])
protocol_version = int(sys.argv[2])
motor_id = int(sys.argv[3])

port = pd.DxlComm("/dev/ttyUSB0", baudnum=baudnum, protocol_version=protocol_version)
joint = pd.Joint(motor_id)
port.attach_joint(joint)

while True:
    curr_angle = joint.get_angle()
    print(f"[{motor_id}]: {curr_angle:.2f}")
