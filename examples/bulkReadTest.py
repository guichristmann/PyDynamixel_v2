import sys
import PyDynamixel as pd
from time import time

if len(sys.argv) != 3:
    print(f"Usage: python {sys.argv[0]} <baudnum> <protocol>")
    sys.exit(1)

# Use this so you don't need to specify it every time Joint() is called
pd.setDefaultCtrlTable("RH8D-1")

baudnum = int(sys.argv[1])
protocol_version = int(sys.argv[2])

port = pd.DxlComm("/dev/ttyUSB0", baudnum=baudnum, protocol_version=protocol_version)
j1 = pd.Joint(9)
j2 = pd.Joint(11)
j3 = pd.Joint(13)
j4 = pd.Joint(105)
port.attach_joints([j1, j2, j3, j4])

while True:
    start = time()
    ppositions = port.bulk_read_present_position()
    end = time()
    elapsed = end - start
    print(ppositions)
    print(f"{elapsed} s -- {1/elapsed} Hz")
