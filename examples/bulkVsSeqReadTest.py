import sys
import PyDynamixel as pd
from time import time, sleep
import numpy as np

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
j4 = pd.Joint(101)
j5 = pd.Joint(103)
j6 = pd.Joint(105)
j7 = pd.Joint(107)
j8 = pd.Joint(109)
j9 = pd.Joint(111)
port.attach_joints([j1, j2, j3, j4, j5, j6, j7, j8, j9])

times = []
for i in range(300):
    start = time()
    ppositions = port.bulk_read_present_position()
    end = time()
    elapsed = end - start
    times.append(elapsed)
    print(ppositions)
    print(f"{elapsed} s -- {1/elapsed} Hz")

bulk_mean = np.mean(times)
print(f"Mean: {bulk_mean} s -- {1/bulk_mean} Hz")
sleep(10)

times = []
for i in range(300):
    start = time()
    for j in port.joints:
        j.get_angle()
    end = time()
    elapsed = end - start
    times.append(elapsed)
    print(f"{elapsed} s -- {1/elapsed} Hz")

seq_mean = np.mean(times)
print("---- Bulk Read ----")
print(f"Mean: {bulk_mean} s -- {1/bulk_mean} Hz")
print("---- Sequential Read ----")
print(f"Mean: {seq_mean} s -- {1/seq_mean} Hz")
