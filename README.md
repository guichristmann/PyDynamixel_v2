# PyDynamixel_v2
Python interface to the Dynamixel protocol. Supports both Protocols 1 and 2 using Dynamixel SDK python library.

Default is MX-28 control table.

## Requirements
PySerial:
```
pip install pyserial
```
Dynamixel SDK python library: 
```
https://github.com/ROBOTIS-GIT/DynamixelSDK
```

## Usage

teste.py script has main functionalities, but basically:

Create a DxlComm instance, the Joint instances, and then attach them to the first.
```
import PyDynamixel_v2 as pd
from math import pi

serial = pd.DxlComm(port='/dev/ttyUSB0', baudrate=1000000)
joint1 = pd.Joint(servo_id=1)
joint2 = pd.Joint(servo_id=2)

serial.attach_joints([joint1, joint2])
```

Enable joint torques and then move them with sync write.
```
serial.enable_joints()
serial.send_angles([90, 20]) 
  # send these degrees in the same order as the joints were attached
  # joint1 = 90 degrees, joint2 = 20 degrees
serial.send_angles([pi/2, pi/3], radian=True) # angles can be sent as radians too
```

Read joints current position angle. Protocol 2.0 uses sync read.
```
angles = serial.get_angles()
  # the radian flag also works here
angles = serial.get_angles(radian=radian)
```

There are also individual read and write functions.
```
joint1.get_angle()
joint1.send_angle(pi/2, radian=True)
```

There are two ping functions, individual and broadcast. Broadcast is only available within Protocol 2.0
```
serial.broadcast_ping()
joint1.ping()
```
