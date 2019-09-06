# PyDynamixel_v2
Python interface to the Dynamixel protocol. Supports both Protocols 1 and 2 using Dynamixel SDK python library.

## Requirements
PySerial:
```
pip install pyserial
```
[Dynamixel SDK python library](https://github.com/ROBOTIS-GIT/DynamixelSDK)

## Before Usage - Maximize reading and writing speed

Give USB port permissions.
```
sudo chown username /dev/ttyUSB0
```

Set it to low_latency (1ms, instead of default 16ms)
```
setserial /dev/ttyUSB0 low_latency
```

Check if it worked.
```
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

## Usage and Examples

To install the module simply run

```
python setup.py install
```

The control tables for each motor, describing addresses and lengths are in `DynamixelControlTables.py`. The only motors implemented right now are MX-28 (for Protocol 1.0 and 2.0) and RH8D hand from Seed robotics (Protocol 1.0). If you want to add more motors just edit that file referencing the Dynamixel manuals and add the entry to the `ctrltables_str_mappings` variable.
The Sync and Bulk operations will only work if all motors attached to the port have the same control table.

#### Instantiating a single MX28 with Protocol 1.0 and reading present position:

```python
import PyDynamixel as pd

# Parameters
baudnum = 1 # baudrate is calculated as 2Mbps / (baudnum+1)
protocol_version = 1
motor_id = 10

# Create the port object
port = pd.DxlComm("/dev/ttyUSB0", baudnum=baudnum, protocol_version=protocol_version)
# Create the joint object. Control table for MX28 Protocol 1.0
joint = pd.Joint(motor_id, control_table="MX28-1")

# Attach joint to port
port.attach_joint(joint)

# Read current angle
curr_angle = joint.get_angle()
print(f"[{motor_id}]: {curr_angle:.2f}")
```

#### Instantiating a single MX28 with Protocol 2.0 and reading present position:

```python
import PyDynamixel as pd

# Parameters
baudnum = 1 # baudrate is calculated as 2Mbps / (baudnum+1)
protocol_version = 2
motor_id = 10

# Create the port object
port = pd.DxlComm("/dev/ttyUSB0", baudnum=baudnum, protocol_version=protocol_version)

# Create the joint object. Control table for MX28 Protocol 2.0
joint = pd.Joint(motor_id, control_table="MX28-2")

# Attach joint to port
port.attach_joint(joint)

# Read current angle
curr_angle = joint.get_angle()
print(f"[{motor_id}]: {curr_angle:.2f}")
```

#### Instantiating several MX28 Protocol 1.0 and reading present position sequentially:

```python
import PyDynamixel as pd

# Parameters
baudnum = 1 # baudrate is calculated as 2Mbps / (baudnum+1)
protocol_version = 1

# Create the port object
port = pd.DxlComm("/dev/ttyUSB0", baudnum=baudnum, protocol_version=protocol_version)

# Use this so you don't need to specify it every time Joint() is called
pd.setDefaultCtrlTable("MX28-1")
joints = [pd.Joint(1), pd.Joint(2), pd.Joint(3), pd.Joint(4)]

port.attach_joints(joints)

for joint in joints:
    curr_angle = joint.get_angle()
    print(f"[{joint.servo_id}]: {curr_angle:.2f}")
```

#### Instantiating several MX28 Protocol 1.0 and performing bulk read for present position:

```python
import PyDynamixel as pd

# Parameters
baudnum = 1 # baudrate is calculated as 2Mbps / (baudnum+1)
protocol_version = 1

# Create the port object
port = pd.DxlComm("/dev/ttyUSB0", baudnum=baudnum, protocol_version=protocol_version)

# Use this so you don't need to specify it every time Joint() is called
pd.setDefaultCtrlTable("MX28-1")
joints = [pd.Joint(1), pd.Joint(2), pd.Joint(3), pd.Joint(4)]

port.attach_joints(joints)

# Bulk read joints 1 and 2
angles = port.bulk_read_present_position(ids=[1, 2])
print(angles)

# Bulk read all attached joints
angles = port.bulk_read_present_position()
print(angles)
```

#### Instantiating several MX28 Protocol 1.0 and performing sync write of goal position:

```python
import PyDynamixel as pd

# Parameters
baudnum = 1 # baudrate is calculated as 2Mbps / (baudnum+1)
protocol_version = 1

# Create the port object
port = pd.DxlComm("/dev/ttyUSB0", baudnum=baudnum, protocol_version=protocol_version)

# Use this so you don't need to specify it every time Joint() is called
pd.setDefaultCtrlTable("MX28-1")
port.attach_joints([pd.Joint(1), pd.Joint(2), pd.Joint(3), pd.Joint(4), pd.Joint(5)])

# Set goal value in joint object. This does NOT send any messages, just changes
# value of the object in memory
for j in port.joints:
    j.set_goal_value(3.14)

# Sends the goal_value of each joint object to the actual motors
port.sync_write_goal_position()
