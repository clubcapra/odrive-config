"""
Minimal example for controlling an ODrive via the CANSimple protocol.

Puts the ODrive into closed loop control mode, sends a velocity setpoint of 1.0
and then prints the encoder feedback.

Assumes that the ODrive is already configured for velocity control.

See https://docs.odriverobotics.com/v/latest/manual/can-protocol.html for protocol
documentation.
"""

import can
import struct

from sympy import false
from can_simple_utils import CanSimpleNode

import tty
import sys
import termios

CLOSED_LOOP_CONTROL=8


node21 = True
node22 = True
node23 = True
node24 = True

node21_connected=False
node22_connected=False
node23_connected=False
node24_connected=False

speed=10
min_speed = 0
max_speed = 30
last_key = ""
step = 5

bus = can.interface.Bus("can0", bustype="socketcan", bytrate=500000)


# Flush CAN RX buffer so there are no more old pending messages
while not (bus.recv(timeout=0) is None): pass

can21 = CanSimpleNode(bus, 21)
can22 = CanSimpleNode(bus, 22)
can23 = CanSimpleNode(bus, 23)
can24 = CanSimpleNode(bus, 24)

can21.clear_errors_msg()
can22.clear_errors_msg()
can23.clear_errors_msg()
can24.clear_errors_msg()

# Put axis into closed loop control state
can21.set_state_msg(CLOSED_LOOP_CONTROL)
can22.set_state_msg(CLOSED_LOOP_CONTROL)
can23.set_state_msg(CLOSED_LOOP_CONTROL)
can24.set_state_msg(CLOSED_LOOP_CONTROL)

print("closed loop set")

# Wait for axis to enter closed loop control by scanning heartbeat messages
for msg in bus:
    if msg.arbitration_id == (can21.node_id << 5 | 0x01): # 0x01: Heartbeat
        error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
        if state == CLOSED_LOOP_CONTROL: # 8: AxisState.CLOSED_LOOP_CONTROL
            node21_connected = True
    if msg.arbitration_id == (can22.node_id << 5 | 0x01): # 0x01: Heartbeat
        error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
        if state == CLOSED_LOOP_CONTROL: # 8: AxisState.CLOSED_LOOP_CONTROL
            node22_connected = True
    if msg.arbitration_id == (can23.node_id << 5 | 0x01): # 0x01: Heartbeat
        error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
        if state == CLOSED_LOOP_CONTROL: # 8: AxisState.CLOSED_LOOP_CONTROL
            node23_connected = True
    if msg.arbitration_id == (can24.node_id << 5 | 0x01): # 0x01: Heartbeat
        error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
        if state == CLOSED_LOOP_CONTROL: # 8: AxisState.CLOSED_LOOP_CONTROL
            node24_connected = True
    print( node21_connected, node22_connected, node23_connected, node24_connected)
    if (node21_connected or not node21) and (node22_connected or not node22) and (node23_connected or not node23) and (node24_connected or not node24):
        break

print("closed loop confirmed")

# Set velocity to 1.0 turns/s
if (node21) : can21.set_velocity(0)
if (node22) : can22.set_velocity(0)
if (node23) : can23.set_velocity(0)
if (node24) : can24.set_velocity(0)

orig_settings = termios.tcgetattr(sys.stdin)

tty.setcbreak(sys.stdin)
x = 0
while x != chr(27): # ESC
    key=sys.stdin.read(1)[0]

    if key == "-":
        speed = max(speed - step, min_speed)
        key = last_key

    if key == "+" or key == "=":
        speed = min(speed + step, max_speed)
        key = last_key

    if key == " ":
        speed = 0
        key = last_key

    last_key = key

    if key == "w":
        if (node21) : can21.set_velocity(-speed)
        if (node22) : can22.set_velocity(-speed)
        if (node23) : can23.set_velocity(speed)
        if (node24) : can24.set_velocity(speed)
        print("front  ", speed)
    
    if key == "s":
        if (node21) : can21.set_velocity(speed)
        if (node22) : can22.set_velocity(speed)
        if (node23) : can23.set_velocity(-speed)
        if (node24) : can24.set_velocity(-speed)
        print("back   ", speed)

    if key == "a":
        if (node21) : can21.set_velocity(speed)
        if (node22) : can22.set_velocity(speed)
        if (node23) : can23.set_velocity(speed)
        if (node24) : can24.set_velocity(speed)
        print("left   ", speed)

    if key == "d":
        if (node21) : can21.set_velocity(-speed)
        if (node22) : can22.set_velocity(-speed)
        if (node23) : can23.set_velocity(-speed)
        if (node24) : can24.set_velocity(-speed)
        print("right  ", speed)
    

termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)

# Print encoder feedback
# for msg in bus:
#     if msg.arbitration_id == (node_id << 5 | 0x09): # 0x09: Get_Encoder_Estimates
#         pos, vel = struct.unpack('<ff', bytes(msg.data))
#         print(f"pos: {pos:.3f} [turns], vel: {vel:.3f} [turns/s]")