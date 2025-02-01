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
IDLE=1


node21 = True
node22 = True
node23 = True
node24 = True


def setOpen():
    if (node21) : can21.set_state_msg(CLOSED_LOOP_CONTROL)
    if (node22) : can22.set_state_msg(CLOSED_LOOP_CONTROL)
    if (node23) : can23.set_state_msg(CLOSED_LOOP_CONTROL)
    if (node24) : can24.set_state_msg(CLOSED_LOOP_CONTROL)
    waitState(CLOSED_LOOP_CONTROL)
    print("controlled mode")

def setIdle():
    if (node21) : can21.set_state_msg(IDLE)
    if (node22) : can22.set_state_msg(IDLE)
    if (node23) : can23.set_state_msg(IDLE)
    if (node24) : can24.set_state_msg(IDLE)
    waitState(IDLE)
    print("idle mode")

def runRight(speed):
    can21.set_velocity(speed)
    can22.set_velocity(speed)


def runLeft(speed):
    can23.set_velocity(-speed)
    can24.set_velocity(-speed)

def clearErr():
    can21.clear_errors_msg()
    can22.clear_errors_msg()
    can23.clear_errors_msg()
    can24.clear_errors_msg()

def waitState(stateWaited):
    node21_connected=False
    node22_connected=False
    node23_connected=False
    node24_connected=False

    for msg in bus:
        if msg.arbitration_id == (can21.node_id << 5 | 0x01): # 0x01: Heartbeat
            error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
            print("answer 21")
            if state == stateWaited: # 8: AxisState.stateWaited
                node21_connected = True
        if msg.arbitration_id == (can22.node_id << 5 | 0x01): # 0x01: Heartbeat
            error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
            print("answer 22")
            if state == stateWaited: # 8: AxisState.stateWaited
                node22_connected = True
        if msg.arbitration_id == (can23.node_id << 5 | 0x01): # 0x01: Heartbeat
            error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
            print("answer 23")
            if state == stateWaited: # 8: AxisState.stateWaited
                node23_connected = True
        if msg.arbitration_id == (can24.node_id << 5 | 0x01): # 0x01: Heartbeat
            error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
            print("answer 24")
            if state == stateWaited: # 8: AxisState.stateWaited
                node24_connected = True
        print( node21_connected, node22_connected, node23_connected, node24_connected)
        if (node21_connected or not node21) and (node22_connected or not node22) and (node23_connected or not node23) and (node24_connected or not node24):
            break

speed=5
min_speed = 0
max_speed = 15
last_key = ""
step = 5

bus = can.interface.Bus("can0", bustype="socketcan", bytrate=500000)

# Flush CAN RX buffer so there are no more old pending messages
while not (bus.recv(timeout=0) is None): pass

can21 = CanSimpleNode(bus, 21)
can22 = CanSimpleNode(bus, 22)
can23 = CanSimpleNode(bus, 23)
can24 = CanSimpleNode(bus, 24)

clearErr()

# Put axis into closed loop control state
setIdle()

# Wait for axis to enter closed loop control by scanning heartbeat messages

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
    askToOpen = False

    print("real", key)
    if key == "i":
        setIdle()
        key = last_key

    elif key == "o":
        askToOpen = True
        # speed = min(speed, 10)
        key = last_key

    elif key == "-":
        speed = max(speed - step, min_speed)
        key = last_key

    elif key == "+" or key == "=":
        speed = min(speed + step, max_speed)
        key = last_key

    elif key == " ":
        speed = 0
        key = last_key
    
    elif key == "e":
        clearErr()
        key = last_key

    last_key = key

    if key == "w":
        runRight(speed)
        runLeft(speed)
        print("front  ", speed)
    
    if key == "s":
        runRight(-speed)
        runLeft(-speed)
        print("back   ", speed)

    if key == "a":
        runRight(speed)
        runLeft(-speed)
        print("left   ", speed)

    if key == "d":
        runRight(-speed)
        runLeft(speed)
        print("right  ", speed)

    if askToOpen:
        setOpen()
    print(key)
    

termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)

# Print encoder feedback
# for msg in bus:
#     if msg.arbitration_id == (node_id << 5 | 0x09): # 0x09: Get_Encoder_Estimates
#         pos, vel = struct.unpack('<ff', bytes(msg.data))
#         print(f"pos: {pos:.3f} [turns], vel: {vel:.3f} [turns/s]")