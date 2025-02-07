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

from can_simple_utils import CanSimpleNode

import tty
import sys
import termios

from xbox_controller import XboxController
from time import sleep

CLOSED_LOOP_CONTROL=8
IDLE=1

min_speed = 0
max_speed = 58

bus = can.interface.Bus("can0", bustype="socketcan", bytrate=250000)

# Flush CAN RX buffer so there are no more old pending messages
while not (bus.recv(timeout=0) is None): pass

right_tracks_node_ids = [21, 22]
left_tracks_node_ids = [23, 24]

# Initialize CAN nodes dynamically
right_tracks = [CanSimpleNode(bus, node_id) for node_id in right_tracks_node_ids]
left_tracks = [CanSimpleNode(bus, node_id) for node_id in left_tracks_node_ids]

use_tank_drive = False
debug_print = False

def tank_drive(x_axis, y_axis):
    left_speed = y_axis + x_axis
    right_speed = y_axis - x_axis
    left_speed = max(-1, min(1, left_speed))
    right_speed = max(-1, min(1, right_speed))
    return left_speed, right_speed

def set_state(state):
    for node in right_tracks + left_tracks:
        node.set_state_msg(state)
    waitState(state)
    print("Mode:", "Controlled" if state == CLOSED_LOOP_CONTROL else "Idle")

def runRight(speed):
    for node in right_tracks:
        node.set_velocity(speed)

def runLeft(speed):
    for node in left_tracks:
        node.set_velocity(-speed)

def clearErr():
    for node in right_tracks + left_tracks:
        node.clear_errors_msg()

def waitState(stateWaited):
    for msg in bus:
        for node in right_tracks + left_tracks:
            node.wait_state(stateWaited, msg)
        if debug_print:
            print([node.connected for node in right_tracks + left_tracks])
        if all(node.connected for node in right_tracks + left_tracks):
            break

clearErr()
set_state(IDLE)

for node in right_tracks + left_tracks:
    node.set_velocity(0)

xbox_controller = XboxController()
isOpen = False
isClearError = False

try:
    while True:
        sleep(0.1)
        speed = max(xbox_controller.RightTrigger * max_speed - 0.1, 0)

        if (xbox_controller.A == 1 or xbox_controller.RightBumper == 1) and not isOpen:
            use_tank_drive = xbox_controller.A == 1
            set_state(CLOSED_LOOP_CONTROL)
            isOpen = True

        if ((xbox_controller.A == 0 and xbox_controller.RightBumper == 0) or not xbox_controller.Connected) and isOpen:
            set_state(IDLE)
            isOpen = False

        if xbox_controller.B == 1 and not isClearError:
            clearErr()
            isClearError = True

        if xbox_controller.B == 0 and isClearError:
            isClearError = False

        if xbox_controller.UpDPad == 1:
            runRight(speed)
            runLeft(speed)
            if debug_print: print("Moving forward", speed)

        elif xbox_controller.DownDPad == 1:
            runRight(-speed)
            runLeft(-speed)
            if debug_print: print("Moving backward", speed)

        elif xbox_controller.LeftDPad == 1:
            runRight(speed)
            runLeft(-speed)
            if debug_print: print("Turning left", speed)

        elif xbox_controller.RightDPad == 1:
            runRight(-speed)
            runLeft(speed)
            if debug_print: print("Turning right", speed)

        else:
            if use_tank_drive:
                right, left = tank_drive(xbox_controller.LeftJoystickX, xbox_controller.LeftJoystickY)
                runRight(right * max_speed)
                runLeft(left * max_speed)
                if debug_print:
                    print(f'{xbox_controller.LeftJoystickX:.2f} {xbox_controller.LeftJoystickY:.2f} {left:.2f} {right:.2f}')
            else:
                runRight(xbox_controller.RightJoystickY * max_speed)
                runLeft(xbox_controller.LeftJoystickY * max_speed)
                if debug_print:
                    print(f'{xbox_controller.LeftJoystickY:.2f} {xbox_controller.RightJoystickY:.2f}')

except KeyboardInterrupt:
    print()

set_state(IDLE)
bus.shutdown()
print("Application exited")

# Print encoder feedback
# for msg in bus:
#     if msg.arbitration_id == (node_id << 5 | 0x09): # 0x09: Get_Encoder_Estimates
#         pos, vel = struct.unpack('<ff', bytes(msg.data))
#         print(f"pos: {pos:.3f} [turns], vel: {vel:.3f} [turns/s]")
