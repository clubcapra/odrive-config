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


node21 = True
node22 = True
node23 = True
node24 = True

use_tank_drive = False
debug_print = False

def tank_drive(x_axis, y_axis):
    # Tank drive model:
    # Left speed = y + x (for forward/reverse and turning)
    # Right speed = y - x (for forward/reverse and turning)
    
    # Ensure values stay within the -1 to 1 range.
    left_speed = y_axis + x_axis  # Forward/backward + turning
    right_speed = y_axis - x_axis  # Forward/backward - turning
    
    # Clamp the speeds to the range [-1, 1] to prevent overflow.
    left_speed = max(-1, min(1, left_speed))
    right_speed = max(-1, min(1, right_speed))
    
    return left_speed, right_speed

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
    for msg in bus:
        can21.wait_state(stateWaited, msg)
        can22.wait_state(stateWaited, msg)
        can23.wait_state(stateWaited, msg)
        can24.wait_state(stateWaited, msg)
        
        if debug_print:
            print( can21.connected, can22.connected, can23.connected, can24.connected)
        if (can21.connected or not node21) and (can22.connected or not node22) and (can23.connected or not node23) and (can24.connected or not node24):
            break

min_speed = 0
max_speed = 58

bus = can.interface.Bus("can0", bustype="socketcan", bytrate=250000)

# Flush CAN RX buffer so there are no more old pending messages
while not (bus.recv(timeout=0) is None): pass

can21 = CanSimpleNode(bus, 11)
can22 = CanSimpleNode(bus, 22)
can23 = CanSimpleNode(bus, 23)
can24 = CanSimpleNode(bus, 13)

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

xbox_controller = XboxController()
isOpen = False
isClearError = False
try:
    while True: # ESC
        sleep(0.1)
        speed = max(xbox_controller.RightTrigger * max_speed - 0.1, 0)

        if (xbox_controller.A == 1 or xbox_controller.RightBumper == 1) and isOpen == False:
            if xbox_controller.A == 1:
                use_tank_drive = True
            elif xbox_controller.RightBumper == 1:
                use_tank_drive = False
            setOpen()
            isOpen = True
            # speed = min(speed, 10)

        if ((xbox_controller.A == 0 and xbox_controller.RightBumper == 0) or xbox_controller.Connected == False) and isOpen == True:
            setIdle()
            isOpen = False
        
        if xbox_controller.B == 1 and isClearError == False:
            clearErr()
            isClearError = True

        if xbox_controller.B == 0 and isClearError == True:
            isClearError = False

        if xbox_controller.UpDPad == 1:
            runRight(speed)
            runLeft(speed)
            if debug_print:
                print("front  ", speed)
        
        elif xbox_controller.DownDPad == 1:
            runRight(-speed)
            runLeft(-speed)
            if debug_print:
                print("back   ", speed)

        elif xbox_controller.LeftDPad == 1:
            runRight(speed)
            runLeft(-speed)
            if debug_print:
                print("left   ", speed)

        elif xbox_controller.RightDPad == 1:
            runRight(-speed)
            runLeft(speed)
            if debug_print:
                print("right  ", speed)
        else:
            # runRight(0)
            # runLeft(0)
            
            if use_tank_drive:
                right, left = tank_drive(xbox_controller.LeftJoystickX, xbox_controller.LeftJoystickY)
                if debug_print:
                    print('%.2f' % xbox_controller.LeftJoystickX, '%.2f' % xbox_controller.LeftJoystickY, '%.2f' % left, '%.2f' % right)
                runRight(right * max_speed)
                runLeft(left * max_speed)
            else:
                runRight(xbox_controller.RightJoystickY * max_speed)
                runLeft(xbox_controller.LeftJoystickY * max_speed)
                if debug_print:
                    print('%.2f' % xbox_controller.LeftJoystickY, '%.2f' % xbox_controller.RightJoystickY)

except KeyboardInterrupt:
    print()

setIdle()
bus.shutdown()
print("Application exited")

# Print encoder feedback
# for msg in bus:
#     if msg.arbitration_id == (node_id << 5 | 0x09): # 0x09: Get_Encoder_Estimates
#         pos, vel = struct.unpack('<ff', bytes(msg.data))
#         print(f"pos: {pos:.3f} [turns], vel: {vel:.3f} [turns/s]")
