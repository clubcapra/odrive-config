# Control modes
from pathlib import Path
from typing import Dict, List, Literal, Union

# odrive commands
ESTOP_CMD = 0x02
HEARTBEAT_CMD = 0x01
GET_ERROR_CMD = 0x03

ADDRESS_CMD = 0x06
SET_AXIS_STATE_CMD = 0x07
REBOOT_CMD = 0x16
CLEAR_ERRORS_CMD = 0x18
SET_INPUT_POS_CMD = 0x0C  # Set_Input_Pos command ID
SET_INPUT_VEL_CMD = 0x0D  # Set_Input_Pos command ID
GET_ENCODER_ESTIMATES_CMD = 0x09
GET_TEMPERATURE_CMD = 0x15
GET_BUS_VOLTAGE_CURRENT_CMD = 0x17
GET_POWERS_CMD = 0x1D
GET_TORQUES_CMD = 0x1C
SET_TRAJ_VEL_LIMIT_CMD = 0x11
    

REBOOT_ACTION_REBOOT = 0
REBOOT_ACTION_SAVE = 1
REBOOT_ACTION_ERASE = 2


# odrive axis state
STATE_UNDEFINED = 0
STATE_IDLE = 1
STATE_CLOSED_LOOP_CONTROL = 8

# odrive control mode
MODE_VOLTAGE_CONTROL = 0
MODE_TORQUE_CONTROL = 1
MODE_VELOCITY_CONTROL = 2
MODE_POSITION_CONTROL = 3

# odrive input mode
INPUT_INACTIVE = 0
INPUT_PASSTHROUGH = 1

# Drive parameters
MAX_TRACK_SPEED = 58        # rev/s
FLIPPER_SPEED = 58.0         # rev/s
FLIPPER_MOVE_OFFSET = 50         # revs
MAIN_LOOP_INTERVAL = 0.1    # seconds
WATCHDOG_INTERVAL = 1.0     # seconds
TEMP_LOG_INTERVAL = 2.0     # seconds

XBOX_CONFIG_PATH = Path('bindings')

Side = Union[str, Literal['left', 'right']]

# CAN node IDs
TRACK_IDS: Dict[Side, List[int]] = {
    'left':  [21, 22],
    'right': [23, 24],
}

Pos = Union[str, Literal['front_left', 'rear_left', 'front_right', 'rear_right']]

Pair = Union[str, Literal['front', 'rear']]

FLIPPER_IDS: Dict[Pos, int] = {
    'front_left' : 11,
    'rear_left' : 12,
    'front_right' : 13,
    'rear_right' : 14,
}
GET_TEMPERATURE_CMD = 0x15
FLIPPER_OFFSETS_PATH = Path("flipper_pos.json")

def niceFloat(value: float, decimals:int = 3, justify:int = 6) -> str:
    return str(round(value, decimals)).ljust(justify)
