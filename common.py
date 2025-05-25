# Control modes
from pathlib import Path
from typing import Dict, List, Literal, Union


CLOSED_LOOP_CONTROL = 8
IDLE = 1

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
