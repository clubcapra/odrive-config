import asyncio
from dataclasses import dataclass
import itertools
import json
from pathlib import Path
import sys
import can
import struct
import threading
import time
from time import sleep
from typing import List, Dict, Sequence, Tuple, Literal, Union
from asyncio import gather

import prompt_toolkit.completion
import prompt_toolkit.contrib
import prompt_toolkit.contrib.completers
import prompt_toolkit.contrib.regular_languages
import prompt_toolkit.validation

from can_simple_utils import CanSimpleNode
from xbox_controller import ControllerBindings, XboxController

import prompt_toolkit

from control import Device, Flipper, MockFlipper, converge_group, load_flippers, move_group, move_single, save_flippers

# Control modes
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

FLIPPER_IDS: Dict[Pos, int] = {
    'front_left' : 11,
    'rear_left' : 12,
    'front_right' : 13,
    'rear_right' : 14,
}
GET_TEMPERATURE_CMD = 0x15


def init_can_bus(channel:str='can0', bitrate:int=500000) -> can.BusABC:
    print("Starting can bus")
    bus = can.interface.Bus(channel=channel, interface='socketcan', bitrate=bitrate)
    # flush pending frames
    while bus.recv(timeout=0):
        pass
    print("Done")
    return bus


def create_nodes(bus: can.BusABC) -> Tuple[Dict[Side, List[CanSimpleNode]], Dict[Pos, CanSimpleNode]]:
    print("Creating nodes")
    tracks: Dict[Side, List[CanSimpleNode]] = {side: [CanSimpleNode(bus, nid) for nid in ids]
              for side, ids in TRACK_IDS.items()}
    flippers: Dict[Pos, CanSimpleNode] = {pos: CanSimpleNode(bus, nid)
              for pos, nid in FLIPPER_IDS.items()}
    print("Done")
    return tracks, flippers


def estop_monitor(nodes: List[CanSimpleNode], bus: can.BusABC, heartbeat: threading.Event) -> None:
    """Shuts down motors if heartbeat is missed."""
    while True:
        heartbeat.clear()
        if not heartbeat.wait(timeout=WATCHDOG_INTERVAL):
            print("[ERROR] No heartbeat: triggering E-Stop")
            for node in nodes:
                node.call_estop()


def error_monitor(nodes: List[CanSimpleNode], bus: can.BusABC) -> None:
    """Listens for node errors and triggers E-Stop if any occur."""
    while True:
        msg = bus.recv()
        if msg and (msg.arbitration_id & 0x1F) == 0x01:
            code = struct.unpack('<I', msg.data[:4])[0]
            if code != 0:
                nid = msg.arbitration_id >> 5
                print(f"[ERROR] Node {nid} error {code}: triggering E-Stop")
                for node in nodes:
                    node.call_estop()
                break

def pos_monitor(flippers: Dict[Pos, Flipper]) -> None:
    asyncio.run(asyncio.wait([f.update() for f in flippers.values()]))

def clamp(val:float, lo:float=-1.0, hi:float=1.0) -> float:
    return max(min(val, hi), lo)


def handle_tracks(controller: XboxController, tracks: Dict[Side, List[CanSimpleNode]], enabled:bool) -> None:
    """
    Left stick X = throttle, Left stick Y = steering.
    """
    if enabled:
        throttle = -clamp(controller.LeftJoystickX.value)
        steering = clamp(controller.LeftJoystickY.value)
        left_cmd  = throttle + steering
        right_cmd = throttle - steering
    else:
        left_cmd = right_cmd = 0.0

    left_speed  = left_cmd  * MAX_TRACK_SPEED
    right_speed = right_cmd * MAX_TRACK_SPEED

    for node in tracks['left']:
        node.set_velocity(left_speed)
    for node in tracks['right']:
        node.set_velocity(right_speed)

def handle_flippers(controller: XboxController, flippers: Dict[Pos, Flipper], enabled:bool) -> None:
    """Sets flipper velocities based on D-pad Up/Down."""
    positions: List[Pos] = []
    if controller.LeftBumper.state:
        positions.append('front_left')
    if controller.LeftTriggerBtn.state:
        positions.append('rear_left')
    if controller.RightBumper.state:
        positions.append('front_right')
    if controller.RightTriggerBtn.state:
        positions.append('rear_right')

    if len(positions) == 0:
        positions = [
            'front_left',
            'rear_left',
            'front_right',
            'rear_right',
        ]
    
    def _move(pos: float):
        if len(positions) != 1:
            # Control all together 
            if enabled:
                move_group([flippers[p] for p in positions], pos)
            else:
                move_group([flippers[p] for p in positions], 0)
        else:
            move_single(flippers[positions[0]], pos)

        for p in filter(lambda p: p not in positions, flippers.keys()):
            flippers[p].setPosition = flippers[p].position
            
    
    if wasMoving and not (controller.UpDPad.state or controller.DownDPad.state):
        move_group([flippers[p] for p in positions], 0)
        for p in filter(lambda p: p not in positions, flippers.keys()):
            flippers[p].setPosition = flippers[p].position
    if controller.UpDPad.state:
        _move(FLIPPER_MOVE_OFFSET)
    elif controller.DownDPad.state:
        _move(-FLIPPER_MOVE_OFFSET)
    elif controller.LeftDPad.state and controller.Y.state and not enabled:
        for p in positions:
            flippers[p].zero()
    elif controller.RightDPad.state:
        # Converge
        converge_group([flippers[p] for p in positions])

def xbox_binding_completer() -> Sequence[str]:
    return [f.stem for f in XBOX_CONFIG_PATH.iterdir()]

def save_bindings(controller: XboxController):
    while True:
        completer = prompt_toolkit.completion.FuzzyWordCompleter(xbox_binding_completer)
        name = prompt_toolkit.shortcuts.input_dialog("Save bindings", "bindings file name", 
                                                completer=completer).run()
        if not name.endswith('.json'):
            name += '.json'
            
        path = XBOX_CONFIG_PATH.with_name(name)
        if path.exists():
            if not prompt_toolkit.shortcuts.yes_no_dialog("Overwrite?", "File already exists, do you want to overwrite?"):
                continue
        
        with path.open('+w') as wr:
            json.dump(controller.bindings.dump(), wr)
            prompt_toolkit.print_formatted_text(f"Saved to {str(path)}")
            return

def learn_new(controller: XboxController):
    controller.learn()
    save_bindings(controller)

def select_bindings(controller: XboxController):
    if not XBOX_CONFIG_PATH.exists():
        XBOX_CONFIG_PATH.mkdir()
    if len(list(XBOX_CONFIG_PATH.iterdir())) == 0:
        learn_new(controller)
        return
    
    res = prompt_toolkit.shortcuts.button_dialog("Bindings selection",
                                           [
                                               ('Create new', '*new'),
                                               *[(n, n) for n in xbox_binding_completer()]
                                           ])
    if res == '*new':
        learn_new(controller)
    else:
        path = XBOX_CONFIG_PATH.with_name(f'{res}.json')
        with path.open('r') as rd:
            data = json.load(rd)
            controller.load_bindings(ControllerBindings.load(data))

def main():
    mockFlippers = False
    xbox = XboxController()
    if len(sys.argv) >= 2:
        if 'mock' in sys.argv:
            mockFlippers = True
        if 'config' in sys.argv:
            select_bindings(xbox)
            
        print("Usage: run.py [mock]")
        
    if not mockFlippers:
        bus = init_can_bus()
        tracks, flippers = create_nodes(bus)
        all_nodes = tracks['left'] + tracks['right'] + list(flippers.values())
        flipper_devs: Dict[Pos, Flipper] = {name: Flipper(node) for name, node in flippers.items()}
        load_flippers(flipper_devs)
    else:
        flipper_devs: Dict[Pos, Flipper] = {
            'front_left' : MockFlipper(58, 10),
            'rear_left' : MockFlipper(58, 9),
            'front_right' : MockFlipper(55, 10),
            'rear_right' : MockFlipper(55, 9),
        }

    if not mockFlippers:
        heartbeat = threading.Event()

        threading.Thread(target=estop_monitor, args=(all_nodes, bus, heartbeat), daemon=True).start()
        threading.Thread(target=error_monitor, args=(all_nodes, bus), daemon=True).start()

        for node in all_nodes:
            node.clear_errors_msg()
            node.set_state_msg(IDLE)

    drive_enabled = False
    error_cleared = False

    try:
        while True:
            if not mockFlippers:
                heartbeat.set()

            # E-Stop: bumpers
            if xbox.LeftBumper.state or xbox.RightBumper.state:
                if not mockFlippers:
                    for node in all_nodes:
                        node.call_estop()
                else:
                    for d in flipper_devs.values():
                        d.enable = False # type: ignore
                drive_enabled = False

            # Toggle drive enable: A button
            if xbox.A.state and not drive_enabled:
                if not mockFlippers:
                    for node in all_nodes:
                        node.set_state_msg(CLOSED_LOOP_CONTROL)
                else:
                    for d in flipper_devs.values():
                        d.enable = True # type: ignore
                drive_enabled = True
            elif (not xbox.A.state or not xbox.Connected) and drive_enabled:
                if not mockFlippers:
                    for node in all_nodes:
                        node.set_state_msg(IDLE)
                else:
                    for d in flipper_devs.values():
                        d.enable = False # type: ignore
                drive_enabled = False

            # Clear errors: B button
            if xbox.B.state and not error_cleared:
                if not mockFlippers:
                    for node in all_nodes:
                        node.clear_errors_msg()
                error_cleared = True
            elif not xbox.B.state:
                error_cleared = False

            pos_monitor(flipper_devs)
            handle_flippers(xbox, flipper_devs, drive_enabled)
            if not mockFlippers:
                handle_tracks(xbox, tracks, drive_enabled)
            else:
                print()
                for name, flipper in flipper_devs.items():
                    shortName = ''.join([n[0] for n in name.split('_')]).upper()
                    values: Dict[str, float] = {
                        '_p': flipper._position,
                        '_s': flipper._setPosition,
                        'P': flipper.position,
                        'S': flipper.setPosition,
                        '_o': flipper._offset,
                        '_t': flipper._targetOffset,
                        '_v': flipper._velocity,
                    }
                    fields = [f'{n}:{str(round(v, 3)).ljust(7)}' for n, v in values.items()]
                    print(f'{shortName}: {"|".join(fields)}')

            sleep(MAIN_LOOP_INTERVAL)

    except KeyboardInterrupt:
        print("[INFO] KeyboardInterrupt: E-Stopping all nodes.")
        print("[WARNING] DO NOT FORCE KILL!!! SAVING FLIPPER POSITIONS IN 3 SECONDS!")
        if not mockFlippers:
            for node in all_nodes:
                node.call_estop()
            sleep(3)
            pos_monitor(flipper_devs)
            save_flippers(flipper_devs)

    finally:
        if not mockFlippers:
            print("[WARNING] DO NOT FORCE KILL!!! SAVING FLIPPER POSITIONS IN 3 SECONDS!")
            for node in all_nodes:
                node.set_state_msg(IDLE)
            sleep(3)
            pos_monitor(flipper_devs)
            save_flippers(flipper_devs)
            bus.shutdown()
        print("Application exited")


if __name__ == '__main__':
    main()
