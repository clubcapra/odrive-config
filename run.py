import asyncio
import sys
import can
import struct
import threading
import time
from time import sleep
from typing import List, Dict, Tuple, Literal, Union
from asyncio import gather

from can_simple_utils import CanSimpleNode
from xbox_controller import XboxController

from control import Device, Flipper, MockFlipper, Pair, load_flippers, save_flippers

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
        throttle = -clamp(controller.LeftJoystickX)
        steering = clamp(controller.LeftJoystickY)
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


def handle_flippers(controller: XboxController, flippers: Dict[Pos, Flipper], front_pair: Pair, rear_pair: Pair, all_pair: Pair, enabled:bool) -> None:
    """Sets flipper velocities based on D-pad Up/Down."""
    positions: List[Pos] = []
    if controller.LeftBumper:
        positions.append('front_left')
    if controller.LeftTrigger:
        positions.append('rear_left')
    if controller.RightBumper:
        positions.append('front_right')
    if controller.RightTrigger:
        positions.append('rear_right')

    is_all = len(positions) == 4
    is_front = len(positions) == 2 and all(['front' in pos for pos in positions])
    is_rear = len(positions) == 2 and all(['front' in pos for pos in positions])
    
    if len(positions) == 0:
        positions = [
            'front_left',
            'rear_left',
            'front_right',
            'rear_right',
        ]
    
    def _move(pos: float):
        if enabled:
            if is_all:
                # Control each individually (RB, RT, LB, LT all being held)
                for flipper in flippers.values():
                    flipper.move(pos)
            elif len(positions) == 4:
                # Control all together 
                all_pair.move(pos)
            elif is_front:
                # Control front together
                front_pair.move(pos)
                rear_pair.move(0) # Stop rear
            elif is_rear:
                # Control rear together
                rear_pair.move(pos)
                front_pair.move(0) # Stop front
            elif len(positions) == 1:
                # Control a single flipper
                for name, flipper in flippers.items():
                    if name in positions:
                        flipper.move(pos)
                    else:
                        flipper.move(0) # Stop others
        else:
            all_pair.move(0) # Stop all
    
    if controller.UpDPad:
        _move(FLIPPER_MOVE_OFFSET)
    elif controller.DownDPad:
        _move(-FLIPPER_MOVE_OFFSET)
    elif controller.LeftDPad and controller.Y and not enabled:
        if is_all:
            # Set main pair zero (RB, RT, LB, LT all being held)
            all_pair.zero()
        elif len(positions) == 4:
            # Set all individual zeros
            for flipper in flippers.values():
                flipper.zero()
        elif is_front:
            # Set front offset
            front_pair.zero()
        elif is_rear:
            # Set rear offset
            rear_pair.zero()
        elif len(positions) == 1:
            # Set specific flipper offset
            for name, flipper in flippers.items():
                if name in positions:
                    flipper.zero()
    elif controller.RightDPad:
        if enabled:
            if is_all:
                # Return all to individual zero (RB, RT, LB, LT all being held)
                for flipper in flippers.values():
                    flipper.go_home()
            elif len(positions) == 4:
                # Return front and rear to offset
                all_pair.go_home()
            elif is_front:
                # Return front to offset
                front_pair.go_home()
                rear_pair.move(0) # Stop rear
            elif is_rear:
                # Return rear to offset
                rear_pair.go_home()
                front_pair.move(0) # Stop front
            elif len(positions) == 1:
                # Set specific flipper offset
                for name, flipper in flippers.items():
                    if name in positions:
                        flipper.go_home()
        else:
            all_pair.move(0) # Stop all
    else:
        all_pair.move(0)

def main():
    mockFlippers = False
    if len(sys.argv) >= 2:
        if sys.argv[1] == 'mock':
            mockFlippers = True
        else:
            print("Usage: run.py [mock]")
            return
        
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
    front_pair: Pair = Pair(flipper_devs['front_left'], flipper_devs['front_right'])
    rear_pair: Pair = Pair(flipper_devs['rear_left'], flipper_devs['rear_right'])
    all_pair: Pair = Pair(front_pair, rear_pair)

    if not mockFlippers:
        heartbeat = threading.Event()

        threading.Thread(target=estop_monitor, args=(all_nodes, bus, heartbeat), daemon=True).start()
        threading.Thread(target=error_monitor, args=(all_nodes, bus), daemon=True).start()

        for node in all_nodes:
            node.clear_errors_msg()
            node.set_state_msg(IDLE)

    xbox = XboxController()
    drive_enabled = False
    error_cleared = False

    try:
        while True:
            if not MockFlipper:
                heartbeat.set()

            # E-Stop: bumpers
            if xbox.LeftBumper or xbox.RightBumper:
                if not MockFlipper:
                    for node in all_nodes:
                        node.call_estop()
                drive_enabled = False

            # Toggle drive enable: A button
            if xbox.A and not drive_enabled:
                if not MockFlipper:
                    for node in all_nodes:
                        node.set_state_msg(CLOSED_LOOP_CONTROL)
                drive_enabled = True
            elif (not xbox.A or not xbox.Connected) and drive_enabled:
                if not MockFlipper:
                    for node in all_nodes:
                        node.set_state_msg(IDLE)
                drive_enabled = False

            # Clear errors: B button
            if xbox.B and not error_cleared:
                if not MockFlipper:
                    for node in all_nodes:
                        node.clear_errors_msg()
                error_cleared = True
            elif not xbox.B:
                error_cleared = False

            pos_monitor(flipper_devs)
            handle_flippers(xbox, flipper_devs, front_pair, rear_pair, all_pair, drive_enabled)
            if not MockFlipper:
                handle_tracks(xbox, tracks, drive_enabled)
            else:
                for name, flipper in flipper_devs.items():
                    shortName = ''.join([n[0] for n in name.split('_')]).upper()
                    values: Dict[str, float] = {
                        '_p': flipper._position,
                        '_s': flipper._setposition,
                        'P': flipper.position,
                        'S': flipper.setposition,
                        '_o': flipper._offset,
                        '_v': flipper._velocity,
                    }
                    print()
                    fields = [f'{n}:{str(round(v, 3)).ljust(7)}' for n, v in values.items()]
                    print(f'{shortName}: {"|".join(fields)}')

            sleep(MAIN_LOOP_INTERVAL)

    except KeyboardInterrupt:
        print("[INFO] KeyboardInterrupt: E-Stopping all nodes.")
        print("[WARNING] DO NOT FORCE KILL!!! SAVING FLIPPER POSITIONS IN 3 SECONDS!")
        if not MockFlipper:
            for node in all_nodes:
                node.call_estop()
            sleep(3)
            pos_monitor(flipper_devs)
            save_flippers(flipper_devs)

    finally:
        if not MockFlipper:
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
