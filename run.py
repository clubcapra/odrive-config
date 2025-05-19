import can
import struct
import threading
import time
from time import sleep
from typing import List, Dict, Tuple, Literal

from can_simple_utils import CanSimpleNode
from xbox_controller import XboxController

# Control modes
CLOSED_LOOP_CONTROL = 8
IDLE = 1

# Drive parameters
MAX_TRACK_SPEED = 58        # rev/s
FLIPPER_SPEED = 58.0         # rev/s
MAIN_LOOP_INTERVAL = 0.1    # seconds
WATCHDOG_INTERVAL = 1.0     # seconds
TEMP_LOG_INTERVAL = 2.0     # seconds

Side = Literal['left', 'right']

# CAN node IDs
TRACK_IDS: Dict[Side, List[int]] = {
    'left':  [21, 22],
    'right': [23, 24],
}

Pos = Literal['front_left', 'rear_left', 'front_right', 'rear_right']

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


def handle_flippers(controller: XboxController, flippers: Dict[Pos, CanSimpleNode]) -> None:
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

    if len(positions) == 0:
        positions = [
            'front_left',
            'rear_left',
            'front_right',
            'rear_right',
        ]
    if controller.UpDPad:
        vel = FLIPPER_SPEED
    elif controller.DownDPad:
        vel = -FLIPPER_SPEED
    else:
        vel = 0.0

    for p in positions:
        flippers[p].set_velocity(vel)


def main():
    bus = init_can_bus()
    tracks, flippers = create_nodes(bus)
    all_nodes = tracks['left'] + tracks['right'] + list(flippers.values())

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
            heartbeat.set()

            # E-Stop: bumpers
            if xbox.LeftBumper or xbox.RightBumper:
                for node in all_nodes:
                    node.call_estop()
                drive_enabled = False

            # Toggle drive enable: A button
            if xbox.A and not drive_enabled:
                for node in all_nodes:
                    node.set_state_msg(CLOSED_LOOP_CONTROL)
                drive_enabled = True
            elif (not xbox.A or not xbox.Connected) and drive_enabled:
                for node in all_nodes:
                    node.set_state_msg(IDLE)
                drive_enabled = False

            # Clear errors: B button
            if xbox.B and not error_cleared:
                for node in all_nodes:
                    node.clear_errors_msg()
                error_cleared = True
            elif not xbox.B:
                error_cleared = False

            handle_tracks(xbox, tracks, drive_enabled)
            handle_flippers(xbox, flippers)

            sleep(MAIN_LOOP_INTERVAL)

    except KeyboardInterrupt:
        print("[INFO] KeyboardInterrupt: E-Stopping all nodes.")
        for node in all_nodes:
            node.call_estop()

    finally:
        for node in all_nodes:
            node.set_state_msg(IDLE)
        bus.shutdown()
        print("Application exited")


if __name__ == '__main__':
    main()
