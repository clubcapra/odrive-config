import can
import struct
import threading
import time
from time import sleep

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

# CAN node IDs
TRACK_IDS = {
    'left':  [21, 22],
    'right': [23, 24],
}
FLIPPER_IDS = [11,12,13,14]
GET_TEMPERATURE_CMD = 0x15


def init_can_bus(channel='can0', bitrate=500000):
    print("Starting can bus")
    bus = can.interface.Bus(channel=channel, interface='socketcan', bitrate=bitrate)
    # flush pending frames
    while bus.recv(timeout=0):
        pass
    print("Done")
    return bus


def create_nodes(bus):
    print("Creating nodes")
    tracks = {side: [CanSimpleNode(bus, nid) for nid in ids]
              for side, ids in TRACK_IDS.items()}
    flippers = [CanSimpleNode(bus, nid) for nid in FLIPPER_IDS]
    print("Done")
    return tracks, flippers


def estop_monitor(nodes, bus, heartbeat):
    """Shuts down motors if heartbeat is missed."""
    while True:
        heartbeat.clear()
        if not heartbeat.wait(timeout=WATCHDOG_INTERVAL):
            print("[ERROR] No heartbeat: triggering E-Stop")
            for node in nodes:
                node.call_estop()


def error_monitor(nodes, bus):
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


def clamp(val, lo=-1.0, hi=1.0):
    return max(min(val, hi), lo)


def handle_tracks(controller, tracks, enabled):
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


def handle_flippers(controller, flippers):
    """Sets flipper velocities based on D-pad Up/Down."""
    if controller.UpDPad:
        vel = FLIPPER_SPEED
    elif controller.DownDPad:
        vel = -FLIPPER_SPEED
    else:
        vel = 0.0

    for node in flippers:
        node.set_velocity(vel)


def main():
    bus = init_can_bus()
    tracks, flippers = create_nodes(bus)
    all_nodes = tracks['left'] + tracks['right'] + flippers

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
