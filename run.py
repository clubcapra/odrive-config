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
from typing import Generic, List, Dict, Protocol, Sequence, Tuple, Literal, TypeVar, Union
from asyncio import gather

import prompt_toolkit.completion
import prompt_toolkit.contrib
import prompt_toolkit.contrib.completers
import prompt_toolkit.contrib.regular_languages
import prompt_toolkit.utils
import prompt_toolkit.validation

from can_simple_utils import GET_ENCODER_ESTIMATES_CMD, CanSimpleNode
from xbox_controller import Axis, Button, ControllerBindings, XboxController

import prompt_toolkit

from control import AllInstruction, FakeFlipper, Flipper, OnExit, PairInstruction, SingleInstruction, load_flippers, save_flippers

from common import *


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


def msg_monitor(nodes: List[CanSimpleNode], flippers: Dict[Pos, Flipper], posEvents: Dict[Pos, threading.Event], bus: can.BusABC) -> None:
    """Listens for node errors and triggers E-Stop if any occur."""
    while True:
        msg = bus.recv()
        if msg:
            if (msg.arbitration_id & 0x1F) == 0x01:
                code = struct.unpack('<I', msg.data[:4])[0]
                if code != 0:
                    nid = msg.arbitration_id >> 5
                    print(f"[ERROR] Node {nid} error {code}: triggering E-Stop")
                    for node in nodes:
                        node.call_estop()
                    break
            elif (msg.arbitration_id & GET_ENCODER_ESTIMATES_CMD) == 0x01:
                pos, vel = struct.unpack('<ff', msg.data)
                nid = msg.arbitration_id >> 5
                for n, f in flippers.items():
                    if f.node.node_id == nid:
                        f._position = pos
                        f._velocity = vel
                        print(f"{nid} pos: {pos} vel: {vel}")
                        posEvents[n].set()
        

def clamp(val:float, lo:float=-1.0, hi:float=1.0) -> float:
    return max(min(val, hi), lo)


def handle_tracks(controller: XboxController, tracks: Dict[Side, List[CanSimpleNode]], enabled:bool) -> None:
    """
    Left stick X = throttle, Left stick Y = steering.
    """
    if enabled:
        throttle = -clamp(controller.LeftJoystickY.value)
        steering = -clamp(controller.LeftJoystickX.value)
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

U = TypeVar('U', covariant=True)
class MultiContext(Generic[U]):
    def __init__(self, contexts: List[U]):
        self.contexts = contexts
        
    def __enter__(self) -> List[U]:
        res = []
        for c in self.contexts:
            res.append(c.__enter__()) # type: ignore
        return res
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        for c in self.contexts:
            c.__exit__(exc_type, exc_val, exc_tb) # type: ignore

async def read_loop(reader: can.AsyncBufferedReader, nodes: List[CanSimpleNode]):
    async for msg in reader:
        for n in nodes:
            n.read_msg(msg)

async def read_positions_loop(nodes: List[CanSimpleNode]):
    i = 0
    while True:
        if i % 10 == 0:
            for n in nodes:
                n.get_encoder_estimates_msg()
                print(f'{n.node_id}: pos: {str(round(n.position, 2)).ljust(6)} vel: {str(round(n.velocity, 2)).ljust(6)}')
            i = 0
        await asyncio.sleep(MAIN_LOOP_INTERVAL)
        i+=1    

def read_positions():
    with init_can_bus() as bus:
        reader = can.AsyncBufferedReader()
        tracks, flippers = create_nodes(bus)
        all_nodes: List[CanSimpleNode] = tracks['left'] + tracks['right'] + list(flippers.values())
        # with MultiContext(all_nodes):
        async def run():
            notifier = can.Notifier(bus, [reader], loop=asyncio.get_running_loop())
            with MultiContext([]):
                try:
                    while True:
                        await asyncio.gather(read_positions_loop(all_nodes), read_loop(reader, all_nodes))
                except KeyboardInterrupt:
                    print("[INFO] KeyboardInterrupt: E-Stopping all nodes.")

                finally:
                    print("Application exited")
        asyncio.run(run())

    

async def control_main_loop(xbox: XboxController,
                            mockFlippers: bool, 
                            heartbeat: threading.Event, 
                            all_nodes: List[CanSimpleNode], 
                            flippers: Dict[Pos, CanSimpleNode],
                            tracks: Dict[Side, List[CanSimpleNode]],
                            flipper_devs: Dict[Pos, Flipper]):
    
    
    drive_enabled = False
    error_cleared = False
    while True:
        heartbeat.set()

        # E-Stop: bumpers
        if xbox.LeftBumper.state or xbox.RightBumper.state:
            for node in all_nodes:
                node.call_estop()
            drive_enabled = False

        # Toggle drive enable: A button
        if xbox.A.state and not drive_enabled:
            for node in all_nodes:
                node.set_state_msg(STATE_CLOSED_LOOP_CONTROL)
            drive_enabled = True
        elif (not xbox.A.state or not xbox.Connected) and drive_enabled:
            for node in all_nodes:
                node.set_state_msg(STATE_IDLE)
            drive_enabled = False

        # Clear errors: B button
        if xbox.B.state and not error_cleared:
            for node in all_nodes:
                node.clear_errors_msg()
            error_cleared = True
        elif not xbox.B.state:
            error_cleared = False

        for f in flipper_devs.values():
            f.update()
        for f in flipper_devs.values():
            f.run()
            
            
        if mockFlippers:
            for f in flippers.values():
                f.update() # type: ignore
            
        if not mockFlippers:
            handle_tracks(xbox, tracks, drive_enabled)
            for name, flipper1 in flipper_devs.items():
                if name != 'front_right':
                    continue
                shortName = ''.join([n[0] for n in name.split('_')]).upper()
                values: Dict[str, float] = {
                    '_p': flipper1._position,
                    '_s': flipper1._setPosition,
                    'P': flipper1.position,
                    'S': flipper1.setPosition,
                    '_v': flipper1._velocity,
                    '_z': flipper1._zero,
                }
                fields = [f'{n}:{str(round(v, 3)).ljust(8)}' for n, v in values.items()]
                print(f'{shortName}: {"|".join(fields)}')
        else:
            print()
            for name, flipper in flipper_devs.items():
                shortName = ''.join([n[0] for n in name.split('_')]).upper()
                values: Dict[str, float] = {
                    '_p': flipper._position,
                    '_s': flipper._setPosition,
                    'P': flipper.position,
                    'S': flipper.setPosition,
                    '_v': flipper._velocity,
                    '_z': flipper._zero,
                }
                fields = [f'{n}:{str(round(v, 3)).ljust(8)}' for n, v in values.items()]
                print(f'{shortName}: {"|".join(fields)}')

        await asyncio.sleep(MAIN_LOOP_INTERVAL)

async def control(xbox: XboxController, mockFlippers: bool):
    with MultiContext([]) if mockFlippers else init_can_bus() as ctx:
        if not mockFlippers:
            bus:can.BusABC = ctx # type: ignore
            tracks, flippers = create_nodes(bus)
            all_nodes = tracks['left'] + tracks['right'] + list(flippers.values())
            reader = can.AsyncBufferedReader()
            notifier = can.Notifier(
                bus,
                [reader],
                loop=asyncio.get_event_loop()
            )
        else:
            tracks:Dict[Side, List[CanSimpleNode]] = {side: list([FakeFlipper(58, 10) for _ in range(2)]) for side in ['left', 'right']}
            flippers: Dict[Pos, CanSimpleNode] = {
                'front_left' : FakeFlipper(58, 10),
                'rear_left' : FakeFlipper(58, 9),
                'front_right' : FakeFlipper(55, 10),
                'rear_right' : FakeFlipper(55, 9),
            }
            all_nodes = tracks['left'] + tracks['right'] + list(flippers.values())
        flipper_devs: Dict[Pos, Flipper] = {name: Flipper(node) for name, node in flippers.items()}
        frontInstruction = PairInstruction(xbox, 'front')
        rearInstruction = PairInstruction(xbox, 'rear')
        allIstruction = AllInstruction(xbox)
        for pos, flipper in flipper_devs.items():
            flipper.addInstruction(allIstruction)
            flipper.addInstruction(SingleInstruction(xbox, pos))
            if pos.startswith('front'):
                flipper.addInstruction(frontInstruction)
            else:
                flipper.addInstruction(rearInstruction)
        
        heartbeat = threading.Event()
        if not mockFlippers:
            threading.Thread(target=estop_monitor, args=(all_nodes, bus, heartbeat), daemon=True).start()
            # threading.Thread(target=msg_monitor, args=(all_nodes, flipper_devs, posEvents, bus), daemon=True).start()

        for node in all_nodes:
            node.clear_errors_msg()
            node.set_state_msg(STATE_IDLE)
        load_flippers(flipper_devs)

        async def onExit():
            if not mockFlippers:
                print("DO NOT KILL THE PROGRAM SAVING FLIPPER POSITIONS IN 3 SECCONDS!!!")
                try:
                    for node in all_nodes:
                        node.call_estop()
                except Exception as e:
                    print(f"Couldn't call estop: {e}")
                await asyncio.sleep(3)
                try:
                    # Read the messages for 1 second to get latest position
                    await asyncio.wait([read_loop(reader, all_nodes)], timeout=1)
                except asyncio.TimeoutError:
                    pass
                
                save_flippers(flipper_devs)

        try:
            async with OnExit(onExit):
                for flipper in flippers.values():
                    flipper.set_traj_vel_limit(58)
                while True:
                    if mockFlippers:
                        await asyncio.gather(
                            control_main_loop(xbox,
                                            mockFlippers,
                                            heartbeat,
                                            all_nodes,
                                            flippers,
                                            tracks,
                                            flipper_devs),
                        )
                    else:
                        await asyncio.gather(
                            read_loop(reader, all_nodes),
                            control_main_loop(xbox,
                                            mockFlippers,
                                            heartbeat,
                                            all_nodes,
                                            flippers,
                                            tracks,
                                            flipper_devs),
                        )

        except KeyboardInterrupt:
            print("[INFO] KeyboardInterrupt: E-Stopping all nodes.")

        finally:
            print("Application exited")

def main():
    mockFlippers = False
    xbox = XboxController()
    if len(sys.argv) >= 2:
        if 'mock' in sys.argv:
            mockFlippers = True
        if 'config' in sys.argv:
            if xbox.wait_for_connection():
                print("Could not connect to xbox controller")
                return
            select_bindings(xbox)
        if 'read' in sys.argv:
            read_positions()
            return
        if 'debug' in sys.argv:
            print("Waiting for xbox to connect")
            if xbox.wait_for_connection():
                print("Could not connect to xbox controller")
                return
            while True:
                actions = {
                    'LS X' : xbox.LeftJoystickX,
                    'LS Y' : xbox.LeftJoystickY,
                    'LT' : xbox.LeftTrigger,
                    'RT' : xbox.RightTrigger,
                    'LB' : xbox.LeftBumper,
                    'RB' : xbox.RightBumper,
                    'A' : xbox.A,
                    'X' : xbox.X,
                    'Y' : xbox.Y,
                    'B' : xbox.B,
                    'LS' : xbox.LeftThumb,
                    'RS' : xbox.RightThumb,
                    'Back' : xbox.Back,
                    'Start' : xbox.Start,
                    'Left' : xbox.LeftDPad,
                    'Right' : xbox.RightDPad,
                    'Up' : xbox.UpDPad,
                    'Down' : xbox.DownDPad,
                }
                fields = []
                print()
                for name, action in actions.items():
                    if isinstance(action, Axis):
                        print(f'{name} raw: {str(action.raw).ljust(5)} actual: {str(round(action.value, 3)).ljust(6)}')
                    if isinstance(action, Button):
                        fields.append(f'{name}:{1 if action.state else 0}')
                print('|'.join(fields))
                sleep(0.5)
            
        print("Usage: run.py [mock]")
    
    if xbox.wait_for_connection():
        asyncio.run(control(xbox, mockFlippers))
    else:
        print("Could not connect to xbox controller")
    


if __name__ == '__main__':
    main()
