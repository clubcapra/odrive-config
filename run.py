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
from typing import Any, Callable, Generic, List, Dict, Protocol, Sequence, Tuple, Literal, TypeVar, Union
from asyncio import gather

import prompt_toolkit.buffer
import prompt_toolkit.completion
import prompt_toolkit.contrib
import prompt_toolkit.contrib.completers
import prompt_toolkit.contrib.regular_languages
import prompt_toolkit.enums
import prompt_toolkit.input
import prompt_toolkit.key_binding
import prompt_toolkit.layout
import prompt_toolkit.utils
import prompt_toolkit.validation
import prompt_toolkit.widgets

from can_simple_utils import CanSimpleNode
from odrive_types import ODriveAxisState, ODriveControlMode, ODriveInputMode
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
        node.set_velocity(-right_speed)

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

async def command_loop(flipper_devs: Dict[Pos, Flipper]):
    from prompt_toolkit.application import Application
    from prompt_toolkit.key_binding import KeyBindings
    from prompt_toolkit.layout import Layout, HSplit, VSplit
    from prompt_toolkit.widgets import Button, Label, TextArea, Box, Frame
    from prompt_toolkit.layout.controls import FormattedTextControl
    from prompt_toolkit.layout.containers import Window
    from prompt_toolkit.styles import Style
    from prompt_toolkit.formatted_text import HTML
    

    def is_float(val:str) -> bool:
        if val == "":
            return True
        try:
            float(val)
            return True
        except:
            return False

    def field(name: str, onSubmit: Callable[[TextArea], Any], onRead: Callable[[TextArea], float]):
        return TextArea(height=1, multiline=False, focus_on_click=True, validator=prompt_toolkit.validation.Validator.from_callable(is_float), prompt=f'{name}: '), onSubmit, onRead

    def set_value(func: Callable[[Flipper], Any]):
        for f in flipper_devs.values():
            func(f)
    fields = [
        field("inertia", lambda t: set_value(lambda f: f.node.set_inertia(float(t.text))), lambda t: flipper_devs['front_left'].node.inertia),
        field("trap_vel", lambda t: set_value(lambda f:  setattr(f.node, 'trap_vel', float(t.text))), lambda t: flipper_devs['front_left'].node.trap_vel),
        field("trap_accel", lambda t: set_value(lambda f: setattr(f.node, 'trap_accel', float(t.text))), lambda t: flipper_devs['front_left'].node.trap_accel),
        field("trap_decel", lambda t: set_value(lambda f: setattr(f.node, 'trap_decel', float(t.text))), lambda t: flipper_devs['front_left'].node.trap_decel),
    ]
    
    # --- UI State ---
    output_label = Label(text="")

    # --- Submit Button ---
    def on_submit():
        try:
            for v in fields:
                v[1](v[0])
            for f in flipper_devs.values():
                f.node.set_trap_config(f.node.trap_vel, f.node.trap_accel, f.node.trap_decel)
            output_label.text = "updated successfully."
        except ValueError:
            output_label.text = "Invalid input. Speed and Angle must be floats."

    submit_button = Button(text="Submit", handler=on_submit)

    # --- Quit ---
    def exit_app():
        app.exit()

    quit_button = Button(text="Quit", handler=exit_app)

    # --- Layout ---
    layout = Layout(
        HSplit([
            Label(text="Select a Flipper:"),
            Frame(Box(HSplit([
                *[f[0] for f in fields],
                VSplit([submit_button, quit_button], padding=2),
                output_label,
            ]), padding=1, style="bg:#222222")),
        ])
    )

    focusable_elements = [*[f[0] for f in fields], submit_button, quit_button]

    # --- App ---
    kb = KeyBindings()

    @kb.add("c-c")
    def _(event):
        event.app.exit()

    @kb.add("tab")
    def _(event):
        current = event.app.layout.current_control
        for i, element in enumerate(focusable_elements):
            if current == element.control:
                next_index = (i + 1) % len(focusable_elements)
                event.app.layout.focus(focusable_elements[next_index])
                break

    @kb.add("s-tab")
    def _(event):
        current = event.app.layout.current_control
        for i, element in enumerate(focusable_elements):
            if current == element.control:
                prev_index = (i - 1) % len(focusable_elements)
                event.app.layout.focus(focusable_elements[prev_index])
                break

    style = Style.from_dict({
        "button": "bg:#444444 #ffffff",
        "frame.label": "bg:#888888 #000000",
    })

    app = Application(layout=layout, 
                      key_bindings=kb, 
                      style=style, 
                      editing_mode=prompt_toolkit.enums.EditingMode.EMACS,
                      refresh_interval=1)
    await app.run_async()

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
        if xbox.Start.state or xbox.Back.state:
            for node in all_nodes:
                node.call_estop()
            drive_enabled = False

        # Toggle drive enable: A button
        if xbox.A.state and not drive_enabled:
            for node in all_nodes:
                node.set_state_msg(ODriveAxisState.CLOSED_LOOP_CONTROL)
            drive_enabled = True
        elif (not xbox.A.state or not xbox.Connected) and drive_enabled:
            for node in all_nodes:
                node.set_state_msg(ODriveAxisState.IDLE)
            drive_enabled = False
        if drive_enabled:
            for flipper in flipper_devs.values():
                # if flipper.node.state == ODriveAxisState.IDLE and abs(flipper.setPosition - flipper.position) > 0.5:
                # if flipper.node.state == ODriveAxisState.IDLE:
                flipper.node.set_state_msg(ODriveAxisState.CLOSED_LOOP_CONTROL)

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
            # for name, flipper1 in flipper_devs.items():
            #     if name != 'front_right':
            #         continue
            #     shortName = ''.join([n[0] for n in name.split('_')]).upper()
            #     values: Dict[str, float] = {
            #         '_p': flipper1._position,
            #         '_s': flipper1._setPosition,
            #         'P': flipper1.position,
            #         'S': flipper1.setPosition,
            #         '_v': flipper1._velocity,
            #         '_z': flipper1._zero,
            #     }
            #     fields = [f'{n}:{str(round(v, 3)).ljust(8)}' for n, v in values.items()]
            #     print(f'{shortName}: {"|".join(fields)}')
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

async def logger_loop(all_nodes: List[CanSimpleNode]):
    fields = [
        'voltage',
        'current',
        'torque',
        'velocity',
        'motor_temp',
        'fet_temp',
    ]
    with MultiContext([Logger(f'node_{node.node_id}', *fields) for node in all_nodes]) as loggers:
        while True:
            for node, logger in zip(all_nodes, loggers):
                logger.entry(node.voltage, node.current, node.torque, node.velocity, node.motorTemperature, node.fetTemperature)
            await asyncio.sleep(1.0)

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
            node.set_state_msg(ODriveAxisState.IDLE)
        
        load_flippers(flipper_devs)
        
        for flipper in flippers.values():
            flipper.set_controller_mode(ODriveControlMode.MODE_POSITION_CONTROL, ODriveInputMode.INPUT_POS_FILTER)
            # flipper.set_controller_mode(ODriveControlMode.MODE_POSITION_CONTROL, ODriveInputMode.INPUT_PASSTHROUGH)
            # flipper.set_controller_mode(ODriveControlMode.MODE_POSITION_CONTROL, ODriveInputMode.INPUT_TRAP_TRAJ)
            # flipper.set_inertia(0.0)
            # flipper.set_trap_config(55.0, 120.0, 120.0)

        async def onExit():
            if not mockFlippers:
                flipper_devs['front_right'].node.close_log()
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
                while True:
                    if mockFlippers:
                        await asyncio.gather(
                            xbox.read_async(),
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
                            xbox.read_async(),
                            read_loop(reader, all_nodes),
                            control_main_loop(xbox,
                                            mockFlippers,
                                            heartbeat,
                                            all_nodes,
                                            flippers,
                                            tracks,
                                            flipper_devs),
                            # command_loop(flipper_devs),
                            logger_loop(all_nodes),
                        )

        except KeyboardInterrupt:
            print("[INFO] KeyboardInterrupt: E-Stopping all nodes.")

        finally:
            print("Application exited")
            xbox.logger.close()

def main():
    mockFlippers = False
    xbox = XboxController(startThread=False)
    if len(sys.argv) >= 2:
        if 'mock' in sys.argv:
            mockFlippers = True
        if 'config' in sys.argv:
            xbox._start_thread = True
            if xbox.wait_for_connection():
                print("Could not connect to xbox controller")
                return
            select_bindings(xbox)
        if 'read' in sys.argv:
            read_positions()
            return
        if 'debug' in sys.argv:
            print("Waiting for xbox to connect")
            xbox._start_thread = True
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
    
    asyncio.run(control(xbox, mockFlippers))
    


if __name__ == '__main__':
    main()
