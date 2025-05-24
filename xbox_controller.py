from __future__ import annotations
import asyncio
from dataclasses import dataclass
import math
from re import S
import threading
from typing import Any, Callable, Dict, Optional, Tuple, overload
import evdev
import json

import prompt_toolkit
import prompt_toolkit.contrib
import prompt_toolkit.data_structures
import prompt_toolkit.eventloop
import prompt_toolkit.layout
import prompt_toolkit.styles
import prompt_toolkit.utils
import prompt_toolkit.widgets

MAX_TRIG_VAL = 1_023
MAX_JOY_VAL = 32_768

def mapValue(fromMin, fromMax, toMin, toMax, value) -> float:
    return (value - fromMin) / (fromMax - fromMin) * (toMax - toMin) + toMin

@dataclass()
class Binding:
    etype: int = -1
    ecode: int = -1
    minValue: int = 0
    maxValue: int = 1
    deadzone: bool = False
    
    @staticmethod
    def from_event(event:evdev.InputEvent, minValue:int = 0, maxValue:int = 1, deadzone:bool = False) -> Binding:
        return Binding(event.type, event.code, minValue, maxValue, deadzone)
    
    @staticmethod
    def load(data: Dict[str, Any]) -> Binding:
        res = Binding()
        res.__dict__.update(data)
        return res
    
    def dump(self) -> Dict[str, Any]:
        return self.__dict__
    
@dataclass
class ControllerBindings:
    LeftJoystickY: Binding = Binding(evdev.ecodes.EV_ABS, evdev.ecodes.ABS_Y, MAX_JOY_VAL, -MAX_JOY_VAL, True)
    LeftJoystickX: Binding = Binding(evdev.ecodes.EV_ABS, evdev.ecodes.ABS_X, -MAX_JOY_VAL, MAX_JOY_VAL, True)
    RightJoystickY: Binding = Binding(evdev.ecodes.EV_ABS, evdev.ecodes.ABS_RY, MAX_JOY_VAL, -MAX_JOY_VAL, True)
    RightJoystickX: Binding = Binding(evdev.ecodes.EV_ABS, evdev.ecodes.ABS_RX, -MAX_JOY_VAL, MAX_JOY_VAL, True)
    LeftTrigger: Binding = Binding(evdev.ecodes.EV_ABS, evdev.ecodes.ABS_RZ, 0, MAX_TRIG_VAL)
    RightTrigger: Binding = Binding(evdev.ecodes.EV_ABS, evdev.ecodes.ABS_Z, 0, MAX_TRIG_VAL)
    DPadY: Binding = Binding(evdev.ecodes.EV_ABS, evdev.ecodes.ABS_HAT0X, 1, -1)
    DPadX: Binding = Binding(evdev.ecodes.EV_ABS, evdev.ecodes.ABS_HAT0Y, -1, 1)
    LeftBumper: Binding = Binding(evdev.ecodes.EV_KEY, evdev.ecodes.BTN_TL)
    RightBumper: Binding = Binding(evdev.ecodes.EV_KEY, evdev.ecodes.BTN_TR)
    A: Binding = Binding(evdev.ecodes.EV_KEY, evdev.ecodes.BTN_SOUTH)
    X: Binding = Binding(evdev.ecodes.EV_KEY, evdev.ecodes.BTN_WEST)
    Y: Binding = Binding(evdev.ecodes.EV_KEY, evdev.ecodes.BTN_NORTH)
    B: Binding = Binding(evdev.ecodes.EV_KEY, evdev.ecodes.BTN_EAST)
    LeftThumb: Binding = Binding(evdev.ecodes.EV_KEY, evdev.ecodes.BTN_THUMBL)
    RightThumb: Binding = Binding(evdev.ecodes.EV_KEY, evdev.ecodes.BTN_THUMBR)
    Back: Binding = Binding(evdev.ecodes.EV_KEY, evdev.ecodes.BTN_SELECT)
    Start: Binding = Binding(evdev.ecodes.EV_KEY, evdev.ecodes.BTN_START)
    LeftDPad: Binding = Binding(evdev.ecodes.EV_ABS, evdev.ecodes.ABS_HAT0X, 0, -1)
    RightDPad: Binding = Binding(evdev.ecodes.EV_ABS, evdev.ecodes.ABS_HAT0X, 0, 1)
    UpDPad: Binding = Binding(evdev.ecodes.EV_ABS, evdev.ecodes.ABS_HAT0Y, 0, -1)
    DownDPad: Binding = Binding(evdev.ecodes.EV_ABS, evdev.ecodes.ABS_HAT0Y, 0, 1)
    
    @staticmethod
    def load(data:Dict[str, Any]) -> ControllerBindings:
        res = ControllerBindings()
        res.__dict__.update({k: Binding.load(v) for k, v in data.items()})
        return res
    
    def dump(self) -> Dict[str, Any]:
        return {k: v.dump() for k, v in self.__dict__} # type: ignore
    
class Button:
    def __init__(self, binding:Binding = Binding()):
        self.binding = binding
        self.raw = 0
        self._lastRaw = 0
        
    @property
    def state(self) -> bool:
        return mapValue(self.binding.minValue, self.binding.maxValue, 0, 1, self.raw) > (self.binding.minValue + self.binding.maxValue) / 2
    
    @property
    def lastState(self) -> bool:
        return mapValue(self.binding.minValue, self.binding.maxValue, 0, 1, self._lastRaw) > (self.binding.minValue + self.binding.maxValue) / 2
    
    @property
    def pressed(self) -> bool:
        return self.state and not self.lastState
    
    @property
    def released(self) -> bool:
        return not self.state and self.lastState
    
    @property
    def held(self) -> bool:
        return self.state and self.lastState
    
    def update(self, event:evdev.InputEvent):
        if event.type == self.binding.etype and event.code == self.binding.ecode:
            self.raw = event.value

class Axis:
    def __init__(self, binding:Binding = Binding(), valueMin:float = -1.0, valueMax:float = 1.0):
        self.binding = binding
        self.raw = 0
        self.valueMin = valueMin
        self.valueMax = valueMax
        self.deadzone = 0.1
    
    def _apply_deadzone(self, value:float) -> float:
        if not self.binding.deadzone:
            return value
        if value > 0:
            return max(0, value - self.deadzone) / (1.0 - self.deadzone)
        else:
            return min(0, value + self.deadzone) / (1.0 - self.deadzone)
        
    @property
    def value(self) -> float:
        return self._apply_deadzone(mapValue(self.binding.minValue, self.binding.maxValue, self.valueMin, self.valueMax, self.raw))
    
    def update(self, event:evdev.InputEvent):
        if event.type == self.binding.etype and event.code == self.binding.ecode:
            self.raw = event.value
    
class XboxController(object):
    """
    XboxController class for interfacing with an Xbox controller using evdev.

    Attributes:
    - MAX_TRIG_VAL: Maximum trigger value.
    - MAX_JOY_VAL: Maximum joystick value.
    - LeftJoystickY: Y-axis value of the left joystick.
    - LeftJoystickX: X-axis value of the left joystick.
    - RightJoystickY: Y-axis value of the right joystick.
    - RightJoystickX: X-axis value of the right joystick.
    - LeftTrigger: Value of the left trigger.
    - RightTrigger: Value of the right trigger.
    - LeftBumper: State of the left bumper button.
    - RightBumper: State of the right bumper button.
    - A: State of the A button.
    - X: State of the X button.
    - Y: State of the Y button.
    - B: State of the B button.
    - LeftThumb: State of the left thumbstick button.
    - RightThumb: State of the right thumbstick button.
    - Back: State of the back button.
    - Start: State of the start button.
    - LeftDPad: State of the left direction pad button.
    - RightDPad: State of the right direction pad button.
    - UpDPad: State of the up direction pad button.
    - DownDPad: State of the down direction pad button.
    """

    MAX_TRIG_VAL = 1_023
    MAX_JOY_VAL = 32_768

    def __init__(self, bindings: ControllerBindings=ControllerBindings(), deadzone=0.1):
        self._deadzone = deadzone
        """
        Initializes the XboxController object and starts the monitoring thread.
        """
        devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        self.device_path = ""
        for device in devices:
            if device.name in ["Xbox Wireless Controller", "Microsoft Xbox Series S|X Controller"]:
                self.device_path = device.path
                break

        if self.device_path == "":
            print("Controller disconnected. Exiting the controller monitor thread.")
            self.Connected = False
            return

        self.device:evdev.InputDevice = evdev.InputDevice(self.device_path)

        
        self.load_bindings(bindings)
        self.Connected = False

        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()
        print("Started XBox controller manager")

    def load_bindings(self, bindings:ControllerBindings):
        self.bindings = bindings
        self.LeftJoystickY = Axis(bindings.LeftJoystickY)
        self.LeftJoystickX = Axis(bindings.LeftJoystickX)
        self.RightJoystickY = Axis(bindings.RightJoystickY)
        self.RightJoystickX = Axis(bindings.RightJoystickX)
        self.LeftTrigger = Axis(bindings.LeftTrigger)
        self.RightTrigger = Axis(bindings.RightTrigger)
        # self.DPadY = Axis(bindings.DPadY)
        # self.DPadX = Axis(bindings.DPadX)
        self.axes = {
            "LeftJoystickY" : self.LeftJoystickY,
            "LeftJoystickX" : self.LeftJoystickX,
            "RightJoystickY" : self.RightJoystickY,
            "RightJoystickX" : self.RightJoystickX,
            "LeftTrigger" : self.LeftTrigger,
            "RightTrigger" : self.RightTrigger,
            # "DPadY" : self.DPadY,
            # "DPadX" : self.DPadX,
        }
        
        self.LeftBumper = Button(bindings.LeftBumper)
        self.RightBumper = Button(bindings.RightBumper)
        self.A = Button(bindings.A)
        self.X = Button(bindings.X)
        self.Y = Button(bindings.Y)
        self.B = Button(bindings.B)
        self.LeftThumb = Button(bindings.LeftThumb)
        self.RightThumb = Button(bindings.RightThumb)
        self.Back = Button(bindings.Back)
        self.Start = Button(bindings.Start)
        self.LeftDPad = Button(bindings.LeftDPad)
        self.RightDPad = Button(bindings.RightDPad)
        self.UpDPad = Button(bindings.UpDPad)
        self.DownDPad = Button(bindings.DownDPad)
        
        self.buttons = {
            "LeftBumper" : self.LeftBumper,
            "RightBumper" : self.RightBumper,
            "A" : self.A,
            "X" : self.X,
            "Y" : self.Y,
            "B" : self.B,
            "LeftThumb" : self.LeftThumb,
            "RightThumb" : self.RightThumb,
            "Back" : self.Back,
            "Start" : self.Start,
            "LeftDPad" : self.LeftDPad,
            "RightDPad" : self.RightDPad,
            "UpDPad" : self.UpDPad,
            "DownDPad" : self.DownDPad,
        }
        
        self.LeftTriggerBtn = Button(Binding(self.LeftTrigger.binding.etype, self.LeftTrigger.binding.ecode, 0, self.MAX_TRIG_VAL))
        self.RightTriggerBtn = Button(Binding(self.RightTrigger.binding.etype, self.RightTrigger.binding.ecode, 0, self.MAX_TRIG_VAL))
        
    
    def learn(self, ):
        def prompt_confirm(event:evdev.InputEvent) -> bool:
            for etype, val in evdev.ecodes.ecodes.items():
                if etype.startswith('EV') and val == event.type:
                    break
            for ecode, val in evdev.ecodes.ecodes.items():
                EV_MAP = {
                    'KEY': 'BTN',
                    'ABS': 'ABS',
                }
                prefix = EV_MAP[etype.split('_')[-1]]
                if val == event.code:
                    if ecode.startswith(prefix):
                        break
            return prompt_toolkit.shortcuts.confirm("Input detected", f"Found etype: {etype} ecode: {ecode}")
        
        def wait_for_btn(restrict:bool = False) -> Optional[evdev.InputEvent]:
            for event in self.device.read_loop():
                event:evdev.InputEvent
                if (restrict and event.type == evdev.ecodes.EV_KEY) or (not restrict and event.type in [evdev.ecodes.EV_ABS, evdev.ecodes.EV_KEY] and event.value != 0):
                    return event
            return None
        
        def wait_for_axis(thresh = 200) -> Optional[evdev.InputEvent]:
            for event in self.device.read_loop():
                event:evdev.InputEvent
                if event.type == evdev.ecodes.EV_ABS and abs(event.value) >= thresh:
                    return event
            return None

        def get_axis_range(binding:Binding):
            actualLbl = prompt_toolkit.widgets.Label("Actual value: 0")
            minLbl = prompt_toolkit.widgets.Label("Min value: 0")
            maxLbl = prompt_toolkit.widgets.Label("Max value: 0")
            slider = prompt_toolkit.widgets.ProgressBar()
            inverted = [False]
            def toggle():
                inverted[0] = not inverted[0]
            
            def quit():
                prompt_toolkit.application.get_app().exit()
            
            invert = prompt_toolkit.widgets.Button("Invert", toggle)
            okBtn = prompt_toolkit.widgets.Button("Ok", quit)
            
            hsplit = prompt_toolkit.layout.HSplit([actualLbl, minLbl, maxLbl, slider, invert, okBtn])
            frame = prompt_toolkit.widgets.Frame(hsplit, "Get axis range")
            layout = prompt_toolkit.layout.Layout(frame)
            app = prompt_toolkit.Application(layout)
            
            async def update_values():
                binding.minValue = 0
                binding.maxValue = 0
                while True:
                    event = await self.device.async_read_one()
                    if event.type == binding.etype and event.code == binding.ecode:
                        binding.minValue = min(binding.minValue, event.value)
                        binding.maxValue = max(binding.maxValue, event.value)
                        actualLbl.text = f"Actual value: {event.value}"
                        minLbl.text = f"Min value: {binding.minValue}"
                        maxLbl.text = f"Max value: {binding.maxValue}"
                        if inverted[0]:
                            slider.percentage = int(mapValue(binding.minValue, binding.maxValue, 100, 0, event.value))
                        else:
                            slider.percentage = int(mapValue(binding.minValue, binding.maxValue, 0, 100, event.value))
            
            app.create_background_task(update_values())
            app.run()
            
            if inverted[0]:
                tmp = binding.minValue
                binding.minValue = binding.maxValue
                binding.maxValue = tmp
            

        for name in self.buttons.keys():
            while True:
                prompt_toolkit.print_formatted_text("Press on button", name)
                if 'DPad' in name:
                    ev = wait_for_axis(1)
                else:    
                    ev = wait_for_btn('Thumb' in name)
                    
                    
                if ev is None:
                    prompt_toolkit.print_formatted_text("Exitting")
                    return
                if prompt_confirm(ev):
                    break
                
            
            self.bindings.__dict__[name].etype = ev.type
            self.bindings.__dict__[name].ecode = ev.code
            self.bindings.__dict__[name].minValue = 0
            self.bindings.__dict__[name].maxValue = ev.value
            
        for name in self.axes.keys():
            if 'DPad' in name:
                continue
            while True:
                prompt_toolkit.print_formatted_text("Move axis", name)
                ev = wait_for_axis()
                if ev is None:
                    prompt_toolkit.print_formatted_text("Exitting")
                    return
                if prompt_confirm(ev):
                    break
            
            self.bindings.__dict__[name].etype = ev.type
            self.bindings.__dict__[name].ecode = ev.code
            get_axis_range(self.bindings.__dict__[name])
            
        self.load_bindings(self.bindings)
            
            
    
    def _monitor_controller(self):
        """
        Monitors the Xbox controller for input events and updates the attributes accordingly.
        """
        try:
            self.Connected = True
            for event in self.device.read_loop():
                # if event.type == evdev.ecodes.EV_ABS:
                #     if event.code == evdev.ecodes.ABS_Y:
                #         self.LeftJoystickY = self._apply_deadzone((-event.value + XboxController.MAX_JOY_VAL) / XboxController.MAX_JOY_VAL)
                #     elif event.code == evdev.ecodes.ABS_X:
                #         self.LeftJoystickX = self._apply_deadzone((event.value - XboxController.MAX_JOY_VAL) / XboxController.MAX_JOY_VAL)
                #     elif event.code == evdev.ecodes.ABS_RY:
                #         self.RightJoystickY = self._apply_deadzone((-event.value + XboxController.MAX_JOY_VAL) / XboxController.MAX_JOY_VAL)
                #     elif event.code == evdev.ecodes.ABS_RX:
                #         self.RightJoystickX = self._apply_deadzone((event.value - XboxController.MAX_JOY_VAL) / XboxController.MAX_JOY_VAL)
                #     # elif event.code == evdev.ecodes.ABS_RZ:
                #     #     self.RightTrigger = event.value / XboxController.MAX_TRIG_VAL
                #     # elif event.code == evdev.ecodes.ABS_Z:
                #     #     self.LeftTrigger = event.value / XboxController.MAX_TRIG_VAL
                #     elif event.code == evdev.ecodes.ABS_GAS:
                #         self.RightTrigger = event.value / XboxController.MAX_TRIG_VAL
                #     elif event.code == evdev.ecodes.ABS_BRAKE:
                #         self.LeftTrigger = event.value / XboxController.MAX_TRIG_VAL
                #     elif event.code == evdev.ecodes.ABS_HAT0X:
                #         if event.value == 1:
                #             self.RightDPad = 1
                #         elif event.value == -1:
                #             self.LeftDPad = 1
                #         elif event.value == 0:
                #             self.RightDPad = self.LeftDPad = 0
                #     elif event.code == evdev.ecodes.ABS_HAT0Y:
                #         if event.value == 1:
                #             self.DownDPad = 1
                #         elif event.value == -1:
                #             self.UpDPad = 1
                #         elif event.value == 0:
                #             self.UpDPad = self.DownDPad = 0
                # elif event.type == evdev.ecodes.EV_KEY:
                #     if event.code == evdev.ecodes.BTN_TL:
                #         self.LeftBumper = event.value
                #     elif event.code == evdev.ecodes.BTN_TR:
                #         self.RightBumper = event.value
                #     elif event.code == evdev.ecodes.BTN_SOUTH:
                #         self.A = event.value
                #     elif event.code == evdev.ecodes.BTN_WEST:
                #         self.Y = event.value
                #     elif event.code == evdev.ecodes.BTN_NORTH:
                #         self.X = event.value
                #     elif event.code == evdev.ecodes.BTN_EAST:
                #         self.B = event.value
                #     elif event.code == evdev.ecodes.BTN_THUMBL:
                #         self.LeftThumb = event.value
                #     elif event.code == evdev.ecodes.BTN_THUMBR:
                #         self.RightThumb = event.value
                #     elif event.code == evdev.ecodes.BTN_SELECT:
                #         self.Back = event.value
                #     elif event.code == evdev.ecodes.BTN_START:
                #         self.Start = event.value
                for btn in self.buttons.values():
                    btn.update(event)
                for axis in self.axes.values():
                    axis.update(event)
                self.LeftTriggerBtn.update(event)
                self.RightTriggerBtn.update(event)
        except OSError:
            print("Controller disconnected. Exiting the controller monitor thread.")
            self.Connected = False

    def _apply_deadzone(self, value):
        if value > 0:
            return max(0, value - self._deadzone) / (1.0 - self._deadzone)
        else:
            return min(0, value + self._deadzone) / (1.0 - self._deadzone)
