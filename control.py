from __future__ import annotations
from asyncio import wait
import asyncio
from datetime import datetime
from pathlib import Path
from statistics import mean
from typing import Callable, Dict, Generic, Iterable, Optional, Sequence, Tuple, TypeAlias
import typing
import can
from typing_extensions import override
from can_simple_utils import CanSimpleNode
from abc import abstractmethod, ABC
from json import dump, load

from odrive_error_codes import get_error_description
import odrive_error_codes
from xbox_controller import XboxController
from common import *

"""Device:
Data:
position: Actual position
setpoint: Actual current setpoint
enable: Activate the movement

Actions:


"""

class FakeFlipper(CanSimpleNode):
    def __init__(self, maxVelocity: float, maxAcceleration: float):
        self._maxVelocity = maxVelocity
        self._maxAcceleration = maxAcceleration
        self._position: float = 0.0
        self._velocity: float = 0.0
        self._setPosition: float = 0.0
        self._lastUpdate: Optional[datetime] = None
        self.enable = False
        self.connected: bool = False

    def __enter__(self) -> CanSimpleNode:

        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        pass
    
    def flush_rx(self) -> None:
        pass

    def await_msg(self, cmd_id: int, timeout=1.0):
        async def _impl():    
            return None
        return asyncio.wait_for(_impl(), timeout)

    def update(self):
        if self._lastUpdate is None or not self.enable:
            self._lastUpdate = datetime.now()
            return
        now = datetime.now()
        deltaTime = now - self._lastUpdate
        self._lastUpdate = now
        
        if self._position < self._setPosition:
            self._position = min(self._setPosition, self._position + self._maxVelocity * deltaTime.total_seconds())
        elif self._position > self._setPosition:
            self._position = max(self._setPosition, self._position - self._maxVelocity * deltaTime.total_seconds())
            

    def clear_errors_msg(self, identify: bool = False) -> None:
        pass
    def reboot_msg(self, action: int):
        pass

    def getErrorDescription(self, error_code: int):
        desc = get_error_description(error_code)
        print(f"CAN {self.node_id} Error Code: {error_code} - {desc}")
        self.clear_errors_msg()

    def set_state_msg(self, state: int):
        CLOSED_LOOP_CONTROL = 8
        IDLE = 1
        if state == IDLE:
            self.enable = False
        if state == CLOSED_LOOP_CONTROL:
            self.enable = True

    def wait_state(self, stateWaited: int, msg: can.Message) -> bool:
        return True

    def set_velocity(self, vel: float) -> None:
        self._velocity = vel

    def set_position(self, pos: float, vel_feedforward: float = 0.0, torque_feedforward: float = 0.0) -> None:
        self._setPosition = pos

    def call_estop(self) -> None:
        self.enable = False

    # ----- Newly added getters for feedback -----

    def get_encoder_estimates_msg(self) -> None:
        pass

    async def get_encoder_estimates(self, timeout: float=1.0) -> Tuple[float, float]:
        return self._position, self._velocity

    def get_temperature_msg(self) -> None:
        pass
    
    async def get_temperature(self, timeout: float=1.0):
        return 0

    def get_bus_voltage_current_msg(self):
        pass
    
    async def get_bus_voltage_current(self, timeout=1.0):
        return 0, 0

    def get_torques_msg(self):
        pass

    async def get_torques(self, timeout=1.0):
        return 0, 0

    def get_powers_msg(self):
        pass

    async def get_powers(self, timeout=1.0):
        return 0, 0

class Instruction:
    """Base class for flipper instructions
    Definitions for documentation:
    A pair reffers to either the two front flippers or the two rear ones
    A sibling reffers to the other flipper in the pair (ex:rear left and rear right are siblings)
    """
    def __init__(self):
        self.flippers:List[Flipper] = []
    
    def _addFlipper(self, flipper: Flipper):
        self.flippers.append(flipper)
    
    def update(self): ...
    
    def command(self) -> float:
        """Calculates the setpoint to send to the flipper

        Returns:
            float: This instruction's contribution to the setpoint.
        """
        ...

    # def feedback(self, value: float):
    #     """Sets the actual relative position this instruction has contributed

    #     Args:
    #         value (float): Position contributed (calculated by removing the command() of the other instructions)
    #     """
    #     ...

class StateBool:
    """A boolean container that tracks latching and unlatching (off->on and on->off respectively) state changes
    """
    def __init__(self, value:bool = False):
        self._state = value
        self._lastState = value
    
    @property
    def state(self) -> bool:
        return self._state
    
    @state.setter
    def state(self, value:bool):
        self._lastState = self._state
        self._state = value
        
    @property
    def latched(self) -> bool:
        return not self._lastState and self._state
    
    @property
    def unlatched(self) -> bool:
        return self._lastState and not self._state
        
    def update(self):
        self._lastState = self._state

class SingleInstruction(Instruction):
    """Controls a single flipper
    
    There are three ways to control a single flipper:
    1- When only this flipper is selected out of its pair, up and down DPad controls it's position
    2- When the pair is selected, right DPad moves the flipper to it's starting position (this will make it go the same position as it's sibling)
    3- Anytime this flipper is selected, left DPad sets the zero
    """
    def __init__(self, controller: XboxController, pos: Pos):
        super().__init__()
        self.controller = controller
        self.pos = pos
        self.setOffset = 0.0
        self.offset = 0.0
        
        self.singleControl = StateBool()
        self.convergeControl = StateBool()
        
    def update(self):
        self.offset = self.flippers[0].position - (self.flippers[0].setPosition - self.setOffset)
    
    def command(self) -> float:
        # Is selected
        match self.pos:
            case 'front_left':
                selected = self.controller.LeftBumper.state
            case 'front_right':
                selected = self.controller.RightBumper.state
            case 'rear_left':
                selected = self.controller.LeftTriggerBtn.state
            case 'rear_right':
                selected = self.controller.RightTriggerBtn.state
            case _:
                selected = False
        
        # Is the corresponding pair selected
        if self.pos.startswith('front'):
            pairSelected = self.controller.LeftBumper.state and self.controller.RightBumper.state
        else:
            pairSelected = self.controller.LeftTriggerBtn.state and self.controller.RightTriggerBtn.state
            
        # Is it the only selected in the pair 
        onlySelected = selected and not pairSelected
        
        # Single control
        # Keep this as an assignment for detecting latches and unlatches
        self.singleControl.state = (onlySelected and self.controller.A.state and
            (self.controller.UpDPad.state or self.controller.DownDPad.state))
        if self.singleControl.state:
            if self.controller.DownDPad.state:
                self.setOffset = self.offset - FLIPPER_MOVE_OFFSET
            elif self.controller.UpDPad.state:
                self.setOffset = self.offset + FLIPPER_MOVE_OFFSET
        if self.singleControl.unlatched:
            self.setOffset = self.offset
                
        # Converge
        self.convergeControl.state = (pairSelected and self.controller.A.state and
            self.controller.RightDPad.state)
        if self.convergeControl.state:
            self.setOffset = 0
        if self.convergeControl.unlatched:
            self.setOffset = self.offset
            
        # Zero
        if selected and self.controller.LeftDPad and not self.controller.A.state:
            self.flippers[0].zero()
            
        return self.setOffset
    
class PairInstruction(Instruction):
    def __init__(self, controller: XboxController, pair: Pair):
        super().__init__()
        self.controller = controller
        self.pair: Pair = pair
        self.setOffset = 0.0
        self.offset = 0.0
        
        self.active = StateBool()
        self.convergeControl = StateBool()
        
    def update(self):
        self.offset = mean([f.position - (f.setPosition - self.setOffset) for f in self.flippers])
        
    def command(self) -> float:
        # Is the corresponding pair selected
        if self.pair == 'front':
            pairSelected = self.controller.LeftBumper.state and self.controller.RightBumper.state
        else:
            pairSelected = self.controller.LeftTriggerBtn.state and self.controller.RightTriggerBtn.state
        # Are all flippers selected (no buttons held)
        allSelected = not (self.controller.LeftBumper.state or self.controller.RightBumper.state or
            self.controller.LeftTriggerBtn.state or self.controller.RightTriggerBtn.state)
        
        # Pair move
        self.active.state = (pairSelected and self.controller.A.state and 
            (self.controller.UpDPad.state or self.controller.DownDPad.state))
        if self.active.state:
            if self.controller.DownDPad.state:
                self.setOffset = self.offset - FLIPPER_MOVE_OFFSET
            if self.controller.UpDPad.state:
                self.setOffset = self.offset + FLIPPER_MOVE_OFFSET
        if self.active.unlatched:
            self.setOffset = self.offset
            
        # Converge
        self.convergeControl.state = (allSelected and self.controller.A.state and
            self.controller.RightDPad.state)
        if self.convergeControl.state:
            self.setOffset = 0
        if self.convergeControl.unlatched:
            self.setOffset = self.offset
            
        return self.setOffset
            
class AllInstruction(Instruction):
    def __init__(self, controller: XboxController):
        super().__init__()
        self.controller = controller
        self.setOffset = 0.0
        self.offset = 0.0
        
        self.active = StateBool()
    
    def update(self):
        self.offset = mean([f.position - (f.setPosition - self.setOffset) for f in self.flippers])
    
    def command(self) -> float:
        self.active.state = (self.controller.A.state and not 
            (self.controller.LeftBumper.state or self.controller.RightBumper.state or
             self.controller.LeftTriggerBtn.state or self.controller.RightTriggerBtn.state) and
            (self.controller.UpDPad.state or self.controller.DownDPad.state))

        if self.active.state:
            if self.controller.DownDPad.state:
                self.setOffset = self.offset - FLIPPER_MOVE_OFFSET
            if self.controller.UpDPad.state:
                self.setOffset = self.offset + FLIPPER_MOVE_OFFSET
        if self.active.unlatched:
            self.setOffset = self.offset
        return self.setOffset
    
class Flipper:
    def __init__(self, node: CanSimpleNode):
        self.node = node
        self._zero: float = 0.0
        self._position: float = 0.0
        self._velocity: float = 0.0
        self._setPosition: float = 0.0
        self.instructions: List[Instruction] = []
    
    def addInstruction(self, instruction:Instruction):
        self.instructions.append(instruction)
        instruction._addFlipper(self)
    
    @property
    def position(self) -> float:
        return self._position - self._zero

    def _sendPosition(self):
        self.node.set_position(self._setPosition)

    @property
    def setPosition(self) -> float:
        return self._setPosition - self._zero
    
    @setPosition.setter
    def setPosition(self, value: float):
        self._setPosition = value + self._zero
        self._sendPosition()
        
    def update(self):
        for i in self.instructions:
            i.update()
            
    def run(self):
        self.setPosition = sum([i.command() for i in self.instructions])
        
    def zero(self):
        self._zero = self._position
        
def ensure_flipper_config() -> bool:
    if FLIPPER_OFFSETS_PATH.exists():
        return True
    FLIPPER_OFFSETS_PATH.touch()
    return False

def load_flippers(flippers: Dict[str, Flipper]):
    if not ensure_flipper_config():
        save_flippers(flippers)
        return
    with FLIPPER_OFFSETS_PATH.open('r') as rd:
        data: Optional[Dict[str, float]] = load(rd)
        
        if data is None:
            print("No data")
            return
        for name, zero in data.items():
            flippers[name]._zero = -zero
        
def save_flippers(flippers: Dict[str, Flipper]):
    data: Dict[str, float] = dict()
    for name, flipper in flippers.items():
        data[name] = flipper.position
        
    with FLIPPER_OFFSETS_PATH.open('w') as wr:
        dump(data, wr)
        wr.flush() # Just as a safety measure

class OnExit():
    def __init__(self, func:Callable[..., None], *args, **kwargs):
        self.func = func
        self.args = args
        self.kwargs = kwargs
        
    def __enter__(self) -> OnExit:
        return self
    
    def __exit__(self, _, __, ___):
        self.func(*self.args, **self.kwargs)