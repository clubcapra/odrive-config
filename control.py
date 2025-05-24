from asyncio import wait
from datetime import datetime
from pathlib import Path
from statistics import mean
from typing import Dict, Iterable, Optional
from typing_extensions import override
from can_simple_utils import CanSimpleNode
from abc import abstractmethod, ABC
from json import dump, load

FLIPPER_OFFSETS_PATH = Path("flipper_pos.json")

"""Device:
Data:
position: Actual position
setpoint: Actual current setpoint
enable: Activate the movement

Actions:


"""

class Device(ABC):
    @property
    @abstractmethod
    def position(self) -> float: ...
    
    @property
    @abstractmethod
    def setPosition(self) -> float: ...
    
    @setPosition.setter
    @abstractmethod
    def setPosition(self, value: float): ...
    
    @property
    @abstractmethod
    def offset(self) -> float: ...
    
    @offset.setter
    @abstractmethod
    def offset(self, value: float): ...
    
    @property
    @abstractmethod
    def targetOffset(self) -> float: ...
    
    @targetOffset.setter
    @abstractmethod
    def targetOffset(self, value: float): ...
    
    @abstractmethod
    async def update(self): ...
    
    @abstractmethod
    def zero(self):
        """Sets offset to current position
        """
        ...
    
class Flipper(Device):
    def __init__(self, node: CanSimpleNode):
        self.node = node
        self._zero: float = 0.0
        self._offset: float = 0.0
        self._position: float = 0.0
        self._velocity: float = 0.0
        self._setPosition: float = 0.0
        self._targetOffset: float = 0.0
        self._offsetting: bool = False
    
    @property
    @override
    def position(self) -> float:
        return self._position - self._zero

    def _sendPosition(self):
        self.node.set_position(self._setPosition)

    @property
    @override
    def setPosition(self) -> float:
        return self._setPosition - self._zero - self._offset
    
    @setPosition.setter
    @override
    def setPosition(self, value: float):
        self._offsetting = False
        self._setPosition = value + self._zero + self._offset
        self._sendPosition()
        
    @property
    @override
    def offset(self) -> float:
        return self._offset
    
    @offset.setter
    @override
    def offset(self, value: float):
        self._offset = value
        self._sendPosition()
        
    @property
    @override
    def targetOffset(self) -> float:
        return self._targetOffset
    
    @targetOffset.setter
    @override
    def targetOffset(self, value: float):
        self._offsetting = True
        
        self._targetOffset = value
        self._sendPosition()
        
    @override
    async def update(self):
        prev = self._position
        self._position, self._velocity = await self.node.get_encoder_estimates()
        if self._offsetting:
            self._offset += self._position - prev
        if abs(self._targetOffset - self._offset) < 0.5:
            self._offsetting = False
        
    @override
    def zero(self):
        self._zero = self._position
        
class MockFlipper(Flipper):
    def __init__(self, maxVelocity: float, maxAcceleration: float):
        self._maxVelocity = maxVelocity
        self._maxAcceleration = maxAcceleration
        self._zero: float = 0.0
        self._offset: float = 0.0
        self._targetOffset: float = 0.0
        self._offsetting = False
        self._position: float = 0.0
        self._velocity: float = 0.0
        self._setPosition: float = 0.0
        self._lastUpdate: Optional[datetime] = None
        self.enable = False
        
    @property
    @override
    def position(self) -> float:
        return self._position - self._zero

    @property
    @override
    def setPosition(self) -> float:
        return self._setPosition - self._zero
    
    @setPosition.setter
    @override
    def setPosition(self, value: float):
        self._targetOffset = self._offset
        self._offsetting = False
        self._setPosition = value + self._zero
        
    @property
    def offset(self) -> float:
        return self._offset
    
    @offset.setter
    def offset(self, value: float):
        self._offset = value
        
    @property
    @override
    def targetOffset(self) -> float:
        return self._targetOffset
    
    @targetOffset.setter
    @override
    def targetOffset(self, value: float):
        self._offsetting = abs(self._targetOffset - self._offset) >= 0.5
        self._targetOffset = value
        
    @override
    async def update(self):
        if self._lastUpdate is None or not self.enable:
            self._lastUpdate = datetime.now()
            return
        now = datetime.now()
        deltaTime = now - self._lastUpdate
        self._lastUpdate = now
        
        prev = self._position
        target = self._setPosition + self._zero + self._targetOffset
        
        if self._position < target:
            self._position = min(target, self._position + self._maxVelocity * deltaTime.total_seconds())
        elif self._position > target:
            self._position = max(target, self._position - self._maxVelocity * deltaTime.total_seconds())
            
        if self._offsetting:
            self._offset += self._position - prev
        if abs(self._targetOffset - self._offset) < 0.5:
            self._offsetting = False
            self._targetOffset = self._offset
            
    @override
    def zero(self):
        old = self._zero
        self._zero = self._position
        self._offset = old - self._zero

def move_single(device: Device, relativePosition: float):
    device.targetOffset = device.offset + relativePosition
    
def converge_group(group: Iterable[Device]):
    for d in group:
        d.targetOffset = 0.0

def move_group(group: Iterable[Device], relativePosition: float):
    target = mean([d.position for d in group]) + relativePosition
    
    for d in group:
        d.setPosition = target
    

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
        for name, offset in data.items():
            flippers[name].offset = -offset
        
def save_flippers(flippers: Dict[str, Flipper]):
    data: Dict[str, float] = dict()
    for name, flipper in flippers.items():
        data[name] = flipper.position
        
    with FLIPPER_OFFSETS_PATH.open('w') as wr:
        dump(data, wr)