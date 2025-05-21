from asyncio import wait
from datetime import datetime
from pathlib import Path
from statistics import mean
from typing import Dict, Optional
from typing_extensions import override
from can_simple_utils import CanSimpleNode
from abc import abstractmethod, ABC
from json import dump, load

FLIPPER_OFFSETS_PATH = Path("flipper_pos.json")

class Device(ABC):
    @property
    @abstractmethod
    def position(self) -> float: ...
    
    @property
    @abstractmethod
    def setposition(self) -> float: ...
    
    @setposition.setter
    @abstractmethod
    def setposition(self, value: float): ...
    
    @abstractmethod
    async def update(self): ...
    
    
    def move(self, positionDelta: float):
        """Moves the setposition to 'self.position + positionDelta'

        Args:
            positionDelta (float): Revs to move relative to current position
        """
        
    @abstractmethod
    def zero(self):
        """Sets offset to current position
        """
        ...
    
    @abstractmethod
    def go_home(self):
        """Go to offset
        """
        ...
        
class Flipper(Device):
    def __init__(self, node: CanSimpleNode):
        self.node = node
        self._offset: float = 0.0
        self._position: float = 0.0
        self._velocity: float = 0.0
        self._setposition: float = 0.0
    
    @property
    @override
    def position(self) -> float:
        return self._position - self._offset

    @property
    @override
    def setposition(self) -> float:
        return self._setposition - self._offset
    
    @setposition.setter
    @override
    def setposition(self, value: float):
        self._setposition = value + self._offset
        self.node.set_position(self._setposition)
        
    @property
    def offset(self) -> float:
        return self._offset
    
    @offset.setter
    def offset(self, value: float):
        self._offset = value
        
    @override
    async def update(self):
        self._position, self._velocity = await self.node.get_encoder_estimates()
        
    @override
    def zero(self):
        self._offset = self._position
        
    @override
    def move(self, positionDelta: float):
        self.setposition = self.position + positionDelta
        
    @override
    def go_home(self):
        self.setposition = self.offset
        
class MockFlipper(Flipper):
    def __init__(self, maxVelocity: float, maxAcceleration: float):
        self._maxVelocity = maxVelocity
        self._maxAcceleration = maxAcceleration
        self._offset: float = 0.0
        self._position: float = 0.0
        self._velocity: float = 0.0
        self._setposition: float = 0.0
        self._lastUpdate: Optional[datetime] = None
        
    @property
    @override
    def position(self) -> float:
        return self._position - self._offset

    @property
    @override
    def setposition(self) -> float:
        return self._setposition - self._offset
    
    @setposition.setter
    @override
    def setposition(self, value: float):
        self._setposition = value + self._offset
        
    @property
    def offset(self) -> float:
        return self._offset
    
    @offset.setter
    def offset(self, value: float):
        self._offset = value
        
    @override
    async def update(self):
        if self._lastUpdate is None:
            self._lastUpdate = datetime.now()
            return
        now = datetime.now()
        deltaTime = now - self._lastUpdate
        self._lastUpdate = now
        
        if self._position < self._setposition:
            self._position = min(self._setposition, self._position + self._maxVelocity * deltaTime.total_seconds())
        elif self._position > self._setposition:
            self._position = max(self._setposition, self._position - self._maxVelocity * deltaTime.total_seconds())
    @override
    def zero(self):
        self._offset = self._position
        
    @override
    def move(self, positionDelta: float):
        self.setposition = self.position + positionDelta
        
    @override
    def go_home(self):
        self.setposition = self.offset
        
class Pair(Device):
    def __init__(self, dev1: Device, dev2: Device):
        self.dev1 = dev1
        self.dev2 = dev2
        self._diff: float = 0.0
        self._offset: float = 0.0
        self._isMoving = False
        self._isHoming = False

    @property
    @override
    def position(self) -> float:
        return mean([self.dev1.position, self.dev2.position]) - self._offset

    @property
    @override
    def setposition(self) -> float:
        return mean([self.dev1.setposition, self.dev2.setposition]) - self._offset
    
    @setposition.setter
    @override
    def setposition(self, value: float):
        self.dev1.setposition = value + self._offset + self._diff
        self.dev2.setposition = value + self._offset - self._diff
        
    @override
    async def update(self):
        await wait([self.dev1.update(), self.dev2.update()])
        
    @override
    def zero(self):
        self._offset = self.position
        
    @override
    def move(self, positionDelta: float):
        if positionDelta == 0:
            self._isMoving = False
            self.dev1.move(0)
            self.dev2.move(0)
            return
        elif not self._isMoving:
            self._isMoving = True
            self._diff = self.dev1.position - self.position
        self.dev1.setposition = self.position + self._diff + positionDelta
        self.dev2.setposition = self.position - self._diff + positionDelta
        
    @override
    def go_home(self):
        self._diff = 0
        self.setposition = 0
        self._isMoving = False
    
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