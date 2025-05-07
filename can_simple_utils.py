# 100% from https://github.com/odriverobotics/ODriveResources/blob/master/examples/can_simple_utils.py
import asyncio
import can
import struct
import time

from odrive_error_codes import get_error_description

ADDRESS_CMD = 0x06
SET_AXIS_STATE_CMD = 0x07
REBOOT_CMD = 0x16
CLEAR_ERRORS_CMD = 0x18

REBOOT_ACTION_REBOOT = 0
REBOOT_ACTION_SAVE = 1
REBOOT_ACTION_ERASE = 2

class CanSimpleNode():
    def __init__(self, bus: can.Bus, node_id: int): # type: ignore
        self.bus = bus
        self.node_id = node_id
        self.reader = can.AsyncBufferedReader()
        self.stateChanged = False
        self.connected = False
        self.state = 0
        self.voltage = 0
        self.current = 0

    def __enter__(self):
        self.notifier = can.Notifier(self.bus, [self.reader], loop=asyncio.get_running_loop())
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.notifier.stop()
        pass

    def flush_rx(self):
        while not self.reader.buffer.empty():
            self.reader.buffer.get_nowait()

    def await_msg(self, cmd_id: int, timeout=1.0):
        async def _impl():
            async for msg in self.reader:
                if msg.arbitration_id == (self.node_id << 5 | cmd_id):
                    return msg
        return asyncio.wait_for(_impl(), timeout)

    def clear_errors_msg(self, identify: bool = False):
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5) | CLEAR_ERRORS_CMD,
            data=b'\x01' if identify else b'\x00',
            is_extended_id=False
        ))

    def reboot_msg(self, action: int):
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5) | REBOOT_CMD,
            data=[action],
            is_extended_id=False
        ))

    def getErrorDescription(self, error_code):
        error_description = get_error_description(error_code)
        print(f"CAN {self.node_id} Error Code: {error_code} - {error_description}")
        self.clear_errors_msg()

    def set_state_msg(self, state: int):
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | SET_AXIS_STATE_CMD),
            data=struct.pack('<I', state),
            is_extended_id=False
        ))
        self.stateChanged = False
    
    def handle_msg(self, msg):
        if msg.arbitration_id == (self.node_id << 5 | 0x01):  # Heartbeat
            self.connected = True
            error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
            self.state = state
            if error != 0:
                self.getErrorDescription(error)  # Check for error codes

        if msg.arbitration_id == (self.node_id << 5 | 0x17): # Bus Voltage
            self.voltage, self.current = struct.unpack('<ff', msg.data)


    def wait_state(self, stateWaited: int):
        return stateWaited == self.state

    def set_velocity(self, vel:float):
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | 0x0d), # 0x0d: Set_Input_Vel
            data=struct.pack('<ff', vel, 0.0), # 1.0: velocity, 0.0: torque feedforward
            is_extended_id=False
        ))

    def set_position(self, pos: float, vel_feedforward: float = 0.0):
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | 0x0c),  # 0x0c: Set_Input_Pos
            data=struct.pack('<fff', pos, vel_feedforward, 0.0),  # Position, velocity, torque
            is_extended_id=False
        ))

    def get_encoder_estimates(self, timeout=1.0):
        """
        Wait for a 'Get_Encoder_Estimates' message (ID 0x09) from this node.
        Returns (pos_estimate, vel_estimate) as floats.
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            msg = self.bus.recv(timeout=0.1)
            if msg and msg.arbitration_id == (self.node_id << 5 | 0x09):
                pos, vel = struct.unpack('<ff', msg.data)
                return pos
        raise TimeoutError(f"Node {self.node_id}: No encoder estimate received within {timeout} s")