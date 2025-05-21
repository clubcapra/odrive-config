from __future__ import annotations
# 100% from https://github.com/odriverobotics/ODriveResources/blob/master/examples/can_simple_utils.py
import asyncio
import can
import struct

from typing import Tuple
from odrive_error_codes import get_error_description

ADDRESS_CMD = 0x06
SET_AXIS_STATE_CMD = 0x07
REBOOT_CMD = 0x16
CLEAR_ERRORS_CMD = 0x18
SET_INPUT_POS_CMD = 0x0C  # Set_Input_Pos command ID

# Newly added CANSimple Get_... command IDs
GET_ENCODER_ESTIMATES_CMD = 0x09
GET_TEMPERATURE_CMD = 0x15
GET_BUS_VOLTAGE_CURRENT_CMD = 0x17
GET_POWERS_CMD = 0x1D
GET_TORQUES_CMD = 0x1C

REBOOT_ACTION_REBOOT = 0
REBOOT_ACTION_SAVE = 1
REBOOT_ACTION_ERASE = 2

class CanSimpleNode():
    def __init__(self, bus: can.BusABC, node_id: int):
        self.bus: can.BusABC = bus
        self.node_id: int = node_id
        self.reader: can.AsyncBufferedReader = can.AsyncBufferedReader()
        self.connected: bool = False

    def __enter__(self) -> CanSimpleNode:
        self.notifier = can.Notifier(
            self.bus, [self.reader], loop=asyncio.get_running_loop()
        )
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.notifier.stop()

    def flush_rx(self) -> None:
        while not self.reader.buffer.empty():
            self.reader.buffer.get_nowait()

    def await_msg(self, cmd_id: int, timeout=1.0):
        async def _impl():
            async for msg in self.reader:
                if msg.arbitration_id == ((self.node_id << 5) | cmd_id):
                    return msg
        return asyncio.wait_for(_impl(), timeout)

    def clear_errors_msg(self, identify: bool = False) -> None:
        data = b'\x01' if identify else b'\x00'
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5) | CLEAR_ERRORS_CMD,
            data=data,
            is_extended_id=False
        ))

    def reboot_msg(self, action: int):
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5) | REBOOT_CMD,
            data=[action],
            is_extended_id=False
        ))

    def getErrorDescription(self, error_code: int):
        desc = get_error_description(error_code)
        print(f"CAN {self.node_id} Error Code: {error_code} - {desc}")
        self.clear_errors_msg()

    def set_state_msg(self, state: int):
        payload = struct.pack('<I', state)
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5) | SET_AXIS_STATE_CMD,
            data=payload,
            is_extended_id=False
        ))
        self.connected = False

    def wait_state(self, stateWaited: int, msg: can.Message) -> bool:
        if self.connected:
            return True
        expected_id = (self.node_id << 5) | 0x01  # Heartbeat cmd_id=1
        if msg.arbitration_id == expected_id:
            error, state, result, traj_done = struct.unpack('<IBBB', msg.data[:7])
            if state == stateWaited:
                if error != 0:
                    self.getErrorDescription(error)
                self.connected = True
                return True
        return False

    def set_velocity(self, vel: float) -> None:
        payload = struct.pack('<ff', vel, 0.0)
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5) | 0x0D,  # Set_Input_Vel
            data=payload,
            is_extended_id=False
        ))

    def set_position(self, pos: float, vel_feedforward: float = 0.0, torque_feedforward: float = 0.0) -> None:
        """
        Set target position (revolutions) with optional velocity and torque feed-forward.

        Frame layout:
          Bytes 0-3: Input_Pos (float32, rev)
          Bytes 4-5: Vel_FF (int16, 0.001 rev/s)
          Bytes 6-7: Torque_FF (int16, 0.001 Nm)
        """
        vel_int = int(vel_feedforward * 1000)
        torque_int = int(torque_feedforward * 1000)
        data = struct.pack('<fhh', pos, vel_int, torque_int)
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5) | SET_INPUT_POS_CMD,
            data=data,
            is_extended_id=False
        ))

    def call_estop(self) -> None:
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5) | 0x02,  # Estop cmd_id=2
            data=b'',
            is_extended_id=False
        ))

    # ----- Newly added getters for feedback -----

    def get_encoder_estimates_msg(self) -> None:
        """Request encoder position and velocity."""
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5) | GET_ENCODER_ESTIMATES_CMD,
            is_extended_id=False,
            is_remote_frame=True
        ))

    async def get_encoder_estimates(self, timeout: float=1.0) -> Tuple[float, float]:
        self.get_encoder_estimates_msg()
        msg = await self.await_msg(GET_ENCODER_ESTIMATES_CMD, timeout)
        return struct.unpack('<ff', msg.data)

    def get_temperature_msg(self) -> None:
        """Request FET and motor temperatures."""
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5) | GET_TEMPERATURE_CMD,
            is_extended_id=False,
            is_remote_frame=True
        ))

    async def get_temperature(self, timeout: float=1.0):
        self.get_temperature_msg()
        msg = await self.await_msg(GET_TEMPERATURE_CMD, timeout)
        return struct.unpack('<ff', msg.data)

    def get_bus_voltage_current_msg(self):
        """Request bus voltage and current."""
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5) | GET_BUS_VOLTAGE_CURRENT_CMD,
            is_extended_id=False,
            is_remote_frame=True
        ))

    async def get_bus_voltage_current(self, timeout=1.0):
        self.get_bus_voltage_current_msg()
        msg = await self.await_msg(GET_BUS_VOLTAGE_CURRENT_CMD, timeout)
        return struct.unpack('<ff', msg.data)

    def get_torques_msg(self):
        """Request torque setpoint and estimate."""
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5) | GET_TORQUES_CMD,
            is_extended_id=False,
            is_remote_frame=True
        ))

    async def get_torques(self, timeout=1.0):
        self.get_torques_msg()
        msg = await self.await_msg(GET_TORQUES_CMD, timeout)
        return struct.unpack('<ff', msg.data)

    def get_powers_msg(self):
        """Request electrical and mechanical power."""
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5) | GET_POWERS_CMD,
            is_extended_id=False,
            is_remote_frame=True
        ))

    async def get_powers(self, timeout=1.0):
        self.get_powers_msg()
        msg = await self.await_msg(GET_POWERS_CMD, timeout)
        return struct.unpack('<ff', msg.data)
