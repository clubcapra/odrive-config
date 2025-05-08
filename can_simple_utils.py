from __future__ import annotations
# 100% from https://github.com/odriverobotics/ODriveResources/blob/master/examples/can_simple_utils.py
import asyncio
from datetime import datetime, timedelta
import can
import struct

from typing import Tuple
from common import *
from odrive_error_codes import get_error_description




class CanSimpleNode():
    def __init__(self, bus: can.BusABC, node_id: int):
        self.bus: can.BusABC = bus
        self.node_id: int = node_id
        self.reader: can.AsyncBufferedReader = can.AsyncBufferedReader()
        self.connected: bool = False
        self._position = 0.0
        self.velocity = 0.0
        self.torque = 0.0
        self.error = 0
        self._lastError = 0
        self._nextPrint = datetime.now()
        self.disarmReason = 0
        self.state = STATE_IDLE
        self.current = 0.0
        self.voltage = 0.0
        self.fetTemperature = 0.0
        self.motorTemperature = 0.0

    @property
    def position(self) -> float:
        return self._position

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
    
    def read_msg(self, msg: can.Message):
        nid = ((msg.arbitration_id & (0b11111 << 5)) >> 5)
        if nid != self.node_id:
            return
        cmd_id = msg.arbitration_id & 0b11111
        if cmd_id == GET_ENCODER_ESTIMATES_CMD:
            self._position, self.velocity = struct.unpack('<ff', msg.data)
            # print(f'{nid} {self.node_id}: pos:{self.position} vel:{self.velocity}')
        elif cmd_id == GET_BUS_VOLTAGE_CURRENT_CMD:
            self.voltage, self.current = struct.unpack('<ff', msg.data)
        elif cmd_id == GET_TEMPERATURE_CMD:
            self.fetTemperature, self.motorTemperature = struct.unpack('<ff', msg.data)
        elif cmd_id == GET_BUS_VOLTAGE_CURRENT_CMD:
            self.voltage, self.current = struct.unpack('<ff', msg.data)
        elif cmd_id == GET_TORQUES_CMD:
            _, self.torque = struct.unpack('<ff', msg.data)
        elif cmd_id == GET_ERROR_CMD:
            self.error, self.disarmReason = struct.unpack('<II', msg.data)
            if self.error != 0:
                self.getErrorDescription(self.error)
        elif cmd_id == HEARTBEAT_CMD:
            errOrDisarmReason, self.state, procedureDone, trajDone = struct.unpack('<IBBBx', msg.data)


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
        now = datetime.now()
        if self._lastError != self.error or self._nextPrint <= now:
            desc = get_error_description(error_code)
            print(f"CAN {self.node_id} Error Code: {error_code} - {desc}")
            self._nextPrint = now + timedelta(seconds=1)
        self._lastError = self.error
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
            arbitration_id=(self.node_id << 5) | SET_INPUT_VEL_CMD,  # Set_Input_Vel
            data=payload,
            is_extended_id=False
        ))

    def set_traj_vel_limit(self, vel: float) -> None:
        payload = struct.pack('<f', vel)
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5) | SET_TRAJ_VEL_LIMIT_CMD,
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
