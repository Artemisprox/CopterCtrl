"""Motor communication driver over RS-485/Modbus-RTU style serial protocol.

Protocol frame format:
    [ADDR][FUNC][DATA...][CRC_LO][CRC_HI]

This module wraps common commands from the provided protocol document:
- 0x03 Read holding register(s)
- 0x04 Read input/status register(s)
- 0x06 Write single register
- 0x10 Write multiple registers
- 0x42 Enable/disable servo
- 0x41 Save parameters
- 0x43 Clear alarm
"""

from __future__ import annotations

from dataclasses import dataclass
import struct
import threading
import time
from typing import Iterable, List, Optional

import serial


class ModbusError(RuntimeError):
    """Raised when device returns an exception frame."""


@dataclass(frozen=True)
class RegisterMap:
    """Default register map for speed/position loops.

    NOTE: register addresses depend on your drive firmware. Adjust here.
    """

    actual_speed: int = 0x0000
    actual_position: int = 0x0001
    target_speed: int = 0x0100
    target_position: int = 0x0101


class MotorCommDriver:
    def __init__(
        self,
        port: str,
        slave_id: int,
        baudrate: int = 19200,
        bytesize: int = serial.EIGHTBITS,
        parity: str = serial.PARITY_EVEN,
        stopbits: int = serial.STOPBITS_ONE,
        timeout: float = 0.02,
        reg_map: RegisterMap | None = None,
    ) -> None:
        if not (1 <= slave_id <= 247):
            raise ValueError("slave_id must be in [1,247]")

        self.slave_id = slave_id
        self.reg = reg_map or RegisterMap()
        self._ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=bytesize,
            parity=parity,
            stopbits=stopbits,
            timeout=timeout,
        )
        self._lock = threading.Lock()

    def close(self) -> None:
        if self._ser.is_open:
            self._ser.close()

    @staticmethod
    def crc16(data: bytes) -> int:
        crc = 0xFFFF
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc & 0xFFFF

    def _build_frame(self, func: int, payload: bytes) -> bytes:
        frame_wo_crc = bytes([self.slave_id, func]) + payload
        crc = self.crc16(frame_wo_crc)
        return frame_wo_crc + struct.pack("<H", crc)

    def _transceive(self, request: bytes, expected_min_len: int) -> bytes:
        with self._lock:
            self._ser.reset_input_buffer()
            self._ser.write(request)
            self._ser.flush()

            # Read progressively for low-latency RTU responses
            deadline = time.monotonic() + self._ser.timeout
            buf = bytearray()
            while time.monotonic() < deadline:
                chunk = self._ser.read(256)
                if chunk:
                    buf.extend(chunk)
                    # if we have enough bytes and line becomes idle, parse now
                    if len(buf) >= expected_min_len:
                        time.sleep(0.001)
                        if self._ser.in_waiting == 0:
                            break
                else:
                    break

        response = bytes(buf)
        if len(response) < expected_min_len:
            raise TimeoutError(f"short response: {response.hex(' ')}")

        if response[0] != self.slave_id:
            raise RuntimeError(f"unexpected slave id: {response[0]:#x}")

        crc_recv = struct.unpack("<H", response[-2:])[0]
        crc_calc = self.crc16(response[:-2])
        if crc_recv != crc_calc:
            raise RuntimeError(
                f"CRC mismatch recv={crc_recv:#06x} calc={crc_calc:#06x}, raw={response.hex(' ')}"
            )

        func = response[1]
        if func & 0x80:
            exc = response[2]
            raise ModbusError(f"device exception func={func:#x}, code={exc:#x}")

        return response

    def read_holding(self, start_addr: int, count: int) -> List[int]:
        payload = struct.pack(">HH", start_addr & 0xFFFF, count & 0xFFFF)
        req = self._build_frame(0x03, payload)
        rsp = self._transceive(req, expected_min_len=5 + 2)

        byte_count = rsp[2]
        data = rsp[3 : 3 + byte_count]
        if len(data) != byte_count:
            raise RuntimeError("invalid byte count")
        return [struct.unpack(">H", data[i : i + 2])[0] for i in range(0, byte_count, 2)]

    def read_status(self, start_addr: int, count: int) -> List[int]:
        payload = struct.pack(">HH", start_addr & 0xFFFF, count & 0xFFFF)
        req = self._build_frame(0x04, payload)
        rsp = self._transceive(req, expected_min_len=5 + 2)

        byte_count = rsp[2]
        data = rsp[3 : 3 + byte_count]
        if len(data) != byte_count:
            raise RuntimeError("invalid byte count")
        return [struct.unpack(">H", data[i : i + 2])[0] for i in range(0, byte_count, 2)]

    def write_single(self, addr: int, value: int) -> None:
        payload = struct.pack(">HH", addr & 0xFFFF, value & 0xFFFF)
        req = self._build_frame(0x06, payload)
        rsp = self._transceive(req, expected_min_len=8)
        if rsp[2:6] != payload:
            raise RuntimeError("write_single echo mismatch")

    def write_multiple(self, start_addr: int, values: Iterable[int]) -> None:
        words = [v & 0xFFFF for v in values]
        count = len(words)
        if count == 0:
            return
        data = b"".join(struct.pack(">H", w) for w in words)
        payload = (
            struct.pack(">HHB", start_addr & 0xFFFF, count & 0xFFFF, len(data)) + data
        )
        req = self._build_frame(0x10, payload)
        rsp = self._transceive(req, expected_min_len=8)
        echoed_addr, echoed_count = struct.unpack(">HH", rsp[2:6])
        if echoed_addr != (start_addr & 0xFFFF) or echoed_count != count:
            raise RuntimeError("write_multiple ack mismatch")

    def set_enable(self, enabled: bool) -> None:
        payload = bytes([0x55 if enabled else 0xAA])
        req = self._build_frame(0x42, payload)
        self._transceive(req, expected_min_len=5)

    def save_params(self) -> None:
        req = self._build_frame(0x41, b"")
        self._transceive(req, expected_min_len=4)

    def clear_alarm(self) -> None:
        req = self._build_frame(0x43, b"")
        self._transceive(req, expected_min_len=4)

    # -------- high-level helpers --------
    def get_actual_speed(self) -> int:
        return self.read_status(self.reg.actual_speed, 1)[0]

    def get_actual_position(self) -> int:
        return self.read_status(self.reg.actual_position, 1)[0]

    def set_target_speed(self, speed_cmd: int) -> None:
        self.write_single(self.reg.target_speed, speed_cmd)

    def set_target_position(self, position_cmd: int) -> None:
        self.write_single(self.reg.target_position, position_cmd)
