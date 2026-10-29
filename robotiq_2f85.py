"""Robotiq 2F-85 gripper driver (Modbus RTU over USB-RS485).

No external Modbus library: the gripper only needs FC03 (read holding
registers) and FC16 (write multiple registers), so they are implemented
directly on top of pyserial.

Register map (Robotiq 2F/3F "Gripper" family):
  Output (write, 0x03E8..):  byte0 ACTION, byte1-2 reserved,
                             byte3 rPR position, byte4 rSP speed, byte5 rFR force
  Input  (read,  0x07D0..):  byte0 STATUS, byte1 reserved, byte2 FAULT,
                             byte3 rPR echo, byte4 gPO position, byte5 gCU current

Position units are raw counts: 0 = fully open, 255 = fully closed.
"""

import os
import time

import serial


def _default_port() -> str:
    """Prefer the stable /dev/robotiq symlink created by setup_permissions.sh;
    fall back to the first FTDI node if the udev rule is not installed."""
    return "/dev/robotiq" if os.path.exists("/dev/robotiq") else "/dev/ttyUSB0"


DEFAULT_PORT = _default_port()
DEFAULT_BAUD = 115200
DEFAULT_SLAVE = 9

OUTPUT_REG = 0x03E8   # 1000
INPUT_REG = 0x07D0    # 2000

STROKE_MM = 85.0      # 2F-85 full opening

# Position feedback (gPO) does not span the full 0..255 command range: the
# fingers hit their mechanical stops short of it. These are the limits measured
# on one unit (repeatable across runs). They affect only the mm conversion, not
# raw-count commands. Re-measure with calibrate() on a different gripper, or
# after changing fingertips or pads.
POS_OPEN = 3          # gPO at the fully open stop  -> STROKE_MM
POS_CLOSED = 230      # gPO at the fully closed stop -> 0 mm

_FAULTS = {
    0x00: "no fault",
    0x05: "action delayed, activation must be completed first",
    0x07: "activation bit must be set before any action",
    0x08: "maximum operating temperature exceeded, wait for cool-down",
    0x09: "no communication for at least 1 second",
    0x0A: "under minimum operating voltage",
    0x0B: "automatic release in progress",
    0x0C: "internal fault, contact support",
    0x0D: "activation fault, verify no interference during activation",
    0x0E: "overcurrent triggered",
    0x0F: "automatic release completed",
}

_OBJ = {
    0: "moving (no object detected yet)",
    1: "stopped, object detected while opening",
    2: "stopped, object detected while closing",
    3: "at requested position (no object detected)",
}


class GripperError(RuntimeError):
    pass


def _crc16(data: bytes) -> bytes:
    """Modbus RTU CRC-16, returned little-endian as it goes on the wire."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return bytes([crc & 0xFF, (crc >> 8) & 0xFF])


class Status:
    """Decoded gripper input registers."""

    def __init__(self, raw: bytes):
        self.raw = raw
        status = raw[0]
        self.activated = bool(status & 0x01)          # gACT
        self.goto = bool(status & 0x08)               # gGTO
        self.activation_state = (status >> 4) & 0x03  # gSTA
        self.object_status = (status >> 6) & 0x03     # gOBJ
        self.fault_code = raw[2] & 0x0F               # gFLT
        self.position_request = raw[3]                # rPR echo
        self.position = raw[4]                        # gPO
        self.current_raw = raw[5]                     # gCU

    @property
    def ready(self) -> bool:
        return self.activated and self.activation_state == 3

    @property
    def moving(self) -> bool:
        return self.object_status == 0 and self.goto

    @property
    def object_detected(self) -> bool:
        return self.object_status in (1, 2)

    @property
    def opening_mm(self) -> float:
        """Approximate finger opening in mm, linear between the measured stops."""
        span = POS_CLOSED - POS_OPEN
        mm = STROKE_MM * (POS_CLOSED - self.position) / span
        return max(0.0, min(STROKE_MM, mm))

    @property
    def current_mA(self) -> float:
        return self.current_raw * 10.0

    @property
    def fault(self) -> str:
        return _FAULTS.get(self.fault_code, f"unknown fault 0x{self.fault_code:02X}")

    def __str__(self) -> str:
        return (
            f"pos={self.position:3d} ({self.opening_mm:5.1f} mm)  "
            f"req={self.position_request:3d}  current={self.current_mA:.0f} mA  "
            f"activated={self.activated}  sta={self.activation_state}  "
            f"obj={_OBJ.get(self.object_status)}  fault={self.fault}"
        )


class Robotiq2F85:
    def __init__(self, port=DEFAULT_PORT, baudrate=DEFAULT_BAUD,
                 slave_id=DEFAULT_SLAVE, timeout=0.5):
        self.slave_id = slave_id
        self.ser = serial.Serial(
            port=port, baudrate=baudrate, bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
            timeout=timeout, write_timeout=timeout,
        )
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    # ---- context manager -------------------------------------------------
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.disconnect()

    def disconnect(self):
        """Release the serial port. Named distinctly from close(), which is a
        motion command that shuts the gripper's fingers."""
        if self.ser.is_open:
            self.ser.close()

    # ---- Modbus RTU primitives ------------------------------------------
    def _transfer(self, pdu: bytes, expected_len: int, retries: int = 3) -> bytes:
        frame = bytes([self.slave_id]) + pdu
        frame += _crc16(frame)
        last = None
        for _ in range(retries):
            self.ser.reset_input_buffer()
            self.ser.write(frame)
            self.ser.flush()
            resp = self.ser.read(expected_len)
            if len(resp) != expected_len:
                last = f"short response ({len(resp)}/{expected_len} bytes): {resp.hex()}"
                time.sleep(0.01)
                continue
            if _crc16(resp[:-2]) != resp[-2:]:
                last = f"CRC mismatch: {resp.hex()}"
                time.sleep(0.01)
                continue
            if resp[0] != self.slave_id:
                last = f"wrong slave id {resp[0]}"
                time.sleep(0.01)
                continue
            if resp[1] & 0x80:
                raise GripperError(f"Modbus exception code {resp[2]}")
            return resp
        if last and last.startswith("short response (0/"):
            # Nothing at all came back. The USB-RS485 adapter answers to the
            # host regardless, so total silence means the gripper itself is not
            # driving the bus - almost always loss of its 24 V supply (a supply
            # tripping its current limit on grip stall current looks exactly
            # like this), otherwise an unseated or miswired RS485 pair.
            raise GripperError(
                "gripper did not reply at all - check its 24 V supply "
                "(a supply that tripped on grip current latches off until "
                "reset) and the RS485 wiring; the USB adapter itself is fine")
        raise GripperError(f"no valid reply from gripper: {last}")

    def _read_registers(self, address: int, count: int) -> bytes:
        pdu = bytes([0x03, address >> 8, address & 0xFF, count >> 8, count & 0xFF])
        # slave + fc + bytecount + data + crc
        resp = self._transfer(pdu, 5 + 2 * count)
        return resp[3:3 + 2 * count]

    def _write_registers(self, address: int, values: bytes):
        count = len(values) // 2
        pdu = bytes([0x10, address >> 8, address & 0xFF,
                     count >> 8, count & 0xFF, len(values)]) + values
        self._transfer(pdu, 8)

    # ---- gripper API -----------------------------------------------------
    def get_status(self) -> Status:
        return Status(self._read_registers(INPUT_REG, 3))

    def _command(self, action: int, position: int, speed: int, force: int):
        payload = bytes([action, 0x00, 0x00,
                         position & 0xFF, speed & 0xFF, force & 0xFF])
        self._write_registers(OUTPUT_REG, payload)

    def reset(self):
        """Clear rACT; required before a fresh activation or to clear a fault."""
        self._command(0x00, 0, 0, 0)
        time.sleep(0.1)

    def activate(self, timeout: float = 10.0) -> Status:
        """Run the activation (calibration) sequence. Safe to call when already active."""
        status = self.get_status()
        if status.ready:
            return status
        self.reset()
        self._command(0x01, 0, 0, 0)  # rACT = 1
        deadline = time.time() + timeout
        while time.time() < deadline:
            status = self.get_status()
            if status.fault_code not in (0x00, 0x05, 0x07):
                raise GripperError(f"activation fault: {status.fault}")
            if status.ready:
                return status
            time.sleep(0.1)
        raise GripperError("activation timed out")

    def move(self, position: int, speed: int = 255, force: int = 150,
             wait: bool = True, timeout: float = 5.0) -> Status:
        """Move to a raw position (0 = open, 255 = closed)."""
        position = max(0, min(255, int(position)))
        speed = max(0, min(255, int(speed)))
        force = max(0, min(255, int(force)))
        status = self.get_status()
        if not status.ready:
            raise GripperError("gripper is not activated - call activate() first")
        self._command(0x09, position, speed, force)  # rACT | rGTO
        if not wait:
            return self.get_status()
        deadline = time.time() + timeout
        # let the gripper register the new target before polling for completion
        while time.time() < deadline:
            status = self.get_status()
            if status.position_request == position:
                break
            time.sleep(0.01)
        while time.time() < deadline:
            status = self.get_status()
            if status.object_status != 0:
                return status
            time.sleep(0.02)
        raise GripperError("move timed out")

    def move_mm(self, opening_mm: float, **kwargs) -> Status:
        """Move to an approximate opening in millimetres (0..85)."""
        opening_mm = max(0.0, min(STROKE_MM, float(opening_mm)))
        span = POS_CLOSED - POS_OPEN
        position = round(POS_CLOSED - span * opening_mm / STROKE_MM)
        return self.move(position, **kwargs)

    def open(self, **kwargs) -> Status:
        return self.move(0, **kwargs)

    def close(self, **kwargs) -> Status:
        """Close the fingers (position 255). To release the serial port use
        disconnect()."""
        return self.move(255, **kwargs)

    def stop(self):
        """Clear rGTO: the gripper holds its current position."""
        self._command(0x01, 0, 0, 0)

    def auto_release(self, open_direction: bool = True):
        """Emergency slow release (rATR). Requires reset()+activate() afterwards."""
        action = 0x11 | (0x20 if open_direction else 0x00)  # rACT | rATR | rARD
        self._command(action, 0, 0, 0)

    def calibrate(self) -> tuple:
        """Drive to both stops and report the measured (open, closed) gPO values.

        Use the result to update POS_OPEN / POS_CLOSED if the fingertips or
        pads are changed. Moves the fingers through the full stroke.
        """
        open_pos = self.move(0, wait=True).position
        closed_pos = self.move(255, wait=True).position
        return open_pos, closed_pos
