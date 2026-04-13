"""Serial communication with Marlin firmware via G-code.

Thread-safe. Supports dry-run mode for testing without hardware.
"""

from __future__ import annotations

import logging
import re
import threading
import time
from typing import Callable

logger = logging.getLogger(__name__)

POSITION_RE = re.compile(
    r"X:(?P<X>-?\d+\.?\d*)\s+"
    r"Y:(?P<Y>-?\d+\.?\d*)\s+"
    r"Z:(?P<Z>-?\d+\.?\d*)\s+"
    r"E:(?P<E>-?\d+\.?\d*)"
)

# Regex to extract axis values from G1 commands (for dry-run position tracking)
_G1_AXIS_RE = re.compile(r"([XYZE])(-?\d+\.?\d*)")

_INIT_COMMANDS = [
    "G21",       # mm mode
    "G90",       # absolute positioning
    "M82",       # absolute extrusion (E0 = rotation)
]


class SerialConnection:
    """Thread-safe serial connection to MKS Gen L V2.1 running Marlin.

    In dry-run mode, commands are logged and position is simulated.
    """

    def __init__(self, port: str, baud: int, dry_run: bool = False,
                 gcode_file: str | None = None) -> None:
        self._port = port
        self._baud = baud
        self._dry_run = dry_run
        self._serial = None
        self._lock = threading.Lock()
        self._connected = False
        self._sim_pos = {"X": 0.0, "Y": 0.0, "Z": 0.0, "E": 0.0}
        self.on_command: Callable[[str, str], None] | None = None
        self._gcode_file = open(gcode_file, "w") if gcode_file else None

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def dry_run(self) -> bool:
        return self._dry_run

    def connect(self, disable_endstops: bool = True) -> dict[str, float]:
        """Open serial port and send init sequence. Returns initial position."""
        if self._dry_run:
            logger.info(f"[DRY RUN] Connect to {self._port} @ {self._baud}")
            self._connected = True
            for cmd in _INIT_COMMANDS:
                self.send(cmd)
            if disable_endstops:
                self.send("M211 S0")
            return self.get_position()

        import serial
        with self._lock:
            # macOS rejects non-standard baud rates (e.g. 250000) via termios.
            # Always open at a standard baud first, then switch via the
            # IOSSIOSPEED ioctl which pyserial triggers when you assign .baudrate.
            # This works reliably on both macOS and Linux.
            is_standard = self._baud in (9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600)
            if is_standard:
                self._serial = serial.Serial(self._port, self._baud, timeout=5)
            else:
                logger.info("Non-standard baud %d — opening at 9600 then switching via IOSSIOSPEED",
                            self._baud)
                self._serial = serial.Serial(self._port, 9600, timeout=5)
                self._serial.baudrate = self._baud
            time.sleep(2)  # wait for controller boot/reset
            # drain startup messages
            while self._serial.in_waiting:
                self._serial.readline()

        self._connected = True
        for cmd in _INIT_COMMANDS:
            self.send(cmd)
        if disable_endstops:
            self.send("M211 S0")
        return self.get_position()

    def send(self, cmd: str, timeout: float = 20.0) -> str:
        """Send a G-code command and wait for ok/error response.

        Thread-safe. Returns the full response string.
        """
        cmd = cmd.strip()
        if not cmd:
            return ""

        if self._gcode_file:
            self._gcode_file.write(cmd + "\n")

        if self._dry_run:
            return self._send_dry_run(cmd)

        with self._lock:
            if not self._connected or self._serial is None:
                raise ConnectionError("Not connected")

            self._serial.write(f"{cmd}\n".encode())
            logger.debug(f">> {cmd}")

            lines = []
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                raw = self._serial.readline()
                if not raw:
                    continue
                line = raw.decode(errors="replace").strip()
                if not line:
                    continue
                logger.debug(f"<< {line}")
                lines.append(line)
                if line.startswith("ok") or line.startswith("error"):
                    break
            else:
                raise TimeoutError(f"No response to '{cmd}' within {timeout}s")

        response = "\n".join(lines)
        if self.on_command:
            self.on_command(cmd, response)
        return response

    def send_and_wait(self, cmd: str, timeout: float = 60.0) -> str:
        """Send command then M400 (wait for motion to complete)."""
        resp = self.send(cmd, timeout=timeout)
        self.send("M400", timeout=timeout)
        return resp

    def get_position(self) -> dict[str, float]:
        """Query current position via M114. Returns {X, Y, Z, E}."""
        if self._dry_run:
            return dict(self._sim_pos)

        response = self.send("M114")
        for line in response.split("\n"):
            m = POSITION_RE.search(line)
            if m:
                return {k: float(v) for k, v in m.groupdict().items()}
        raise ValueError(f"Could not parse position from: {response}")

    def close(self) -> None:
        """Close the serial port and G-code file."""
        with self._lock:
            self._connected = False
            if self._serial is not None:
                self._serial.close()
                self._serial = None
            if self._gcode_file is not None:
                self._gcode_file.close()
                logger.info("G-code saved to %s", self._gcode_file.name)
                self._gcode_file = None
        logger.info("Serial connection closed")

    # ── Dry-run internals ────────────────────────────────────

    def _send_dry_run(self, cmd: str) -> str:
        logger.debug(f"[DRY] >> {cmd}")
        self._update_sim_position(cmd)

        if cmd.strip() == "M114":
            pos = self._sim_pos
            response = f"X:{pos['X']:.2f} Y:{pos['Y']:.2f} Z:{pos['Z']:.2f} E:{pos['E']:.2f} Count X:0 Y:0 Z:0\nok"
        else:
            response = "ok"

        logger.debug(f"[DRY] << {response.split(chr(10))[0]}")
        if self.on_command:
            self.on_command(cmd, response)
        return response

    def _update_sim_position(self, cmd: str) -> None:
        """Update simulated position from G1/G0/G28 commands."""
        upper = cmd.upper().strip()
        if upper.startswith("G28"):
            self._sim_pos = {"X": 0.0, "Y": 0.0, "Z": 0.0, "E": 0.0}
        elif upper.startswith(("G1 ", "G0 ")):
            for match in _G1_AXIS_RE.finditer(upper):
                axis = match.group(1)
                value = float(match.group(2))
                if axis in self._sim_pos:
                    self._sim_pos[axis] = value
