"""High-level motion control with safe Z sequencing.

The key invariant: safe_move_to() NEVER combines Z with XY in one G1 command.
This solves the diagonal motion bug that made OpenPnP unusable for our setup.
"""

from __future__ import annotations

import logging

from nanopnp.config import NanoPnPConfig
from nanopnp.serial_comm import SerialConnection

logger = logging.getLogger(__name__)


class MotionController:
    """Translates high-level moves into safe G-code sequences.

    Two motion primitives:
    - safe_move_to: retract Z → move XY → lower Z (for travel between locations)
    - move_to: raw G1 with only specified axes (for controlled sequences like paste dispensing)
    """

    def __init__(self, serial: SerialConnection, config: NanoPnPConfig) -> None:
        self._serial = serial
        self._config = config
        self._safe_z = config.z_heights.safe
        self._feedrate = config.machine.travel_feedrate
        self._z_feedrate = config.machine.z_feedrate
        self._vacuum_is_on = False
        self._current_tool: int | None = None
        self._limits = config.machine.axis_limits

    @property
    def serial(self) -> SerialConnection:
        return self._serial

    def _check_limits(self, x: float | None = None, y: float | None = None) -> None:
        """Raise ValueError if coordinates exceed axis limits."""
        if x is not None:
            if x < self._limits.x.min or x > self._limits.x.max:
                raise ValueError(f"X={x:.2f} outside limits [{self._limits.x.min}, {self._limits.x.max}]")
        if y is not None:
            if y < self._limits.y.min or y > self._limits.y.max:
                raise ValueError(f"Y={y:.2f} outside limits [{self._limits.y.min}, {self._limits.y.max}]")

    def home(self) -> None:
        """Home all axes (M400 + G28)."""
        logger.info("Homing all axes")
        self._serial.send("M400")
        self._serial.send("G28")
        self._vacuum_is_on = False

    def move_to(self, *, x: float | None = None, y: float | None = None,
                z: float | None = None, e: float | None = None,
                feedrate: float | None = None) -> None:
        """Raw G1 move. Only specified axes are included.

        Does NOT enforce safe Z. Use safe_move_to() for travel between locations.
        This exists for controlled sequences (paste dispense, pick/place Z moves).
        """
        self._check_limits(x, y)
        parts = ["G1"]
        if x is not None:
            parts.append(f"X{x:.4f}")
        if y is not None:
            parts.append(f"Y{y:.4f}")
        if z is not None:
            parts.append(f"Z{z:.4f}")
        if e is not None:
            parts.append(f"E{e:.4f}")
        # Use z_feedrate when only Z is moving, travel_feedrate otherwise
        z_only = z is not None and x is None and y is None
        f = feedrate or (self._z_feedrate if z_only else self._feedrate)
        parts.append(f"F{f:.0f}")

        if len(parts) <= 2:  # only "G1" + feedrate, no axes
            return

        cmd = " ".join(parts)
        self._serial.send_and_wait(cmd)

    def safe_move_to(self, x: float, y: float, z: float | None = None,
                     e: float | None = None, feedrate: float | None = None) -> None:
        """Move to (x, y) with guaranteed safe Z retraction.

        Sequence:
        1. Retract Z to safe height
        2. Move XY (+ optional E rotation)
        3. Lower Z to target (if specified)

        Z is NEVER combined with XY in the same G1 command.
        """
        self._check_limits(x, y)
        f_xy = feedrate or self._feedrate
        f_z = self._z_feedrate
        logger.info(f"safe_move_to X={x} Y={y} Z={z} E={e}")

        # Step 1: retract Z to safe height (slow Z feedrate)
        self._serial.send_and_wait(f"G1 Z{self._safe_z:.4f} F{f_z:.0f}")

        # Step 2: move XY + E (default E0 to keep nozzle straight)
        e_val = e if e is not None else 0.0
        self._serial.send_and_wait(f"G1 X{x:.4f} Y{y:.4f} E{e_val:.4f} F{f_xy:.0f}")

        # Step 3: lower Z if specified (slow Z feedrate)
        if z is not None:
            self._serial.send_and_wait(f"G1 Z{z:.4f} F{f_z:.0f}")

    def vacuum_on(self) -> None:
        """Turn vacuum on. No-op if already on."""
        if self._vacuum_is_on:
            return
        logger.info("Vacuum ON")
        self._serial.send(self._config.vacuum.on_cmd)
        dwell = self._config.vacuum.pick_dwell_ms
        if dwell > 0:
            self._serial.send(f"G4 P{dwell}")
        self._serial.send("M400")
        self._vacuum_is_on = True

    def vacuum_off(self) -> None:
        """Turn vacuum off. No-op if already off."""
        if not self._vacuum_is_on:
            return
        logger.info("Vacuum OFF")
        self._serial.send(self._config.vacuum.off_cmd)
        dwell = self._config.vacuum.place_dwell_ms
        if dwell > 0:
            self._serial.send(f"G4 P{dwell}")
        self._serial.send("M400")
        self._vacuum_is_on = False

    def tool_select(self, tool: int) -> None:
        """Select tool head. T0 = vacuum nozzle (M82), T1 = paste dispenser (M83). No-op if already selected."""
        if self._current_tool == tool:
            return
        logger.info(f"Tool select T{tool}")
        self._serial.send(f"T{tool}")
        if tool == 0:
            self._serial.send("M82")  # absolute extrusion for rotation
        elif tool == 1:
            self._serial.send("M83")  # relative extrusion for paste
        self._current_tool = tool

    def get_position(self) -> dict[str, float]:
        """Get current position {X, Y, Z, E}."""
        return self._serial.get_position()

    def wait(self) -> None:
        """Wait for all queued moves to complete."""
        self._serial.send("M400")

    def send_raw(self, cmd: str) -> str:
        """Send raw G-code (for GUI console). Returns response."""
        return self._serial.send(cmd)
