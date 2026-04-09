"""Solder paste dispensing for SOIC8, SOT-23-6, and LQFN16 packages.

Draws continuous paste lines along pad rows using the T1 solder extruder
with M83 (relative extrusion). Each package type has a specific line pattern:
  SOIC8:  2 lines (left row, right row)
  SOT236: 2 lines (left row, right row)
  LQFN16: 4 lines (bottom, right, top, left) + center EP dot

Motion uses safe_move_to for travel and raw move_to for the controlled
lower → extrude → raise dispensing sequence.
"""

from __future__ import annotations

import logging
import math

from nanopnp.config import NanoPnPConfig, Placement
from nanopnp.motion import MotionController

log = logging.getLogger(__name__)


class PasteDispenser:
    """Dispenses solder paste along pad rows using T1 extruder."""

    def __init__(self, motion: MotionController, config: NanoPnPConfig) -> None:
        self._motion = motion
        self._config = config
        pc = config.paste_dispensing
        self._dispense_z = pc.dispense_z
        self._safe_z = pc.safe_z
        self._travel_f = pc.travel_feedrate
        self._dispense_f = pc.dispense_feedrate
        self._extrude_per_mm = pc.extrude_per_mm

    def dispense_board(self, placements: list[Placement]) -> int:
        """Run full paste dispensing job for all placements.

        Switches to T1, dispenses paste for each placement, returns to origin on T0.
        Returns count of components dispensed.
        """
        self._motion.tool_select(1)  # T1 + M83 (relative extrusion)
        dispensed = 0

        for p in placements:
            part = self._config.parts.get(p.part_id)
            if not part:
                log.warning("No part definition for %s (%s), skipping paste", p.ref, p.part_id)
                continue

            ok = self._dispense_component(p.ref, p.x, p.y, p.rotation, part.name)
            if ok:
                dispensed += 1

        # Return home
        self._motion.safe_move_to(0, 0)
        self._motion.tool_select(0)  # T0 + M82

        log.info("Paste dispensing complete: %d/%d components", dispensed, len(placements))
        return dispensed

    def _dispense_component(self, ref: str, cx: float, cy: float,
                            rotation: float, part_name: str) -> bool:
        """Dispatch to the correct package dispense method. Returns True if dispensed."""
        name_lower = part_name.lower()

        if "soic" in name_lower or "so08" in name_lower:
            log.info("Dispensing paste for %s (SOIC8) at (%.2f, %.2f)", ref, cx, cy)
            self._dispense_soic8(cx, cy, rotation)
        elif "sot" in name_lower:
            log.info("Dispensing paste for %s (SOT236) at (%.2f, %.2f)", ref, cx, cy)
            self._dispense_sot236(cx, cy, rotation)
        elif "qfn" in name_lower or "lqfn" in name_lower:
            log.info("Dispensing paste for %s (LQFN16) at (%.2f, %.2f)", ref, cx, cy)
            self._dispense_lqfn16(cx, cy, rotation)
        else:
            log.warning("No paste pattern for %s (part=%s), skipping", ref, part_name)
            return False
        return True

    # ── Package-specific patterns ─────────────────────────────

    def _dispense_soic8(self, cx: float, cy: float, rot: float) -> None:
        """2 lines: left row (pads 1→4), right row (pads 5→8)."""
        r = self._rot
        # Left row: x=-2.7, y from +1.905 to -1.905
        self._draw_line(*r(-2.7, 1.905, rot, cx, cy), *r(-2.7, -1.905, rot, cx, cy))
        # Right row: x=+2.7, y from -1.905 to +1.905
        self._draw_line(*r(2.7, -1.905, rot, cx, cy), *r(2.7, 1.905, rot, cx, cy))

    def _dispense_sot236(self, cx: float, cy: float, rot: float) -> None:
        """2 lines: left row (pads 1→3), right row (pads 4→6)."""
        r = self._rot
        # Left row: x=-1.15, y from +0.95 to -0.95
        self._draw_line(*r(-1.15, 0.95, rot, cx, cy), *r(-1.15, -0.95, rot, cx, cy))
        # Right row: x=+1.15, y from -0.95 to +0.95
        self._draw_line(*r(1.15, -0.95, rot, cx, cy), *r(1.15, 0.95, rot, cx, cy))

    def _dispense_lqfn16(self, cx: float, cy: float, rot: float) -> None:
        """4 perimeter lines + center EP dot."""
        r = self._rot
        # Bottom: pads 1-4 along y=-1.525
        self._draw_line(*r(-0.75, -1.525, rot, cx, cy), *r(0.75, -1.525, rot, cx, cy))
        # Right: pads 5-8 along x=+1.525
        self._draw_line(*r(1.525, -0.75, rot, cx, cy), *r(1.525, 0.75, rot, cx, cy))
        # Top: pads 9-12 along y=+1.525
        self._draw_line(*r(0.75, 1.525, rot, cx, cy), *r(-0.75, 1.525, rot, cx, cy))
        # Left: pads 13-16 along x=-1.525
        self._draw_line(*r(-1.525, 0.75, rot, cx, cy), *r(-1.525, -0.75, rot, cx, cy))
        # Center exposed pad
        self._dispense_dot(cx, cy)

    # ── Primitives ────────────────────────────────────────────

    def _draw_line(self, sx: float, sy: float, ex: float, ey: float) -> None:
        """Draw one paste line: travel to start → lower → extrude → raise."""
        length = math.hypot(ex - sx, ey - sy)
        e_amount = round(length * self._extrude_per_mm, 4)

        # Travel to start at safe Z (use paste travel feedrate, not default)
        self._motion.safe_move_to(sx, sy, feedrate=self._travel_f)
        # Lower to dispense height
        self._motion.move_to(z=self._dispense_z, feedrate=self._travel_f)
        # Dispense along line
        self._motion.move_to(x=ex, y=ey, e=e_amount, feedrate=self._dispense_f)
        # Raise
        self._motion.move_to(z=self._safe_z, feedrate=self._travel_f)

    def _dispense_dot(self, x: float, y: float) -> None:
        """Dispense a dot of paste at a point."""
        e_amount = round(self._extrude_per_mm * 0.5, 4)
        self._motion.safe_move_to(x, y, feedrate=self._travel_f)
        self._motion.move_to(z=self._dispense_z, feedrate=self._travel_f)
        self._motion.move_to(e=e_amount)
        self._motion.move_to(z=self._safe_z, feedrate=self._travel_f)

    @staticmethod
    def _rot(dx: float, dy: float, angle_deg: float,
             cx: float, cy: float) -> tuple[float, float]:
        """Rotate pad offset by component angle and add to center."""
        rad = math.radians(angle_deg)
        cos_r = math.cos(rad)
        sin_r = math.sin(rad)
        return (
            round(cx + dx * cos_r - dy * sin_r, 4),
            round(cy + dx * sin_r + dy * cos_r, 4),
        )
