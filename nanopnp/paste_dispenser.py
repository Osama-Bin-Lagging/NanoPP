"""Solder paste dispensing using Gerber-parsed pad rows.

This is the tried-and-tested pipeline:
1. Parse .gbr file for %TO.C...%TD*% blocks to get component pads
2. Group pads into rows via union-find (nearest-neighbor threshold)
3. Order each row via PCA (works for any rotation)
4. Generate G-code: travel → extrude across row → retract

Dispenses at a fixed travel Z (no per-line Z descent). Relies on
extrusion rate + row length to control paste quantity.

Exposes two entry points:
  dispense_gerber(path)   — parse a .gbr file and dispense all components
  dispense_board(placements) — back-compat: use config placements (currently
                               falls back to gerber path if configured)
"""

from __future__ import annotations

import logging
import math
import re
from pathlib import Path

from nanopnp.board_parser import parse_edge_cuts
from nanopnp.config import NanoPnPConfig, Placement
from nanopnp.motion import MotionController

log = logging.getLogger(__name__)


# ── Gerber parser ──────────────────────────────────────────────

def parse_gbr(filename: str | Path) -> dict[str, list[tuple[float, float]]]:
    """Parse a KiCad Gerber X2 file and return {component_ref: [(x,y), ...]}."""
    path = Path(filename)
    if not path.exists():
        return {}
    content = path.read_text()

    components: dict[str, list[tuple[float, float]]] = {}
    # Extract pads by footprint reference (X2 attributes)
    for m in re.finditer(r'%TO\.C,(\S+?)\*%(.+?)%TD\*%', content, re.DOTALL):
        ref, block = m.group(1), m.group(2)
        pads: list[tuple[float, float]] = []
        for pad in re.finditer(r'X(-?\d+)Y(-?\d+)D03', block):
            x = int(pad.group(1)) / 1e6  # 4.6 format scaling
            y = int(pad.group(2)) / 1e6
            pads.append((x, y))
        if pads:
            components.setdefault(ref, []).extend(pads)
    return components


# ── Geometry helpers ───────────────────────────────────────────

def _dist(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _pca_direction(pads: list[tuple[float, float]]) -> tuple[float, float]:
    """Principal component direction (unit vector) for a set of 2D points."""
    n = len(pads)
    mx = sum(p[0] for p in pads) / n
    my = sum(p[1] for p in pads) / n
    cxx = sum((p[0] - mx) ** 2 for p in pads)
    cyy = sum((p[1] - my) ** 2 for p in pads)
    cxy = sum((p[0] - mx) * (p[1] - my) for p in pads)
    if abs(cxy) > 1e-10:
        disc = math.sqrt(max(0, ((cxx - cyy) / 2) ** 2 + cxy ** 2))
        l1 = (cxx + cyy) / 2 + disc
        vx, vy = l1 - cyy, cxy
        mag = math.sqrt(vx ** 2 + vy ** 2)
        return vx / mag, vy / mag
    return (1.0, 0.0) if cxx >= cyy else (0.0, 1.0)


def group_into_rows(pads: list[tuple[float, float]]) -> list[list[tuple[float, float]]]:
    """Union-find grouping by nearest-neighbor threshold, then PCA-sort each row."""
    if len(pads) <= 1:
        return [list(pads)]

    # Find minimum pairwise distance
    min_d = min(
        _dist(pads[i], pads[j])
        for i in range(len(pads))
        for j in range(i + 1, len(pads))
    )

    # Union-find
    n = len(pads)
    parent = list(range(n))

    def find(i: int) -> int:
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    def union(i: int, j: int) -> None:
        ri, rj = find(i), find(j)
        if ri != rj:
            parent[ri] = rj

    threshold = min_d * 1.8
    for i in range(n):
        for j in range(i + 1, n):
            if _dist(pads[i], pads[j]) <= threshold:
                union(i, j)

    groups: dict[int, list[tuple[float, float]]] = {}
    for i in range(n):
        groups.setdefault(find(i), []).append(pads[i])

    rows: list[list[tuple[float, float]]] = []
    for g in groups.values():
        n_g = len(g)
        mx = sum(p[0] for p in g) / n_g
        my = sum(p[1] for p in g) / n_g
        vx, vy = _pca_direction(g)
        rows.append(sorted(g, key=lambda p: (p[0] - mx) * vx + (p[1] - my) * vy))

    # Global sort: Y-first, then X (stable order across components)
    rows.sort(key=lambda r: (sum(p[1] for p in r) / len(r),
                             sum(p[0] for p in r) / len(r)))
    return rows


# ── Dispenser ──────────────────────────────────────────────────

class PasteDispenser:
    """Dispenses solder paste using Gerber-derived pad rows."""

    def __init__(self, motion: MotionController, config: NanoPnPConfig) -> None:
        self._motion = motion
        self._config = config
        p = config.paste_dispensing
        self.FEED_XY = p.feed_xy
        self.FEED_XY_SLOW = p.feed_xy_slow
        self.FEED_Z = p.feed_z
        self.FEED_E0 = p.feed_e0
        self.FEED_PASTE = p.feed_paste
        self.Z_TRAVEL = p.z_travel
        self.E_PRIME = p.e_prime
        self.E_RATE = p.e_rate
        self.E_RETRACT = p.e_retract
        self._tool_off_x = p.tool_offset_x
        self._tool_off_y = p.tool_offset_y
        # State
        self._curr_e = 0.0
        self._primed = False
        self._shift_x = 0.0
        self._shift_y = 0.0
        self._y_flip = False

    def dispense_board(self, placements: list[Placement]) -> int:
        """Back-compat entry: dispense paste for all enabled placements.

        NOTE: this path no longer uses the hardcoded SOIC8/SOT236/LQFN16
        patterns. For the full paste pipeline, call dispense_gerber()
        with a .gbr file path instead.

        For compatibility with existing job flow, this does a best-effort
        fallback: if board.gerber_path is set in config, use that. Otherwise
        log a warning that there's nothing to dispense (placements alone
        don't contain pad geometry).
        """
        gerber_path = getattr(self._config.board, "gerber_path", None)
        if gerber_path:
            return self.dispense_gerber(gerber_path)

        log.warning(
            "dispense_board called without a Gerber file; "
            "set board.gerber_path in config to enable paste dispensing"
        )
        return 0

    def dispense_paste(
        self,
        edge_cuts_path: str | Path,
        paste_path: str | Path,
        refs: list[str] | None = None,
        no_dispense: bool = False,
    ) -> int:
        """Dispense paste by anchoring each component's Gerber pads around
        its placement position from config.

        Args:
            refs: if given, dispense only these refs (ignores enabled flag).
                  If None, dispense all enabled placements.
        """
        components = parse_gbr(paste_path)
        if not components:
            log.warning("No components parsed from %s", paste_path)
            return 0

        # Build ref → machine position from placements
        if refs is not None:
            ref_set = set(refs)
            place_map: dict[str, tuple[float, float]] = {
                p.ref: (p.x, p.y)
                for p in self._config.placements
                if p.ref in ref_set
            }
        else:
            place_map = {
                p.ref: (p.x, p.y)
                for p in self._config.placements
                if p.enabled
            }

        log.info("[paste] dispensing %d components from %s%s",
                 len(components), Path(paste_path).name,
                 " (NO DISPENSE)" if no_dispense else "")
        self._no_dispense = no_dispense
        self._init_extruder()
        self._primed = False
        dispensed = 0

        for idx, (ref, pads) in enumerate(components.items(), start=1):
            if ref not in place_map:
                log.warning("[paste] %s: no placement in config — skipping", ref)
                continue

            # Per-component shift: center pads on placement position
            gcx = sum(p[0] for p in pads) / len(pads)
            gcy = sum(p[1] for p in pads) / len(pads)
            mx, my = place_map[ref]
            self._shift_x = mx - gcx
            self._shift_y = my - gcy

            rows = group_into_rows(pads)
            log.info("[paste] (%d) %s — %d pads, %d row(s), center → (%.2f, %.2f)",
                     idx, ref, len(pads), len(rows), mx, my)
            for row_idx, row in enumerate(rows, start=1):
                fx, fy = self._transform(*row[0])
                lx, ly = self._transform(*row[-1])
                log.info("[paste]   row %d: (%.2f, %.2f) → (%.2f, %.2f)",
                         row_idx, fx, fy, lx, ly)
                self._dispense_row(row)
            dispensed += 1

        self._finish()
        log.info("[paste] done — %d components dispensed", dispensed)
        return dispensed

    def dispense_gerber(self, gerber_path: str | Path) -> int:
        """Dispense paste from a Gerber using global min-pad anchoring.

        Uses `min(pads)` + `board.first_pad_offset` as the corner reference.
        Prefer `dispense_paste()` which anchors per-component.
        """
        components = parse_gbr(gerber_path)
        if not components:
            log.warning("No components parsed from %s", gerber_path)
            return 0

        all_x = [p[0] for pads in components.values() for p in pads]
        all_y = [p[1] for pads in components.values() for p in pads]

        fp = self._config.board.first_pad_offset
        bo = self._config.board.origin
        self._shift_x = -min(all_x) + fp.x + bo.x
        self._shift_y = -min(all_y) + fp.y + bo.y

        log.info("[paste] dispense_gerber: shift=(%.3f, %.3f)", self._shift_x, self._shift_y)

        self._init_extruder()
        self._primed = False
        dispensed = 0
        for ref, pads in components.items():
            rows = group_into_rows(pads)
            for row in rows:
                self._dispense_row(row)
            dispensed += 1
        self._finish()
        return dispensed

    # ── Internal ───────────────────────────────────────────────

    def _init_extruder(self) -> None:
        """Set up extruder for dispensing, matching the tried-and-tested script exactly."""
        s = self._motion.serial
        s.send("G21")                   # mm mode
        s.send("G90")                   # absolute positioning
        s.send("M302 S0")               # allow cold extrusion (safety for paste)
        s.send("M82")                   # absolute extrusion (E values are cumulative)
        s.send("G92 E0")                # reset extruder position
        self._curr_e = 0.0
        # Per-axis max feedrates. Marlin M203 is mm/s, so /60 from mm/min.
        s.send(
            f"M203 X{self.FEED_XY/60:.2f} Y{self.FEED_XY/60:.2f} "
            f"Z{self.FEED_Z/60:.2f} E{self.FEED_E0/60:.2f}"
        )
        s.send(f"G1 Z{self.Z_TRAVEL:.4f} F{self.FEED_Z}")
        s.send("T1")                    # select paste dispenser
        # Paste extruder (T1) gets its own E max.
        s.send(f"M203 T1 E{self.FEED_PASTE/60:.2f}")
        # Do NOT call motion.tool_select(1) — it sends M83 (relative) and
        # breaks the cumulative absolute-E pattern below.

    def _dispense_row(self, row: list[tuple[float, float]]) -> None:
        """Dispense along one row of pads using the tried-and-tested pattern.

        Pattern: travel to start, extrude while moving slowly to end, retract.
        No Z movement — nozzle stays at Z_TRAVEL throughout.
        """
        if not row:
            return
        x0, y0 = self._transform(*row[0])
        xn, yn = self._transform(*row[-1])
        s = self._motion.serial

        # 1. Fast travel to row start
        s.send(f"G0 X{x0:.4f} Y{y0:.4f} F1000")

        if self._no_dispense:
            # Movement only — travel to row end, pause, no extrusion
            s.send(f"G0 X{xn:.4f} Y{yn:.4f} F1000")
            s.send("G4 P200")
            return

        # 1a. Prime the paste line on the very first row so paste is flowing
        #     before we start the across-row move.
        if not self._primed:
            self._curr_e += self.E_PRIME
            s.send(f"G1 E{self._curr_e:04.0f} F{self.FEED_PASTE}")
            self._primed = True

        # 2. Extrude while moving slowly to the row end
        self._curr_e += self.E_RATE
        s.send(f"G1 E{self._curr_e:04.0f} F{self.FEED_PASTE}")
        s.send(f"G1 X{xn:.4f} Y{yn:.4f} F{self.FEED_XY_SLOW}")

        # 3. Retract + dwell
        self._curr_e -= self.E_RETRACT
        s.send(f"G1 E{self._curr_e:04.0f} F{self.FEED_PASTE}")
        s.send("G4 P200")

    def _finish(self) -> None:
        """Return to origin and end program (matches tried-and-tested script)."""
        s = self._motion.serial
        s.send("G1 X0 Y0 F1000")
        s.send("M30")                   # end of program

    def _transform(self, x: float, y: float) -> tuple[float, float]:
        """Apply Gerber→machine shift, tool offset, and optional Y-flip."""
        new_x = x + self._shift_x + self._tool_off_x
        new_y = (-y if self._y_flip else y) + self._shift_y + self._tool_off_y
        return new_x, new_y
