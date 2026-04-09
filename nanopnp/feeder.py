"""Tape feeder management: pick positions, feed tracking, state persistence.

Each feeder holds a strip of tape with components at regular pitch intervals.
Two reference holes (ref_hole, last_hole) define the tape direction. Parts
are picked at: ref_hole + feed_count * pitch * tape_direction.

The feeder's rotation field indicates the part orientation in the tape
(per EIA-481). The PnP engine uses this to compute nozzle pre-rotation.
"""

from __future__ import annotations

import json
import logging
import math
from pathlib import Path

from nanopnp.config import NanoPnPConfig, FeederConfig, XYZ

log = logging.getLogger(__name__)


class FeederError(Exception):
    """Raised when a feeder operation fails."""


class FeederManager:
    """Calculates pick positions and tracks tape feed state."""

    def __init__(self, config: NanoPnPConfig) -> None:
        self._config = config
        # Work with the config's feeder dict directly — mutations to
        # feed_count are reflected back in the config object.
        self._feeders = config.feeders

    # ── Lookup ──────────────────────────────────────────────

    def get_feeder(self, part_id: str) -> FeederConfig:
        """Return first enabled feeder for part_id with remaining parts.

        Raises FeederError if none available.
        """
        for feeder in self._feeders.values():
            if feeder.part_id == part_id and feeder.enabled and self.remaining(feeder) > 0:
                return feeder
        raise FeederError(f"No enabled feeder with parts remaining for {part_id!r}")

    # ── Tape geometry ───────────────────────────────────────

    def tape_direction(self, feeder: FeederConfig) -> tuple[float, float]:
        """Normalized 2D direction vector from ref_hole to last_hole."""
        dx = feeder.last_hole.x - feeder.ref_hole.x
        dy = feeder.last_hole.y - feeder.ref_hole.y
        length = math.hypot(dx, dy)
        if length < 0.01:
            raise FeederError(
                f"Feeder {feeder.slot}: ref_hole and last_hole are too close "
                f"({length:.3f}mm) to determine tape direction"
            )
        return dx / length, dy / length

    def get_pick_position(self, feeder: FeederConfig) -> XYZ:
        """Calculate pick position for current feed_count.

        pick_pos = ref_hole + feed_count * pitch * tape_direction
        Z is always ref_hole.z (tape surface height).
        """
        dir_x, dir_y = self.tape_direction(feeder)
        offset = feeder.feed_count * feeder.pitch
        return XYZ(
            x=round(feeder.ref_hole.x + offset * dir_x, 4),
            y=round(feeder.ref_hole.y + offset * dir_y, 4),
            z=feeder.ref_hole.z,
        )

    def get_pick_rotation(self, feeder: FeederConfig) -> float:
        """Return the part rotation in the tape (degrees).

        The PnP engine computes nozzle angle as:
            nozzle_rotation = placement.rotation - pick_rotation
        """
        return feeder.rotation

    # ── Feed tracking ───────────────────────────────────────

    def remaining(self, feeder: FeederConfig) -> int:
        """Number of parts left in this feeder."""
        return max(0, feeder.max_count - feeder.feed_count)

    def advance(self, feeder: FeederConfig) -> None:
        """Increment feed_count after a successful pick."""
        if self.remaining(feeder) <= 0:
            raise FeederError(f"Feeder {feeder.slot}: no parts remaining")
        feeder.feed_count += 1
        log.debug("Feeder %s: advanced to count %d (%d remaining)",
                   feeder.slot, feeder.feed_count, self.remaining(feeder))

    def reset(self, feeder: FeederConfig) -> None:
        """Reset feed_count to 0 (e.g. after reloading tape)."""
        feeder.feed_count = 0

    # ── State persistence ───────────────────────────────────

    def save_state(self, path: str | Path) -> None:
        """Save all feeder feed_counts to a JSON file."""
        state = {
            slot: feeder.feed_count
            for slot, feeder in self._feeders.items()
        }
        Path(path).write_text(json.dumps(state, indent=2))
        log.info("Feeder state saved to %s", path)

    def load_state(self, path: str | Path) -> None:
        """Restore feed_counts from a previously saved JSON file.

        Silently ignores slots in the file that don't exist in config,
        and leaves feeders not in the file at their current count.
        """
        p = Path(path)
        if not p.exists():
            log.warning("Feeder state file not found: %s", p)
            return

        state = json.loads(p.read_text())
        for slot, count in state.items():
            if slot in self._feeders:
                self._feeders[slot].feed_count = int(count)
                log.debug("Feeder %s: restored count to %d", slot, count)

    # ── Summary ─────────────────────────────────────────────

    def status(self) -> list[dict]:
        """Return status summary for all feeders."""
        result = []
        for slot, feeder in self._feeders.items():
            try:
                pos = self.get_pick_position(feeder)
                pos_str = f"({pos.x}, {pos.y}, {pos.z})"
            except FeederError:
                pos_str = "invalid"
            result.append({
                "slot": slot,
                "part_id": feeder.part_id,
                "enabled": feeder.enabled,
                "feed_count": feeder.feed_count,
                "remaining": self.remaining(feeder),
                "next_pick": pos_str,
                "rotation": feeder.rotation,
            })
        return result
