"""Pick-and-place job orchestration: pick → align → place.

Coordinates MotionController, FeederManager, and VisionSystem to execute
the full placement cycle for each component. Vision is optional — if no
camera is available, parts are placed at nominal positions without alignment.

Follows the workflow from PnP_TODO.md steps 1-15.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass, field

from nanopnp.config import NanoPnPConfig, Placement
from nanopnp.motion import MotionController
from nanopnp.feeder import FeederManager, FeederError

log = logging.getLogger(__name__)

# Try importing vision — it's optional
try:
    from nanopnp.vision import VisionSystem, VisionError
except ImportError:
    VisionSystem = None
    VisionError = Exception


# ── Result types ──────────────────────────────────────────────


@dataclass(slots=True)
class PlacementResult:
    ref: str
    part_id: str
    success: bool
    nominal: tuple[float, float, float]       # (x, y, rotation)
    corrected: tuple[float, float, float]     # (x, y, rotation) after vision
    corrections: tuple[float, float, float]   # (dx, dy, drot) applied
    vision_passes: int
    error: str
    duration_s: float


@dataclass(slots=True)
class JobResult:
    placed: int
    failed: int
    skipped: int
    total: int
    placements: list[PlacementResult]
    duration_s: float
    cph: float


# ── Engine ────────────────────────────────────────────────────


class PnPEngine:
    """Orchestrates the full pick-and-place cycle."""

    MAX_RETRIES = 3

    def __init__(self, config: NanoPnPConfig, motion: MotionController,
                 feeders: FeederManager, vision=None,
                 vision_enabled: bool = False) -> None:
        self._config = config
        self._motion = motion
        self._feeders = feeders
        self._vision = vision
        self.vision_enabled = vision_enabled

        self._z = config.z_heights
        self._cam = config.camera.position
        self._board_z = config.board.place_z
        self._max_passes = config.vision.max_passes
        self._converge_mm = config.vision.converge_mm
        self._converge_deg = config.vision.converge_deg

        # Threading controls
        self._pause_event = threading.Event()
        self._pause_event.set()  # not paused
        self._stop_flag = False

        # Drift stats
        self._drift_dx = 0.0
        self._drift_dy = 0.0
        self._drift_drot = 0.0
        self._drift_count = 0

    # ── Job execution ─────────────────────────────────────────

    def run_job(self, placements: list[Placement], paste_dispenser=None,
                paste_first: bool = False, progress_cb=None) -> JobResult:
        """Execute the full PnP job for all enabled placements.

        Args:
            placements: list of Placement objects
            paste_dispenser: optional PasteDispenser to run first
            paste_first: if True and paste_dispenser given, dispense paste before placing
            progress_cb: callback(index, total, ref, status_str) for GUI updates
        """
        self._stop_flag = False
        self._pause_event.set()
        start = time.time()

        enabled = [p for p in placements if p.enabled]

        # Phase 1: paste (optional)
        if paste_first and paste_dispenser and enabled:
            log.info("Phase 1: Solder paste dispensing")
            if progress_cb:
                progress_cb(0, len(enabled), "--", "Dispensing paste...")
            paste_dispenser.dispense_board(enabled)

        # Phase 2: pick and place
        log.info("Phase 2: Pick and place (%d components)", len(enabled))
        self._motion.tool_select(0)  # ensure T0 nozzle

        results: list[PlacementResult] = []
        placed = 0
        failed = 0
        skipped = 0

        for i, p in enumerate(placements):
            if self._stop_flag:
                break
            self._pause_event.wait()

            if not p.enabled:
                skipped += 1
                continue

            if progress_cb:
                progress_cb(i, len(enabled), p.ref, "Picking...")

            result = self._pick_and_place_with_retry(p)
            results.append(result)

            if result.success:
                placed += 1
                if progress_cb:
                    progress_cb(i + 1, len(enabled), p.ref, "Placed OK")
            else:
                failed += 1
                if progress_cb:
                    progress_cb(i + 1, len(enabled), p.ref, f"FAILED: {result.error}")

        # Return home
        self._motion.safe_move_to(0, 0)

        elapsed = time.time() - start
        cph = (placed / elapsed * 3600) if elapsed > 0 and placed > 0 else 0

        log.info("Job complete: %d placed, %d failed, %d skipped in %.1fs (%.0f CPH)",
                 placed, failed, skipped, elapsed, cph)

        return JobResult(
            placed=placed, failed=failed, skipped=skipped,
            total=len(placements), placements=results,
            duration_s=round(elapsed, 1), cph=round(cph, 0),
        )

    def _pick_and_place_with_retry(self, placement: Placement) -> PlacementResult:
        """Try pick-and-place up to MAX_RETRIES times."""
        for attempt in range(self.MAX_RETRIES):
            try:
                return self._pick_and_place(placement)
            except (FeederError, VisionError, Exception) as e:
                log.warning("Attempt %d/%d failed for %s: %s",
                            attempt + 1, self.MAX_RETRIES, placement.ref, e)
                self._discard()
                if attempt == self.MAX_RETRIES - 1:
                    return PlacementResult(
                        ref=placement.ref, part_id=placement.part_id,
                        success=False,
                        nominal=(placement.x, placement.y, placement.rotation),
                        corrected=(0, 0, 0), corrections=(0, 0, 0),
                        vision_passes=0, error=str(e), duration_s=0,
                    )
        # unreachable but satisfies type checker
        raise RuntimeError("retry loop exited unexpectedly")

    def _pick_and_place(self, p: Placement) -> PlacementResult:
        """Execute one pick → align → place cycle."""
        t0 = time.time()
        part = self._config.parts.get(p.part_id)
        part_height = part.height if part else 1.75

        # ── PICK ──
        feeder = self._feeders.get_feeder(p.part_id)
        pick_pos = self._feeders.get_pick_position(feeder)
        pick_rot = self._feeders.get_pick_rotation(feeder)

        log.info("Pick %s from %s at (%.2f, %.2f, %.1f)",
                 p.ref, feeder.slot, pick_pos.x, pick_pos.y, pick_pos.z)

        self._motion.safe_move_to(pick_pos.x, pick_pos.y)
        self._motion.move_to(z=pick_pos.z)
        self._motion.vacuum_on()
        self._motion.move_to(z=self._z.safe)

        # ── ROTATE ──
        # Compensate for part orientation in tape:
        # nozzle_angle = desired_placement_angle - angle_part_sits_in_tape
        nozzle_rot = p.rotation - pick_rot
        self._motion.move_to(e=nozzle_rot)

        # ── ALIGN ──
        dx, dy, drot = 0.0, 0.0, 0.0
        vision_passes = 0

        if self.vision_enabled and self._vision is not None:
            # Travel to camera (nozzle already pre-rotated)
            self._motion.safe_move_to(self._cam.x, self._cam.y)
            self._motion.move_to(z=part_height)

            # Multi-pass alignment
            dx, dy, drot, vision_passes = self._align(part, part_id=p.part_id)

            # Retract from camera
            self._motion.move_to(z=self._z.safe)

        # ── PLACE ──
        nominal = (p.x, p.y, p.rotation)

        # Apply rotation-aware correction
        corr_dx, corr_dy = (0.0, 0.0)
        if abs(dx) > 0.001 or abs(dy) > 0.001 or abs(drot) > 0.001:
            from nanopnp.vision import VisionSystem as VS
            corr_dx, corr_dy = VS.compute_placement_correction(dx, dy, drot)

        final_x = p.x + corr_dx
        final_y = p.y + corr_dy
        final_rot = nozzle_rot - drot

        log.info("Place %s at (%.3f, %.3f, %.1f) corrections=(%.3f, %.3f, %.1f)",
                 p.ref, final_x, final_y, final_rot, corr_dx, corr_dy, drot)

        place_z = self._board_z + part_height
        self._motion.safe_move_to(final_x, final_y, e=final_rot)
        self._motion.move_to(z=place_z)
        self._motion.vacuum_off()
        self._motion.move_to(z=self._z.safe)

        # ── BOOKKEEPING ──
        self._feeders.advance(feeder)
        self._update_drift(dx, dy, drot)

        elapsed = time.time() - t0
        return PlacementResult(
            ref=p.ref, part_id=p.part_id, success=True,
            nominal=nominal,
            corrected=(round(final_x, 3), round(final_y, 3), round(final_rot, 1)),
            corrections=(round(dx, 3), round(dy, 3), round(drot, 1)),
            vision_passes=vision_passes,
            error="", duration_s=round(elapsed, 2),
        )

    # ── Sub-steps ─────────────────────────────────────────────

    def _align(self, part, part_id: str = "") -> tuple[float, float, float, int]:
        """Multi-pass vision alignment. Returns (total_dx, total_dy, total_drot, passes)."""
        total_dx, total_dy, total_drot = 0.0, 0.0, 0.0
        body_w = part.body_width if part else 0
        body_h = part.body_length if part else 0

        for pass_num in range(self._max_passes):
            result = self._vision.detect_part(body_w, body_h, part_id=part_id)
            ddx, ddy, ddrot = result.dx_mm, result.dy_mm, result.drot_deg

            linear = math.hypot(ddx, ddy)
            log.debug("Vision pass %d: dx=%.3f dy=%.3f drot=%.1f linear=%.3f",
                      pass_num + 1, ddx, ddy, ddrot, linear)

            total_dx += ddx
            total_dy += ddy
            total_drot += ddrot

            # Check convergence
            if linear < self._converge_mm and abs(ddrot) < self._converge_deg:
                log.info("Vision converged in %d pass(es)", pass_num + 1)
                return total_dx, total_dy, total_drot, pass_num + 1

            # Check limits
            v = self._config.vision
            if linear > v.max_linear_offset_mm:
                raise VisionError(f"Linear offset {linear:.2f}mm exceeds max {v.max_linear_offset_mm}mm")
            if abs(ddrot) > v.max_angular_offset_deg:
                raise VisionError(f"Angular offset {ddrot:.1f}deg exceeds max {v.max_angular_offset_deg}deg")

            # Apply correction: move nozzle to compensate
            cur = self._motion.get_position()
            self._motion.move_to(
                x=cur["X"] - ddx,
                y=cur["Y"] - ddy,
                e=cur["E"] - ddrot,
            )

        log.warning("Vision did not converge in %d passes", self._max_passes)
        return total_dx, total_dy, total_drot, self._max_passes

    def _discard(self) -> None:
        """Move to discard location and release part."""
        log.info("Discarding part")
        self._motion.safe_move_to(0, 0)
        self._motion.move_to(z=self._z.discard)
        self._motion.vacuum_off()
        self._motion.move_to(z=self._z.safe)

    # ── Drift tracking ────────────────────────────────────────

    def _update_drift(self, dx: float, dy: float, drot: float) -> None:
        self._drift_dx += dx
        self._drift_dy += dy
        self._drift_drot += drot
        self._drift_count += 1

    def get_drift_stats(self) -> dict:
        n = self._drift_count
        if n == 0:
            return {"avg_dx": 0, "avg_dy": 0, "avg_drot": 0, "count": 0}
        return {
            "avg_dx": round(self._drift_dx / n, 4),
            "avg_dy": round(self._drift_dy / n, 4),
            "avg_drot": round(self._drift_drot / n, 3),
            "count": n,
        }

    # ── Threading controls ────────────────────────────────────

    def pause(self) -> None:
        self._pause_event.clear()
        log.info("Job paused")

    def resume(self) -> None:
        self._pause_event.set()
        log.info("Job resumed")

    def stop(self) -> None:
        self._stop_flag = True
        self._pause_event.set()  # unblock if paused
        log.info("Job stop requested")
