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
        self.vision_apply = True  # False = detect but ignore corrections

        self._z = config.z_heights
        self._cam = config.camera.position
        self._cam_config = config.camera
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
        self._motion.home()          # G28 — known start state

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
        try:
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
        finally:
            try:
                self._motion.vacuum_off()
            except Exception:
                pass

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
        # Partial retract at the feeder — not all the way to safe Z. The
        # subsequent safe_move_to(camera) will finish the retract.
        self._motion.move_to(z=self._z.retract)

        # ── ALIGN ──
        dx, dy, drot = 0.0, 0.0, 0.0
        vision_passes = 0
        nozzle_rot = p.rotation - pick_rot

        # Camera waypoint — nozzle stays at E=0 (no pre-rotation).
        # Vision detects the part as-picked and applies only a small
        # residual correction.  The bulk rotation to placement angle
        # happens during XY travel to the board.
        self._motion.safe_move_to(self._cam.x, self._cam.y)

        if self.vision_enabled and self._vision is not None:
            # For alignment, drop to imaging height and run the passes.
            self._motion.move_to(z=self._z.retract)
            dx, dy, drot, vision_passes = self._align(part, part_id=p.part_id)
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

        place_z = self._board_z
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
        """Two-phase vision alignment: correct rotation first, then XY.

        Phase 1 straightens the part (E axis only) so that the subsequent
        XY measurement is not distorted by the part swinging around the
        nozzle tip during rotation correction.

        If vision_apply is False, detection still runs (for debug preview)
        but returns zero corrections — the nozzle stays put.
        """
        total_dx, total_dy, total_drot = 0.0, 0.0, 0.0
        body_w = part.body_width if part else 0
        body_h = part.body_length if part else 0
        yaw_max = part.yaw_max if part else 30.0
        yaw_snap = part.yaw_snap if part else 0.0
        v = self._config.vision
        passes_used = 0
        settle_s = self._cam_config.settle_time_ms / 1000.0

        # ── Phase 1: detect and fix yaw ──
        # Retry until we get a valid detection (confidence > 0).
        max_detect_retries = 5
        for attempt in range(max_detect_retries):
            time.sleep(settle_s)
            result = self._vision.detect_part(body_w, body_h, part_id=part_id)
            passes_used += 1
            if result.confidence > 0:
                break
            log.warning("No detection on attempt %d/%d — retrying",
                        attempt + 1, max_detect_retries)
        else:
            log.error("No detection after %d attempts", max_detect_retries)
            return 0.0, 0.0, 0.0, passes_used

        ddx, ddy, ddrot = result.dx_mm, result.dy_mm, result.drot_deg
        ddrot = self._clamp_yaw(ddrot, yaw_max, yaw_snap, part_id)
        linear = math.hypot(ddx, ddy)

        log.debug("Vision pass %d (rot): dx=%.3f dy=%.3f drot=%.1f linear=%.3f",
                  passes_used, ddx, ddy, ddrot, linear)

        if not self.vision_apply:
            log.info("Vision apply disabled — skipping corrections")
            return 0.0, 0.0, 0.0, passes_used

        total_drot += ddrot

        # Limit checks
        if linear > v.max_linear_offset_mm:
            raise VisionError(f"Linear offset {linear:.2f}mm exceeds max {v.max_linear_offset_mm}mm")
        if abs(ddrot) > v.max_angular_offset_deg:
            raise VisionError(f"Angular offset {ddrot:.1f}deg exceeds max {v.max_angular_offset_deg}deg")

        # Already converged on all axes — use these XY values since no
        # rotation will be applied (no swing, so XY is still valid).
        if linear < self._converge_mm and abs(ddrot) < self._converge_deg:
            total_dx += ddx
            total_dy += ddy
            log.info("Vision converged in %d pass(es)", passes_used)
            return total_dx, total_dy, total_drot, passes_used

        # Apply rotation only — do NOT move XY.  The XY offset measured
        # with the part still rotated will be stale after this correction.
        cur = self._motion.get_position()
        self._motion.move_to(e=cur["E"] - ddrot)

        # ── Phase 2: settle, detect, fix XY + residual rotation ────
        for _ in range(self._max_passes - 1):
            # Retry until valid detection
            for attempt in range(max_detect_retries):
                time.sleep(settle_s)
                result = self._vision.detect_part(body_w, body_h, part_id=part_id)
                passes_used += 1
                if result.confidence > 0:
                    break
                log.warning("No detection on attempt %d/%d — retrying",
                            attempt + 1, max_detect_retries)
            ddx, ddy, ddrot = result.dx_mm, result.dy_mm, result.drot_deg
            ddrot = self._clamp_yaw(ddrot, yaw_max, yaw_snap, part_id)
            linear = math.hypot(ddx, ddy)

            log.debug("Vision pass %d (xy): dx=%.3f dy=%.3f drot=%.1f linear=%.3f",
                      passes_used, ddx, ddy, ddrot, linear)

            total_dx += ddx
            total_dy += ddy
            total_drot += ddrot

            if linear < self._converge_mm and abs(ddrot) < self._converge_deg:
                log.info("Vision converged in %d pass(es)", passes_used)
                return total_dx, total_dy, total_drot, passes_used

            if linear > v.max_linear_offset_mm:
                raise VisionError(f"Linear offset {linear:.2f}mm exceeds max {v.max_linear_offset_mm}mm")
            if abs(ddrot) > v.max_angular_offset_deg:
                raise VisionError(f"Angular offset {ddrot:.1f}deg exceeds max {v.max_angular_offset_deg}deg")

            cur = self._motion.get_position()
            self._motion.move_to(
                x=cur["X"] - ddx,
                y=cur["Y"] - ddy,
                e=cur["E"] - ddrot,
            )

        log.warning("Vision did not converge in %d passes", passes_used)
        return total_dx, total_dy, total_drot, passes_used

    @staticmethod
    def _clamp_yaw(drot: float, yaw_max: float, yaw_snap: float, part_id: str) -> float:
        """Apply per-part yaw policy.

        yaw_snap > 0 (square ICs like QFN): return residual after
            snapping to nearest multiple.  Max correction = snap/2.
            e.g. snap=45 → detected 40° → nearest 45° → residual -5°.
        yaw_snap == 0 (rectangular ICs): ignore if |drot| > yaw_max.
        """
        if yaw_snap > 0:
            snapped = round(drot / yaw_snap) * yaw_snap
            residual = drot - snapped
            if abs(snapped) > 0.1:
                log.info("Yaw %.1f° → nearest %g° multiple = %.0f° → residual %.1f° for %s",
                         drot, yaw_snap, snapped, residual, part_id)
            return residual
        if abs(drot) > yaw_max:
            log.warning("Yaw %.1f° exceeds ±%.1f° for %s — ignoring",
                        drot, yaw_max, part_id)
            return 0.0
        return drot

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
