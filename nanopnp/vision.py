"""Bottom camera vision system for part alignment.

Handles camera lifecycle, frame capture with settle-time buffering,
and part detection via an OpenCV pipeline. The detection pipeline is
currently a placeholder that returns zero offsets — swap in the real
HSV/contour pipeline when ready for alignment testing.

Camera notes:
    - device_index 0 = built-in webcam (macOS), 1 = USB bottom camera
    - OpenCV buffers ~5 frames internally; capture() flushes the buffer
      to get a fresh frame after machine movement
    - settle_time_ms accounts for vibration after nozzle moves to camera
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass

import cv2
import numpy as np

from nanopnp.config import NanoPnPConfig, CameraConfig

log = logging.getLogger(__name__)


# ── Result types ────────────────────────────────────────────────


@dataclass(slots=True)
class VisionResult:
    """Result of a single detect_part() call."""
    dx_mm: float        # X offset from frame center (mm)
    dy_mm: float        # Y offset from frame center (mm)
    drot_deg: float     # Rotation offset (degrees)
    confidence: float   # 0.0–1.0 detection confidence
    frame: np.ndarray | None = None  # annotated frame for debugging


@dataclass(slots=True)
class AlignmentResult:
    """Result of multi-pass align_part() loop."""
    dx_mm: float
    dy_mm: float
    drot_deg: float
    passes: int
    converged: bool


# ── Vision system ───────────────────────────────────────────────


class VisionError(Exception):
    """Raised when camera or detection fails."""


class VisionSystem:
    """Captures frames and computes part offset/rotation via OpenCV."""

    # Number of frames to grab-and-discard when flushing the buffer
    _BUFFER_FLUSH_COUNT = 5

    def __init__(self, config: NanoPnPConfig) -> None:
        self._cam_config: CameraConfig = config.camera
        self._vision_config = config.vision
        self._cap: cv2.VideoCapture | None = None
        self._frame_count = 0

    # ── Lifecycle ───────────────────────────────────────────

    def open(self) -> None:
        """Open the camera device."""
        if self._cap is not None and self._cap.isOpened():
            return

        idx = self._cam_config.device_index
        log.info("Opening camera device %d", idx)
        self._cap = cv2.VideoCapture(idx)

        if not self._cap.isOpened():
            self._cap = None
            raise VisionError(f"Failed to open camera device {idx}")

        # Log camera properties
        w = self._cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        h = self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        fps = self._cap.get(cv2.CAP_PROP_FPS)
        log.info("Camera opened: %.0fx%.0f @ %.1f FPS", w, h, fps)
        self._frame_count = 0

    def close(self) -> None:
        """Release the camera device."""
        if self._cap is not None:
            self._cap.release()
            self._cap = None
            log.info("Camera closed (captured %d frames)", self._frame_count)

    @property
    def is_open(self) -> bool:
        return self._cap is not None and self._cap.isOpened()

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()

    # ── Pattern matching ────────────────────────────────────

    @staticmethod
    def _match_pad_pattern(centroids: list, expected: int) -> list:
        """Select the `expected` most densely-packed contours.

        Works by computing, for each centroid, the sum of distances to its
        `expected-1` nearest neighbors. Real pads in a tight cluster have
        low sums; outliers (like background glare far from the IC) have
        high sums. Keep the N lowest.

        Returns a list of cv2 contours (not centroid tuples).
        """
        import numpy as np
        if len(centroids) <= expected:
            return [c[2] for c in centroids]

        pts = np.array([(c[0], c[1]) for c in centroids])
        n = len(pts)

        # Pairwise distance matrix
        dx = pts[:, 0:1] - pts[:, 0:1].T
        dy = pts[:, 1:2] - pts[:, 1:2].T
        dist_mat = np.sqrt(dx * dx + dy * dy)

        # For each point, sum of distances to its (expected-1) nearest neighbors
        density_score = np.zeros(n)
        for i in range(n):
            sorted_dists = np.sort(dist_mat[i])
            # sorted_dists[0] is 0 (self), take next (expected-1)
            k = min(expected - 1, n - 1)
            density_score[i] = np.sum(sorted_dists[1:1 + k])

        # Keep the `expected` points with lowest scores (most densely packed)
        keep_idx = np.argsort(density_score)[:expected]
        return [centroids[i][2] for i in sorted(keep_idx)]

    # ── Frame capture ───────────────────────────────────────

    def _ensure_open(self) -> cv2.VideoCapture:
        if not self.is_open:
            raise VisionError("Camera not open — call open() first")
        assert self._cap is not None
        return self._cap

    def capture(self, settle: bool = True) -> np.ndarray:
        """Grab a fresh frame, flushing the internal buffer first.

        If settle=True, waits settle_time_ms before capturing to let
        vibrations die down after machine movement.
        """
        cap = self._ensure_open()

        # Wait for mechanical settle
        if settle:
            time.sleep(self._cam_config.settle_time_ms / 1000.0)

        # Flush stale buffered frames
        for _ in range(self._BUFFER_FLUSH_COUNT):
            cap.grab()

        # Read the fresh frame
        ok, frame = cap.read()
        if not ok or frame is None:
            raise VisionError("Failed to capture frame")

        self._frame_count += 1
        return frame

    def get_frame_for_preview(self) -> np.ndarray | None:
        """Grab a frame without settle delay — for GUI live preview.

        Returns None if camera isn't open or frame grab fails.
        Non-blocking: no buffer flush, no settle wait.
        """
        if not self.is_open:
            return None
        assert self._cap is not None
        ok, frame = self._cap.read()
        if not ok:
            return None
        self._frame_count += 1
        return frame

    # ── Detection pipeline ──────────────────────────────────

    def detect_part(
        self,
        expected_width_mm: float = 0,
        expected_height_mm: float = 0,
        part_id: str = "",
    ) -> VisionResult:
        """Run the OpenCV vision pipeline and return part offset from center.

        Pipeline (tuned from real captured SOIC8 images):
            1. BGR → Gray
            2. GaussianBlur (5x5) to reduce noise
            3. Threshold on brightness (find bright metallic pads)
            4. Morphological opening to clean specks
            5. FindContours (external only)
            6. Filter by area [min_area, max_area] — rejects noise and big glare blobs
            7. MinAreaRect on all filtered contours → center, size, angle
            8. Convert pixel offsets to mm via units_per_pixel

        The key insight: SOIC8 pads are essentially white metallic reflectors,
        so a simple grayscale threshold works much better than HSV for our
        lighting setup.
        """
        frame = self.capture(settle=True)
        h, w = frame.shape[:2]
        v = self._vision_config
        debug = frame.copy()

        # Threshold value: use the configurable threshold from config
        # Good values for shiny pads under mixed lighting: 180-220
        threshold = v.threshold

        # Per-part area limits (in pixels). Tuned from real captured images.
        # SOIC8 pads come out as blobs of ~4000-5500 px under typical lighting.
        if part_id == "IC_SOIC8":
            min_area_px = 500   # reject noise specks (areas ~100-300)
            max_area_px = 6000
        elif part_id == "IC_SOT236":
            min_area_px = 200
            max_area_px = 3000
        elif part_id == "IC_LQFN16":
            min_area_px = 150
            max_area_px = 2500
        else:
            min_area_px = 80
            max_area_px = 5000

        # 1-2. Gray + Blur
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        k = v.blur_kernel
        if k % 2 == 0:
            k += 1
        blurred = cv2.GaussianBlur(gray, (k, k), 0)

        # 3. Threshold — isolate bright regions (pads)
        _, binary = cv2.threshold(blurred, threshold, 255, cv2.THRESH_BINARY)

        # 4. Morphological opening to remove noise specks
        kernel = np.ones((3, 3), np.uint8)
        cleaned = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

        # 5. FindContours (external) — much faster and avoids hierarchy issues
        contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 6. Filter by area (both min and max)
        filtered = [
            c for c in contours
            if min_area_px <= cv2.contourArea(c) <= max_area_px
        ]

        if not filtered:
            log.warning("detect_part [%s]: no contours after filtering (threshold=%d, area=%d-%d)",
                        part_id or "stock", threshold, min_area_px, max_area_px)
            return VisionResult(dx_mm=0, dy_mm=0, drot_deg=0, confidence=0, frame=debug)

        # Reject if too few contours (need at least 4 pads for a meaningful rect)
        if len(filtered) < 4:
            log.warning("detect_part [%s]: only %d pads found, need ≥4",
                        part_id or "stock", len(filtered))
            return VisionResult(dx_mm=0, dy_mm=0, drot_deg=0, confidence=0, frame=debug)

        # Compute centroids for all candidate contours
        centroids = []
        for c in filtered:
            M = cv2.moments(c)
            if M["m00"] > 0:
                cx = M["m10"] / M["m00"]
                cy = M["m01"] / M["m00"]
                centroids.append((cx, cy, c))

        if len(centroids) < 4:
            log.warning("detect_part [%s]: too few valid centroids", part_id or "stock")
            return VisionResult(dx_mm=0, dy_mm=0, drot_deg=0, confidence=0, frame=debug)

        # Pattern-match: for SOIC8 we know there should be exactly 8 pads
        # arranged as two parallel rows of 4. For SOT236: 2 rows of 3. For LQFN16:
        # 4 rows of 4. Pick the subset that best matches the expected count+layout.
        expected_count = {"IC_SOIC8": 8, "IC_SOT236": 6, "IC_LQFN16": 16}.get(part_id, 0)

        if expected_count > 0 and len(centroids) > expected_count:
            filtered = self._match_pad_pattern(centroids, expected_count)
            log.info("detect_part [%s]: matched %d-pad pattern (from %d candidates)",
                     part_id, expected_count, len(centroids))
        elif expected_count > 0 and len(centroids) < expected_count:
            log.warning("detect_part [%s]: found only %d candidates, expected %d",
                        part_id, len(centroids), expected_count)
            filtered = [c[2] for c in centroids]
        else:
            filtered = [c[2] for c in centroids]

        # Post-match size validation: if the bbox is much bigger than the
        # expected part body, iteratively drop the contour farthest from
        # the cluster centroid until the bbox fits or we run out of contours.
        # This catches cases where the density-matcher included a far-away
        # bright spot that stretched the MinAreaRect.
        if expected_width_mm > 0 and expected_height_mm > 0 and len(filtered) > 4:
            expected_max = max(expected_width_mm, expected_height_mm) * 1.5  # 50% slack
            upp = max(self._cam_config.units_per_pixel.x, self._cam_config.units_per_pixel.y)
            expected_max_px = expected_max / upp

            for _ in range(len(filtered) - 4):
                all_points = np.vstack(filtered)
                rect = cv2.minAreaRect(all_points)
                rect_center_tmp, rect_size_tmp, _ = rect
                if max(rect_size_tmp) <= expected_max_px:
                    break  # bbox fits, stop dropping
                # Drop the contour whose centroid is farthest from rect center
                dists = []
                for c in filtered:
                    M = cv2.moments(c)
                    if M["m00"] > 0:
                        cx = M["m10"] / M["m00"]
                        cy = M["m01"] / M["m00"]
                        d = (cx - rect_center_tmp[0]) ** 2 + (cy - rect_center_tmp[1]) ** 2
                        dists.append(d)
                    else:
                        dists.append(0)
                worst_idx = int(np.argmax(dists))
                filtered.pop(worst_idx)

        # 6. MinAreaRect on all filtered contour points combined
        all_points = np.vstack(filtered)
        rect = cv2.minAreaRect(all_points)
        rect_center, rect_size, rect_angle = rect

        # Image → machine coordinate mapping:
        #   bottom of image (+img_y) → +machine_X
        #   left of image   (-img_x) → +machine_Y
        #   yaw is around -Z (looking from top, CCW is positive),
        #     so machine_yaw = 90 - image_yaw (after normalization)
        img_dx_px = rect_center[0] - (w / 2)  # +image X = right
        img_dy_px = rect_center[1] - (h / 2)  # +image Y = down

        # Map to machine frame
        machine_dx_px = img_dy_px    # down in image → +X machine
        machine_dy_px = -img_dx_px   # right in image → -Y machine

        # Convert to mm
        dx_mm = machine_dx_px * self._cam_config.units_per_pixel.x
        dy_mm = machine_dy_px * self._cam_config.units_per_pixel.y

        # Angle computation:
        # Expected camera orientation: IC body's LONG axis VERTICAL
        # (pins on left and right of the image). A correctly oriented
        # part has its long axis at ±90° from horizontal.
        #
        # minAreaRect returns (width, height, angle) where angle is the
        # rotation of the "width" side from horizontal, in [-90, 0).
        # We first compute the long-axis angle (from horizontal), then
        # subtract 90° to express it relative to vertical (the expected
        # orientation).
        rw, rh = rect_size
        raw_angle = rect_angle
        if rw < rh:
            long_axis_angle = raw_angle + 90
        else:
            long_axis_angle = raw_angle

        # Express relative to vertical: subtract 90°.
        # A perfectly vertical long axis (90° from horizontal) becomes 0°.
        rel_angle = long_axis_angle - 90

        # Wrap to smallest equivalent rotation in [-45, 45]
        while rel_angle > 45:
            rel_angle -= 90
        while rel_angle < -45:
            rel_angle += 90

        drot_deg = rel_angle

        # Confidence: ratio of detected area to expected part area
        total_area = sum(cv2.contourArea(c) for c in filtered)
        confidence = min(1.0, total_area / max(1, min_area_px * 10))

        # Draw debug overlay: bbox + center + image center + dimensions in mm
        img_center = (w // 2, h // 2)
        box = cv2.boxPoints(rect)
        box = np.int32(box)
        cv2.drawContours(debug, [box], 0, (0, 255, 0), 2)
        # Also draw the individual pad contours in yellow
        cv2.drawContours(debug, filtered, -1, (0, 255, 255), 1)
        cv2.circle(debug, (int(rect_center[0]), int(rect_center[1])), 5, (0, 0, 255), -1)
        cv2.circle(debug, img_center, 5, (255, 0, 0), -1)

        # Compute physical dimensions of detected bbox
        rw_mm = rect_size[0] * self._cam_config.units_per_pixel.x
        rh_mm = rect_size[1] * self._cam_config.units_per_pixel.y

        # Draw dimensions label near the bbox (smaller, for reference)
        label_x = int(rect_center[0]) + 10
        label_y = int(rect_center[1]) - 10
        cv2.putText(debug, f"{rw_mm:.2f}x{rh_mm:.2f}mm", (label_x, label_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

        # Draw dx/dy/dC prominently in the top-left corner (large font)
        hud_x, hud_y = 20, 40
        line_h = 40
        cv2.rectangle(debug, (hud_x - 10, hud_y - 30), (hud_x + 420, hud_y + line_h * 3 - 10),
                      (0, 0, 0), -1)
        cv2.putText(debug, f"dX: {dx_mm:+.3f} mm", (hud_x, hud_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 200, 255), 2, cv2.LINE_AA)
        cv2.putText(debug, f"dY: {dy_mm:+.3f} mm", (hud_x, hud_y + line_h),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 200, 255), 2, cv2.LINE_AA)
        cv2.putText(debug, f"dC: {drot_deg:+.1f} deg", (hud_x, hud_y + line_h * 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 200, 255), 2, cv2.LINE_AA)

        log.info("detect_part [%s]: dx=%.3fmm dy=%.3fmm drot=%.1f conf=%.2f contours=%d",
                 part_id or "stock", dx_mm, dy_mm, drot_deg, confidence, len(filtered))

        return VisionResult(
            dx_mm=dx_mm,
            dy_mm=dy_mm,
            drot_deg=drot_deg,
            confidence=confidence,
            frame=debug,
        )

    # ── Calibration helpers ─────────────────────────────────

    @staticmethod
    def compute_placement_correction(
        dx_mm: float, dy_mm: float, drot_deg: float,
    ) -> tuple[float, float]:
        """Compute rotation-aware XY correction.

        When the nozzle rotates by -drot to straighten the part,
        the XY offset vector also rotates:
            corrected_dx = dx * cos(-drot) - dy * sin(-drot)
            corrected_dy = dx * sin(-drot) + dy * cos(-drot)
        """
        import math
        rad = math.radians(-drot_deg)
        cos_r = math.cos(rad)
        sin_r = math.sin(rad)
        return (
            dx_mm * cos_r - dy_mm * sin_r,
            dx_mm * sin_r + dy_mm * cos_r,
        )

    def calibrate_upp(self, known_width_mm: float, known_height_mm: float) -> dict:
        """Calibrate units-per-pixel using a known-size object on the camera.

        Place a component or calibration target with known dimensions on the
        bottom camera. This captures a frame, runs the detection pipeline to
        measure the object's size in pixels, then computes mm/pixel.

        Args:
            known_width_mm: actual width of the object in mm
            known_height_mm: actual height of the object in mm

        Returns:
            dict with old_upp, new_upp, measured_px, and the frame
        """
        frame = self.capture(settle=True)
        h, w = frame.shape[:2]
        v = self._vision_config

        # Run detection pipeline to find the object
        k = v.blur_kernel if v.blur_kernel % 2 == 1 else v.blur_kernel + 1
        blurred = cv2.GaussianBlur(frame, (k, k), 0)

        center = (w // 2, h // 2)
        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.circle(mask, center, v.mask_diameter // 2, 255, -1)
        masked = cv2.bitwise_and(blurred, blurred, mask=mask)

        hsv = cv2.cvtColor(masked, cv2.COLOR_BGR2HSV_FULL)
        p = v.hsv
        lower = np.array([p.hue_min, p.sat_min, p.val_min])
        upper = np.array([p.hue_max, p.sat_max, p.val_max])
        hsv_mask = cv2.inRange(hsv, lower, upper)

        applied = cv2.bitwise_and(masked, masked, mask=hsv_mask)
        gray = cv2.cvtColor(applied, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, v.threshold, 255, cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        if not contours:
            raise VisionError("Calibration failed: no object detected. "
                              "Place a component on the camera and ensure lighting is correct.")

        all_points = np.vstack(contours)
        rect = cv2.minAreaRect(all_points)
        rect_center, rect_size, rect_angle = rect

        # rect_size is (width, height) in pixels — ensure consistent ordering
        measured_w_px = max(rect_size)
        measured_h_px = min(rect_size)

        if measured_w_px < 5 or measured_h_px < 5:
            raise VisionError(f"Detected object too small: {measured_w_px:.0f}x{measured_h_px:.0f}px. "
                              "Check that the object is visible and in focus.")

        # Compute new units-per-pixel
        # Match longer measured dimension to longer known dimension
        known_long = max(known_width_mm, known_height_mm)
        known_short = min(known_width_mm, known_height_mm)

        new_upp_x = known_long / measured_w_px
        new_upp_y = known_short / measured_h_px

        old_upp = (self._cam_config.units_per_pixel.x, self._cam_config.units_per_pixel.y)

        log.info("UPP calibration: object %dx%d px, known %.2fx%.2f mm",
                 int(measured_w_px), int(measured_h_px), known_width_mm, known_height_mm)
        log.info("UPP old: (%.6f, %.6f) → new: (%.6f, %.6f)",
                 old_upp[0], old_upp[1], new_upp_x, new_upp_y)

        # Draw debug overlay
        debug = frame.copy()
        box = cv2.boxPoints(rect)
        box = np.int32(box)
        cv2.drawContours(debug, [box], 0, (0, 255, 0), 2)
        cv2.putText(debug, f"{measured_w_px:.0f}x{measured_h_px:.0f}px",
                    (int(rect_center[0]) + 10, int(rect_center[1]) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        return {
            "old_upp": old_upp,
            "new_upp": (round(new_upp_x, 8), round(new_upp_y, 8)),
            "measured_px": (round(measured_w_px, 1), round(measured_h_px, 1)),
            "frame": debug,
        }

    def apply_upp(self, upp_x: float, upp_y: float) -> None:
        """Apply new units-per-pixel values."""
        self._cam_config.units_per_pixel.x = upp_x
        self._cam_config.units_per_pixel.y = upp_y
        log.info("Applied new UPP: (%.8f, %.8f)", upp_x, upp_y)

    # ── FPS benchmark ───────────────────────────────────────

    def benchmark_fps(self, num_frames: int = 30, settle: bool = False) -> dict:
        """Capture num_frames and report actual FPS.

        Useful for verifying camera works and measuring real throughput.
        """
        cap = self._ensure_open()

        times: list[float] = []
        for i in range(num_frames):
            t0 = time.perf_counter()
            ok, frame = cap.read()
            t1 = time.perf_counter()
            if not ok:
                log.warning("Frame %d failed", i)
                continue
            times.append(t1 - t0)

        if not times:
            raise VisionError("No frames captured during benchmark")

        total = sum(times)
        avg = total / len(times)
        fps = len(times) / total

        h, w = 0, 0
        ok, frame = cap.read()
        if ok and frame is not None:
            h, w = frame.shape[:2]

        return {
            "frames_captured": len(times),
            "total_seconds": round(total, 3),
            "avg_ms_per_frame": round(avg * 1000, 2),
            "fps": round(fps, 1),
            "resolution": f"{w}x{h}",
            "min_ms": round(min(times) * 1000, 2),
            "max_ms": round(max(times) * 1000, 2),
        }
