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

        Pipeline (from OpenPnP vision-settings.xml):
            1. GaussianBlur(k=blur_kernel)
            2. MaskCircle(d=mask_diameter) — crop to circular ROI
            3. BGR → HSV_FULL → MaskHsv(hsv thresholds) → HSV → BGR → Gray
            4. Threshold (binary)
            5. FindContours → FilterContours(min_area)
            6. MinAreaRect on all contour points → center, size, angle
            7. Convert pixel offsets to mm via units_per_pixel

        Per-part overrides (from OpenPnP):
            IC_SOIC8: threshold=249, min_area=618.84px, partmask=600
            IC_SOT236/IC_LQFN16: use stock defaults
        """
        frame = self.capture(settle=True)
        h, w = frame.shape[:2]
        v = self._vision_config
        debug = frame.copy()

        # Per-part overrides (from vision-settings.xml)
        threshold = v.threshold
        min_area_px = v.min_contour_area  # will be converted below
        partmask_d = 100000
        if part_id == "IC_SOIC8":
            threshold = 249
            min_area_px = 618.84
            partmask_d = 600
        else:
            # Convert min_area from sq mm to sq pixels for stock pipeline
            px_per_mm_x = 1.0 / self._cam_config.units_per_pixel.x
            px_per_mm_y = 1.0 / self._cam_config.units_per_pixel.y
            min_area_px = v.min_contour_area * px_per_mm_x * px_per_mm_y

        # 1. GaussianBlur
        k = v.blur_kernel
        if k % 2 == 0:
            k += 1
        blurred = cv2.GaussianBlur(frame, (k, k), 0)

        # 2. MaskCircle (general + partmask)
        center = (w // 2, h // 2)
        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.circle(mask, center, v.mask_diameter // 2, 255, -1)
        masked = cv2.bitwise_and(blurred, blurred, mask=mask)

        if partmask_d < 100000:
            partmask = np.zeros((h, w), dtype=np.uint8)
            cv2.circle(partmask, center, partmask_d // 2, 255, -1)
            masked = cv2.bitwise_and(masked, masked, mask=partmask)

        # 3. BGR → HSV_FULL → MaskHsv
        hsv = cv2.cvtColor(masked, cv2.COLOR_BGR2HSV_FULL)
        hsv_params = v.hsv
        lower = np.array([hsv_params.hue_min, hsv_params.sat_min, hsv_params.val_min])
        upper = np.array([hsv_params.hue_max, hsv_params.sat_max, hsv_params.val_max])
        hsv_mask = cv2.inRange(hsv, lower, upper)

        # HSV → BGR → Gray (following OpenPnP's stock pipeline)
        hsv_applied = cv2.bitwise_and(masked, masked, mask=hsv_mask)
        gray = cv2.cvtColor(hsv_applied, cv2.COLOR_BGR2GRAY)

        # 4. Threshold
        _, binary = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)

        # 5. FindContours → FilterContours
        contours, _ = cv2.findContours(binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        filtered = [c for c in contours if cv2.contourArea(c) >= min_area_px]

        if not filtered:
            log.warning("detect_part: no contours after filtering (threshold=%d, min_area=%.1f)",
                        threshold, min_area_px)
            return VisionResult(dx_mm=0, dy_mm=0, drot_deg=0, confidence=0, frame=debug)

        # 6. MinAreaRect on all filtered contour points combined
        all_points = np.vstack(filtered)
        rect = cv2.minAreaRect(all_points)
        rect_center, rect_size, rect_angle = rect

        # Offset from image center (pixels)
        offset_x_px = rect_center[0] - (w / 2)
        offset_y_px = rect_center[1] - (h / 2)

        # 7. Convert to mm
        dx_mm = offset_x_px * self._cam_config.units_per_pixel.x
        dy_mm = offset_y_px * self._cam_config.units_per_pixel.y

        # Normalize angle: MinAreaRect returns [-90, 0)
        rw, rh = rect_size
        angle = rect_angle
        if rw < rh:
            angle += 90
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        drot_deg = angle

        # Confidence: ratio of detected area to expected part area
        total_area = sum(cv2.contourArea(c) for c in filtered)
        confidence = min(1.0, total_area / max(1, min_area_px * 10))

        # Draw debug overlay
        box = cv2.boxPoints(rect)
        box = np.int32(box)
        cv2.drawContours(debug, [box], 0, (0, 255, 0), 2)
        cv2.circle(debug, (int(rect_center[0]), int(rect_center[1])), 5, (0, 0, 255), -1)
        cv2.circle(debug, center, 5, (255, 0, 0), -1)

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
