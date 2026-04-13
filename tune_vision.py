"""Iterate over captured camera images and find the right CV pipeline.

Run this to see the pipeline output on all captured frames:
    python tune_vision.py

It tests different strategies:
    1. Value-channel threshold (find bright white pads)
    2. Lab L-channel threshold (alternative brightness)
    3. Adaptive threshold
    4. Canny edge + contour
"""

from __future__ import annotations

import glob
import os
import sys
import cv2
import numpy as np


def detect_pads_value_threshold(frame, threshold=200, min_area=80, max_area=5000):
    """Strategy 1: Find bright white pads via HSV Value channel threshold.

    This is the right approach for SOIC8 images — the pads are essentially
    white/metallic and reflect more light than anything else in frame.
    """
    h, w = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Threshold to isolate bright regions
    _, binary = cv2.threshold(blurred, threshold, 255, cv2.THRESH_BINARY)

    # Morphological opening to clean up specks
    kernel = np.ones((3, 3), np.uint8)
    cleaned = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

    # Find contours
    contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Filter by area (reject specks and background glare)
    pads = []
    for c in contours:
        area = cv2.contourArea(c)
        if min_area <= area <= max_area:
            pads.append(c)

    return pads, binary


def fit_part_rect(pads, image_shape):
    """Fit a minimum-area rectangle around all detected pads.

    Returns (center, size, angle) or None if not enough pads.
    """
    if len(pads) < 4:
        return None
    all_points = np.vstack(pads)
    rect = cv2.minAreaRect(all_points)
    return rect


def draw_result(frame, pads, rect, title=""):
    """Draw the pipeline result on the frame."""
    out = frame.copy()
    h, w = out.shape[:2]
    img_center = (w // 2, h // 2)

    # Draw detected pad contours in yellow
    cv2.drawContours(out, pads, -1, (0, 255, 255), 2)

    # Draw image center in blue
    cv2.drawLine = lambda img, p1, p2, color, thickness: cv2.line(img, p1, p2, color, thickness)
    cv2.line(out, (img_center[0] - 20, img_center[1]),
             (img_center[0] + 20, img_center[1]), (255, 0, 0), 2)
    cv2.line(out, (img_center[0], img_center[1] - 20),
             (img_center[0], img_center[1] + 20), (255, 0, 0), 2)

    info = f"{len(pads)} pads"

    if rect:
        center, size, angle = rect
        box = cv2.boxPoints(rect)
        box = np.int32(box)
        cv2.drawContours(out, [box], 0, (0, 255, 0), 3)

        # Detected center in red
        cv2.circle(out, (int(center[0]), int(center[1])), 6, (0, 0, 255), -1)

        # Offset from image center
        dx_px = center[0] - img_center[0]
        dy_px = center[1] - img_center[1]

        info = f"{len(pads)} pads | {size[0]:.0f}x{size[1]:.0f}px | offset ({dx_px:+.0f},{dy_px:+.0f}) | angle {angle:.1f}"

    # Title bar
    cv2.rectangle(out, (0, 0), (w, 30), (0, 0, 0), -1)
    cv2.putText(out, title, (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(out, info, (10, h - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1, cv2.LINE_AA)

    return out


def process_image(path, threshold=200):
    """Run pipeline on one image and return annotated result."""
    frame = cv2.imread(path)
    if frame is None:
        return None, None

    pads, binary = detect_pads_value_threshold(frame, threshold=threshold)
    rect = fit_part_rect(pads, frame.shape)

    result = draw_result(frame, pads, rect, title=os.path.basename(path))
    return result, binary


def main():
    images = sorted(glob.glob("/Users/manitjhajharia/Desktop/NanoPnP/capture_*.png"))
    if not images:
        print("No capture_*.png files found")
        sys.exit(1)

    print(f"Found {len(images)} images")
    print()

    # Try multiple threshold values on first image to find best
    first = images[0]
    frame = cv2.imread(first)
    print(f"Testing thresholds on {os.path.basename(first)}:")
    best_threshold = 200
    best_pad_count = 0

    for thresh in [150, 170, 180, 190, 200, 210, 220, 230]:
        pads, _ = detect_pads_value_threshold(frame, threshold=thresh)
        rect = fit_part_rect(pads, frame.shape)
        valid = rect is not None and 4 <= len(pads) <= 12
        marker = " ← GOOD" if valid else ""
        print(f"  threshold={thresh}: {len(pads)} pads{marker}")
        if valid and (len(pads) - 8) ** 2 < (best_pad_count - 8) ** 2:
            best_threshold = thresh
            best_pad_count = len(pads)

    print(f"\nBest threshold: {best_threshold} ({best_pad_count} pads)")
    print()

    # Process all images with best threshold
    os.makedirs("/Users/manitjhajharia/Desktop/NanoPnP/vision_results", exist_ok=True)
    summary = []
    for i, img_path in enumerate(images):
        result, binary = process_image(img_path, threshold=best_threshold)
        if result is None:
            continue

        # Also save binary for debugging
        base = os.path.basename(img_path).replace("capture_", "").replace(".png", "")
        cv2.imwrite(f"/Users/manitjhajharia/Desktop/NanoPnP/vision_results/{base}_result.png", result)
        cv2.imwrite(f"/Users/manitjhajharia/Desktop/NanoPnP/vision_results/{base}_binary.png", binary)

        # Re-compute for summary
        frame = cv2.imread(img_path)
        pads, _ = detect_pads_value_threshold(frame, threshold=best_threshold)
        rect = fit_part_rect(pads, frame.shape)
        if rect:
            center, size, angle = rect
            h, w = frame.shape[:2]
            dx = center[0] - w / 2
            dy = center[1] - h / 2
            summary.append({
                "name": base, "pads": len(pads),
                "dx_px": dx, "dy_px": dy,
                "w_px": size[0], "h_px": size[1], "angle": angle,
            })
            print(f"{i+1:2d}. {base}: {len(pads)} pads, "
                  f"offset ({dx:+.0f}, {dy:+.0f}) px, "
                  f"size {size[0]:.0f}x{size[1]:.0f}, angle {angle:.1f}°")
        else:
            summary.append({"name": base, "pads": len(pads), "failed": True})
            print(f"{i+1:2d}. {base}: {len(pads)} pads - FAILED (need ≥4)")

    print()
    print("Results saved to vision_results/")
    if summary:
        valid = [s for s in summary if "failed" not in s]
        print(f"Success: {len(valid)}/{len(summary)}")


if __name__ == "__main__":
    main()
