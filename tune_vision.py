"""Iterate over captured camera images and find the right CV pipeline.

Run this to see the pipeline output on all captured frames:
    python tune_vision.py                    # auto-detect part type
    python tune_vision.py --part soic8       # force SOIC-8 (8 pads)
    python tune_vision.py --part sot236      # force SOT-23-6 (6 pads, masked)

It tests different threshold values and picks the one that produces
the expected number of pads most consistently.
"""

from __future__ import annotations

import argparse
import glob
import os
import sys
import cv2
import numpy as np


# Per-part defaults
PART_CONFIGS = {
    "soic8":  {"expected_pads": 8, "min_area": 500, "max_area": 6000, "mask": False, "thresholds": [170, 180, 190, 200, 210, 220, 230]},
    "sot236": {"expected_pads": 6, "min_area": 50,  "max_area": 800,  "mask": True,  "thresholds": [145, 150, 155, 160, 165, 170, 175, 180]},
    "lqfn16": {"expected_pads": 16, "min_area": 50, "max_area": 800,  "mask": True,  "thresholds": [145, 150, 155, 160, 165, 170, 175, 180]},
}
MASK_RADIUS = 250


def detect_pads(frame, threshold=200, min_area=80, max_area=5000, use_mask=False):
    """Find bright pads via grayscale threshold, optionally within a circular mask."""
    h, w = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    if use_mask:
        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.circle(mask, (w // 2, h // 2), MASK_RADIUS, 255, -1)
        gray = cv2.bitwise_and(gray, gray, mask=mask)

    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, binary = cv2.threshold(blurred, threshold, 255, cv2.THRESH_BINARY)

    kernel = np.ones((3, 3), np.uint8)
    cleaned = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    pads = [c for c in contours if min_area <= cv2.contourArea(c) <= max_area]

    return pads, binary


def density_cluster(pads, expected):
    """Select the `expected` most densely-packed contours."""
    if len(pads) <= expected:
        return pads

    centroids = []
    for c in pads:
        M = cv2.moments(c)
        if M["m00"] > 0:
            centroids.append((M["m10"] / M["m00"], M["m01"] / M["m00"], c))

    if len(centroids) <= expected:
        return [c[2] for c in centroids]

    pts = np.array([(c[0], c[1]) for c in centroids])
    n = len(pts)
    dx = pts[:, 0:1] - pts[:, 0:1].T
    dy = pts[:, 1:2] - pts[:, 1:2].T
    dist_mat = np.sqrt(dx * dx + dy * dy)

    scores = np.zeros(n)
    k = min(expected - 1, n - 1)
    for i in range(n):
        sorted_d = np.sort(dist_mat[i])
        scores[i] = np.sum(sorted_d[1:1 + k])

    keep = np.argsort(scores)[:expected]
    return [centroids[i][2] for i in sorted(keep)]


def fit_part_rect(pads, image_shape):
    """Fit a minimum-area rectangle around all detected pads."""
    if len(pads) < 4:
        return None
    all_points = np.vstack(pads)
    return cv2.minAreaRect(all_points)


def draw_result(frame, pads, rect, title=""):
    """Draw the pipeline result on the frame."""
    out = frame.copy()
    h, w = out.shape[:2]
    img_center = (w // 2, h // 2)

    cv2.drawContours(out, pads, -1, (0, 255, 255), 2)
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
        cv2.circle(out, (int(center[0]), int(center[1])), 6, (0, 0, 255), -1)

        dx_px = center[0] - img_center[0]
        dy_px = center[1] - img_center[1]
        info = (f"{len(pads)} pads | {size[0]:.0f}x{size[1]:.0f}px | "
                f"offset ({dx_px:+.0f},{dy_px:+.0f}) | angle {angle:.1f}")

    cv2.rectangle(out, (0, 0), (w, 30), (0, 0, 0), -1)
    cv2.putText(out, title, (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(out, info, (10, h - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1, cv2.LINE_AA)

    return out


def guess_part(frame):
    """Guess part type from image: count pads at multiple thresholds."""
    # Try SOIC-8 settings first (no mask, high threshold)
    pads_soic, _ = detect_pads(frame, threshold=190, min_area=500, max_area=6000, use_mask=False)
    if 6 <= len(pads_soic) <= 10:
        return "soic8"
    # Try SOT settings (masked, lower threshold)
    pads_sot, _ = detect_pads(frame, threshold=165, min_area=50, max_area=800, use_mask=True)
    if 4 <= len(pads_sot) <= 8:
        return "sot236"
    return "soic8"  # fallback


def main():
    parser = argparse.ArgumentParser(description="Tune vision pipeline on captured images")
    parser.add_argument("--part", choices=list(PART_CONFIGS.keys()),
                        help="Part type (auto-detected if omitted)")
    args = parser.parse_args()

    images = sorted(glob.glob("/Users/manitjhajharia/Desktop/NanoPnP/capture_*.png"))
    if not images:
        print("No capture_*.png files found")
        sys.exit(1)

    print(f"Found {len(images)} images")

    # Auto-detect part type if not specified
    part_type = args.part
    if not part_type:
        frame = cv2.imread(images[0])
        part_type = guess_part(frame)
        print(f"Auto-detected part type: {part_type}")

    cfg = PART_CONFIGS[part_type]
    expected = cfg["expected_pads"]
    print(f"Targeting {expected} pads ({part_type}), mask={'on' if cfg['mask'] else 'off'}")
    print()

    # Find best threshold on first image
    first = cv2.imread(images[0])
    print(f"Testing thresholds on {os.path.basename(images[0])}:")
    best_threshold = cfg["thresholds"][len(cfg["thresholds"]) // 2]
    best_score = 999

    for thresh in cfg["thresholds"]:
        pads, _ = detect_pads(first, threshold=thresh,
                              min_area=cfg["min_area"], max_area=cfg["max_area"],
                              use_mask=cfg["mask"])
        if len(pads) > expected:
            pads = density_cluster(pads, expected)
        rect = fit_part_rect(pads, first.shape)
        valid = rect is not None and 4 <= len(pads)
        marker = " ← GOOD" if valid and len(pads) == expected else ""
        print(f"  threshold={thresh}: {len(pads)} pads{marker}")
        score = abs(len(pads) - expected)
        if valid and score < best_score:
            best_threshold = thresh
            best_score = score

    print(f"\nBest threshold: {best_threshold}")
    print()

    # Process all images
    os.makedirs("/Users/manitjhajharia/Desktop/NanoPnP/vision_results", exist_ok=True)
    summary = []
    for i, img_path in enumerate(images):
        frame = cv2.imread(img_path)
        if frame is None:
            continue

        pads, binary = detect_pads(frame, threshold=best_threshold,
                                   min_area=cfg["min_area"], max_area=cfg["max_area"],
                                   use_mask=cfg["mask"])
        if len(pads) > expected:
            pads = density_cluster(pads, expected)

        rect = fit_part_rect(pads, frame.shape)
        result = draw_result(frame, pads, rect, title=os.path.basename(img_path))

        base = os.path.basename(img_path).replace("capture_", "").replace(".png", "")
        cv2.imwrite(f"/Users/manitjhajharia/Desktop/NanoPnP/vision_results/{base}_result.png", result)
        cv2.imwrite(f"/Users/manitjhajharia/Desktop/NanoPnP/vision_results/{base}_binary.png", binary)

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
        perfect = [s for s in valid if s["pads"] == expected]
        print(f"Success: {len(valid)}/{len(summary)}, exact pad count: {len(perfect)}/{len(summary)}")


if __name__ == "__main__":
    main()
