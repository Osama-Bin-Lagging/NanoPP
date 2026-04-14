"""Bbox annotation tool for vision tuning — 3-point rotated rectangles.

Usage:
    python annotate.py                          # all capture_*.png
    python annotate.py capture_20260414_*.png   # specific files

Controls:
    - Click 3 points: first 2 define one edge, 3rd defines width
    - Press 'n' or Enter to save & go to next image
    - Press 'r' to redo (clear points on current image)
    - Press 'p' to go to previous image
    - Press '1'=SOIC8  '2'=SOT236  '3'=LQFN16 to set part type
    - Press 'q' to quit and save

Saves to vision_results/annotations.json
"""

import glob
import json
import math
import os
import sys
import cv2
import numpy as np

ANNOTATIONS_PATH = "vision_results/annotations.json"
PART_TYPES = {ord('1'): "IC_SOIC8", ord('2'): "IC_SOT236", ord('3'): "IC_LQFN16"}
PART_LABELS = {"IC_SOIC8": "SOIC8 [1]", "IC_SOT236": "SOT236 [2]", "IC_LQFN16": "LQFN16 [3]"}

# State
pts = []
current_box = None  # (center, size, angle)
frame = None
part_type = "IC_SOT236"


def load_annotations():
    if os.path.exists(ANNOTATIONS_PATH):
        return json.loads(open(ANNOTATIONS_PATH).read())
    return {}


def save_annotations(annots):
    os.makedirs(os.path.dirname(ANNOTATIONS_PATH), exist_ok=True)
    open(ANNOTATIONS_PATH, "w").write(json.dumps(annots, indent=2))
    print(f"Saved {len(annots)} annotations to {ANNOTATIONS_PATH}")


def rect_from_3pts(p1, p2, p3):
    """Build a rotated rect from 3 points.

    p1→p2 defines one full edge (length + angle).
    p3 is on the opposite edge — its perpendicular distance from
    the p1→p2 line gives the width.
    """
    # Edge vector
    ex = p2[0] - p1[0]
    ey = p2[1] - p1[1]
    edge_len = math.hypot(ex, ey)
    if edge_len < 1:
        return None

    # Unit vectors along and perpendicular to edge
    ux, uy = ex / edge_len, ey / edge_len
    nx, ny = -uy, ux  # normal (perpendicular)

    # Width = signed projection of (p3 - p1) onto normal
    dx = p3[0] - p1[0]
    dy = p3[1] - p1[1]
    width = abs(dx * nx + dy * ny)

    # Center = midpoint of edge + half-width along normal direction
    mid_x = (p1[0] + p2[0]) / 2
    mid_y = (p1[1] + p2[1]) / 2
    sign = 1 if (dx * nx + dy * ny) > 0 else -1
    center = (mid_x + sign * width / 2 * nx,
              mid_y + sign * width / 2 * ny)

    angle = math.degrees(math.atan2(ey, ex))

    return (center, (edge_len, width), angle)


def draw_overlay(img, box, label="", point_list=None):
    out = img.copy()
    h, w = out.shape[:2]

    # Crosshair at image center
    cx, cy = w // 2, h // 2
    cv2.line(out, (cx - 30, cy), (cx + 30, cy), (255, 0, 0), 1)
    cv2.line(out, (cx, cy - 30), (cx, cy + 30), (255, 0, 0), 1)

    # Draw clicked points
    if point_list:
        for i, p in enumerate(point_list):
            color = (0, 255, 255) if i < 2 else (255, 0, 255)
            cv2.circle(out, (int(p[0]), int(p[1])), 6, color, -1)
            cv2.putText(out, str(i + 1), (int(p[0]) + 8, int(p[1]) - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        # Draw edge line between first 2 points
        if len(point_list) >= 2:
            cv2.line(out, (int(point_list[0][0]), int(point_list[0][1])),
                     (int(point_list[1][0]), int(point_list[1][1])), (0, 255, 255), 2)

    # Draw final box
    if box is not None:
        center, size, angle = box
        rect_pts = cv2.boxPoints((center, size, angle))
        rect_pts = np.int32(rect_pts)
        cv2.drawContours(out, [rect_pts], 0, (0, 255, 0), 2)
        cv2.circle(out, (int(center[0]), int(center[1])), 4, (0, 0, 255), -1)
        cv2.putText(out, f"{size[0]:.0f}x{size[1]:.0f}px  angle={angle:.1f}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # Part type indicator
    pt_label = PART_LABELS.get(part_type, part_type)
    cv2.putText(out, f"Part: {pt_label}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)

    # Help text
    cv2.putText(out, "Click 3 pts: 2 for edge, 1 for width | n=next r=redo p=prev 1/2/3=part q=quit",
                (10, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)

    if label:
        cv2.putText(out, label, (w - 400, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

    return out


def mouse_cb(event, x, y, flags, param):
    global pts, current_box

    if event == cv2.EVENT_LBUTTONDOWN:
        if len(pts) < 3:
            pts.append((x, y))

        if len(pts) == 3:
            current_box = rect_from_3pts(pts[0], pts[1], pts[2])

        display = draw_overlay(frame, current_box, point_list=pts if len(pts) < 3 else None)
        cv2.imshow("Annotate", display)


def main():
    global frame, pts, current_box, part_type

    if len(sys.argv) > 1:
        images = sorted(sys.argv[1:])
    else:
        images = sorted(glob.glob("capture_*.png"))

    if not images:
        print("No images found.")
        sys.exit(1)

    print(f"Found {len(images)} images")
    print("Keys: 1=SOIC8  2=SOT236  3=LQFN16")
    print()

    annots = load_annotations()
    cv2.namedWindow("Annotate", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Annotate", 1280, 720)
    cv2.setMouseCallback("Annotate", mouse_cb)

    i = 0
    while i < len(images):
        img_path = images[i]
        basename = os.path.basename(img_path)
        frame = cv2.imread(img_path)
        if frame is None:
            i += 1
            continue

        # Load existing annotation
        if basename in annots:
            a = annots[basename]
            current_box = ((a["x"], a["y"]), (a["w"], a["h"]), a["angle"])
            part_type = a.get("part", part_type)
        else:
            current_box = None

        pts.clear()
        status = " (done)" if basename in annots else ""
        label = f"[{i + 1}/{len(images)}] {basename}{status}"
        display = draw_overlay(frame, current_box, label)
        cv2.imshow("Annotate", display)

        while True:
            key = cv2.waitKey(0) & 0xFF

            if key == ord('q'):
                save_annotations(annots)
                cv2.destroyAllWindows()
                return

            if key in PART_TYPES:
                part_type = PART_TYPES[key]
                print(f"  Part type: {part_type}")
                display = draw_overlay(frame, current_box, label, point_list=pts if len(pts) < 3 else None)
                cv2.imshow("Annotate", display)

            if key in (ord('n'), 13):  # next
                if current_box is not None:
                    center, size, angle = current_box
                    annots[basename] = {
                        "x": round(center[0], 1), "y": round(center[1], 1),
                        "w": round(size[0], 1), "h": round(size[1], 1),
                        "angle": round(angle, 2),
                        "part": part_type,
                    }
                    print(f"  {basename}: {part_type} {size[0]:.0f}x{size[1]:.0f}px @ {angle:.1f}°")
                i += 1
                break

            if key == ord('r'):
                current_box = None
                pts.clear()
                display = draw_overlay(frame, None, label)
                cv2.imshow("Annotate", display)

            if key == ord('p'):
                i = max(0, i - 1)
                break

    save_annotations(annots)
    cv2.destroyAllWindows()
    print("Done!")


if __name__ == "__main__":
    main()
