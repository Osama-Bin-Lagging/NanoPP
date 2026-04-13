"""G-code path visualizer — renders toolpaths in 2D with matplotlib.

Can visualize:
- .gcode files (static preview)
- Live G-code stream (step-by-step playback)

Color coding:
- Blue:   XY travel at safe Z (rapid moves)
- Red:    Z descent (pick/place)
- Green:  Z retract
- Orange: Paste extrusion (E axis moving with XY)
- Gray:   G28 home moves

Usage:
    python -m nanopnp.visualizer output/full_job_fixed.gcode
    python -m nanopnp.visualizer --animate output/full_job_fixed.gcode
"""

from __future__ import annotations

import re
import sys
from dataclasses import dataclass, field

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.collections import LineCollection
import numpy as np


# ── G-code parser ─────────────────────────────────────────────

@dataclass
class GMove:
    """A single parsed G-code move."""
    x0: float; y0: float; z0: float; e0: float
    x1: float; y1: float; z1: float; e1: float
    feedrate: float
    cmd: str
    line_num: int
    move_type: str  # "travel", "z_down", "z_up", "paste", "home", "vacuum_on", "vacuum_off", "other"


def parse_gcode(text: str) -> list[GMove]:
    """Parse G-code text into a list of moves with classification."""
    moves = []
    pos = {"X": 0.0, "Y": 0.0, "Z": 0.0, "E": 0.0, "F": 900.0}
    axis_re = re.compile(r"([XYZEF])(-?\d+\.?\d*)")

    for i, raw_line in enumerate(text.strip().split("\n")):
        line = raw_line.split(";")[0].strip()  # strip comments
        if not line:
            continue

        # Track vacuum commands
        if line.startswith("M42"):
            mtype = "vacuum_on" if "S0" in line else "vacuum_off"
            moves.append(GMove(
                pos["X"], pos["Y"], pos["Z"], pos["E"],
                pos["X"], pos["Y"], pos["Z"], pos["E"],
                pos["F"], line, i, mtype
            ))
            continue

        # Home
        if line.startswith("G28"):
            old = dict(pos)
            pos["X"] = pos["Y"] = pos["Z"] = pos["E"] = 0.0
            moves.append(GMove(
                old["X"], old["Y"], old["Z"], old["E"],
                0, 0, 0, 0, pos["F"], line, i, "home"
            ))
            continue

        # G0/G1 moves
        if not line.startswith(("G0 ", "G1 ", "G0\t", "G1\t")):
            continue

        old = dict(pos)
        for m in axis_re.finditer(line):
            pos[m.group(1)] = float(m.group(2))

        # Classify the move
        xy_changed = abs(pos["X"] - old["X"]) > 0.001 or abs(pos["Y"] - old["Y"]) > 0.001
        z_changed = abs(pos["Z"] - old["Z"]) > 0.001
        e_changed = abs(pos["E"] - old["E"]) > 0.001

        if xy_changed and e_changed and not z_changed:
            mtype = "paste"  # XY + extrusion = paste dispensing
        elif z_changed and not xy_changed:
            mtype = "z_down" if pos["Z"] > old["Z"] else "z_up"
        elif xy_changed and not z_changed:
            mtype = "travel"
        elif xy_changed and z_changed:
            mtype = "diagonal"  # shouldn't happen in NanoPnP!
        else:
            mtype = "other"

        moves.append(GMove(
            old["X"], old["Y"], old["Z"], old["E"],
            pos["X"], pos["Y"], pos["Z"], pos["E"],
            pos["F"], line, i, mtype
        ))

    return moves


# ── Color map ─────────────────────────────────────────────────

COLORS = {
    "travel":     "#3b82f6",  # blue
    "z_down":     "#ef4444",  # red
    "z_up":       "#22c55e",  # green
    "paste":      "#f97316",  # orange
    "home":       "#9ca3af",  # gray
    "vacuum_on":  "#22c55e",  # green dot
    "vacuum_off": "#ef4444",  # red dot
    "diagonal":   "#ec4899",  # pink (shouldn't happen)
    "other":      "#6b7280",  # dim gray
}

LABELS = {
    "travel":     "XY Travel",
    "z_down":     "Z Descent",
    "z_up":       "Z Retract",
    "paste":      "Paste Extrusion",
    "home":       "Home",
    "vacuum_on":  "Vacuum ON",
    "vacuum_off": "Vacuum OFF",
    "diagonal":   "DIAGONAL (bug!)",
}


# ── Static plot ───────────────────────────────────────────────

def plot_gcode(moves: list[GMove], title: str = "G-code Toolpath",
               show_legend: bool = True, ax: plt.Axes | None = None) -> plt.Figure:
    """Render the toolpath as 2D XY view + Z profile + speed profile."""
    if ax is None:
        fig, (ax, ax_z, ax_f) = plt.subplots(3, 1, figsize=(10, 12),
                                              gridspec_kw={"height_ratios": [3, 1, 1]})
    else:
        fig = ax.figure
        ax_z = None
        ax_f = None

    ax.set_facecolor("#f8fafc")
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_title(title)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3, linestyle="--")

    # Draw machine bounds (faded)
    ax.add_patch(patches.Rectangle((0, 0), 130, 170, linewidth=1,
                                    edgecolor="#d1d5db", facecolor="none", linestyle="--"))

    # Collect lines by type for legend
    drawn_types = set()

    for move in moves:
        color = COLORS.get(move.move_type, COLORS["other"])

        if move.move_type in ("vacuum_on", "vacuum_off"):
            ax.plot(move.x1, move.y1, "o", color=color, markersize=6, zorder=5)
            drawn_types.add(move.move_type)
            continue

        if move.move_type == "other":
            continue

        # Draw line from old to new position
        dx = move.x1 - move.x0
        dy = move.y1 - move.y0
        if abs(dx) < 0.001 and abs(dy) < 0.001:
            # Z-only move — draw a small marker at the position
            if move.move_type in ("z_down", "z_up"):
                marker = "v" if move.move_type == "z_down" else "^"
                ax.plot(move.x1, move.y1, marker, color=color, markersize=5, zorder=4)
                drawn_types.add(move.move_type)
            continue

        linewidth = 2.5 if move.move_type == "paste" else 1.2
        alpha = 1.0 if move.move_type == "paste" else 0.7
        ax.plot([move.x0, move.x1], [move.y0, move.y1],
                color=color, linewidth=linewidth, alpha=alpha, zorder=3)
        drawn_types.add(move.move_type)

    # Mark special positions
    for move in moves:
        if move.move_type == "home":
            ax.plot(0, 0, "s", color=COLORS["home"], markersize=8, zorder=6)
            break

    # Start and end markers
    if moves:
        first_xy = next((m for m in moves if m.move_type == "travel"), None)
        if first_xy:
            ax.plot(first_xy.x1, first_xy.y1, "D", color="#2563eb", markersize=7,
                    zorder=7, label="_start")

    # Legend
    if show_legend:
        legend_handles = []
        for mtype in ["travel", "paste", "z_down", "z_up", "vacuum_on", "vacuum_off", "home", "diagonal"]:
            if mtype in drawn_types:
                if mtype in ("vacuum_on", "vacuum_off"):
                    h = plt.Line2D([0], [0], marker="o", color=COLORS[mtype],
                                   linestyle="None", markersize=6, label=LABELS[mtype])
                elif mtype in ("z_down", "z_up"):
                    marker = "v" if mtype == "z_down" else "^"
                    h = plt.Line2D([0], [0], marker=marker, color=COLORS[mtype],
                                   linestyle="None", markersize=6, label=LABELS[mtype])
                else:
                    h = plt.Line2D([0], [0], color=COLORS[mtype], linewidth=2, label=LABELS[mtype])
                legend_handles.append(h)
        if legend_handles:
            ax.legend(handles=legend_handles, loc="upper right", fontsize=8, framealpha=0.9)

    ax.set_xlim(-5, 140)
    ax.set_ylim(-5, 180)

    # ── Z profile (height over steps) ──
    if ax_z is not None:
        steps = list(range(len(moves)))
        z_vals = [m.z1 for m in moves]
        z_colors = [COLORS.get(m.move_type, COLORS["other"]) for m in moves]

        for i in range(len(moves)):
            if i > 0:
                ax_z.plot([steps[i - 1], steps[i]], [z_vals[i - 1], z_vals[i]],
                          color=z_colors[i], linewidth=1.5, alpha=0.8)
            ax_z.plot(steps[i], z_vals[i], "o", color=z_colors[i], markersize=3, zorder=4)

        # Mark vacuum events
        for i, m in enumerate(moves):
            if m.move_type == "vacuum_on":
                ax_z.axvline(i, color=COLORS["vacuum_on"], alpha=0.4, linewidth=1, linestyle="--")
                ax_z.text(i, max(z_vals) * 0.9, "V", fontsize=7, color=COLORS["vacuum_on"], ha="center")
            elif m.move_type == "vacuum_off":
                ax_z.axvline(i, color=COLORS["vacuum_off"], alpha=0.4, linewidth=1, linestyle="--")

        ax_z.set_facecolor("#f8fafc")
        ax_z.set_xlabel("Step")
        ax_z.set_ylabel("Z (mm)")
        ax_z.set_title("Z Height Profile", fontsize=10)
        ax_z.grid(True, alpha=0.3, linestyle="--")
        ax_z.invert_yaxis()  # 0=top (safe), higher=lower (physical convention)

    # ── Speed profile (feedrate over steps) ──
    if ax_f is not None:
        f_vals = [m.feedrate for m in moves]
        f_colors = [COLORS.get(m.move_type, COLORS["other"]) for m in moves]

        ax_f.bar(range(len(moves)), f_vals, color=f_colors, width=0.8, alpha=0.7)
        ax_f.set_facecolor("#f8fafc")
        ax_f.set_xlabel("Step")
        ax_f.set_ylabel("Feedrate (mm/min)")
        ax_f.set_title("Speed Profile", fontsize=10)
        ax_f.grid(True, alpha=0.3, linestyle="--", axis="y")

    return fig


# ── Animated playback ─────────────────────────────────────────

def animate_gcode(moves: list[GMove], title: str = "G-code Playback",
                  interval_ms: int = 200) -> None:
    """Animate the toolpath step by step."""
    from matplotlib.animation import FuncAnimation

    fig, ax = plt.subplots(1, 1, figsize=(10, 8))
    ax.set_facecolor("#f8fafc")
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_title(title)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3, linestyle="--")
    ax.add_patch(patches.Rectangle((0, 0), 130, 170, linewidth=1,
                                    edgecolor="#d1d5db", facecolor="none", linestyle="--"))
    ax.set_xlim(-5, 140)
    ax.set_ylim(-5, 180)

    # Nozzle marker
    nozzle, = ax.plot(0, 0, "o", color="#dc2626", markersize=10, zorder=10)
    status_text = ax.text(0.02, 0.98, "", transform=ax.transAxes, fontsize=9,
                          verticalalignment="top", fontfamily="monospace",
                          bbox=dict(boxstyle="round", facecolor="white", alpha=0.9))

    drawn_lines = []

    def update(frame):
        if frame >= len(moves):
            return [nozzle, status_text] + drawn_lines

        move = moves[frame]
        color = COLORS.get(move.move_type, COLORS["other"])

        # Draw the move
        if move.move_type not in ("vacuum_on", "vacuum_off", "other"):
            dx = move.x1 - move.x0
            dy = move.y1 - move.y0
            if abs(dx) > 0.001 or abs(dy) > 0.001:
                lw = 2.5 if move.move_type == "paste" else 1.2
                line, = ax.plot([move.x0, move.x1], [move.y0, move.y1],
                               color=color, linewidth=lw, alpha=0.8)
                drawn_lines.append(line)

        if move.move_type in ("vacuum_on", "vacuum_off"):
            dot, = ax.plot(move.x1, move.y1, "o", color=color, markersize=8, zorder=5)
            drawn_lines.append(dot)

        # Update nozzle position
        nozzle.set_data([move.x1], [move.y1])

        # Status text
        status_text.set_text(
            f"Step {frame + 1}/{len(moves)}\n"
            f"X={move.x1:.1f} Y={move.y1:.1f} Z={move.z1:.1f} E={move.e1:.1f}\n"
            f"{move.cmd}\n"
            f"Type: {move.move_type}"
        )

        return [nozzle, status_text] + drawn_lines

    anim = FuncAnimation(fig, update, frames=len(moves),
                         interval=interval_ms, blit=False, repeat=False)
    plt.tight_layout()
    plt.show()


# ── Summary stats ─────────────────────────────────────────────

def summarize(moves: list[GMove]) -> dict:
    """Compute summary statistics for a G-code toolpath."""
    total_travel = 0.0
    total_paste = 0.0
    total_z = 0.0
    move_counts = {}
    has_diagonal = False

    for m in moves:
        move_counts[m.move_type] = move_counts.get(m.move_type, 0) + 1
        dx = m.x1 - m.x0
        dy = m.y1 - m.y0
        dz = m.z1 - m.z0
        dist_xy = (dx**2 + dy**2) ** 0.5
        dist_z = abs(dz)

        if m.move_type == "travel":
            total_travel += dist_xy
        elif m.move_type == "paste":
            total_paste += dist_xy
        elif m.move_type in ("z_down", "z_up"):
            total_z += dist_z
        elif m.move_type == "diagonal":
            has_diagonal = True

    x_coords = [m.x1 for m in moves]
    y_coords = [m.y1 for m in moves]

    return {
        "total_moves": len(moves),
        "move_counts": move_counts,
        "travel_mm": round(total_travel, 1),
        "paste_mm": round(total_paste, 1),
        "z_travel_mm": round(total_z, 1),
        "x_range": (round(min(x_coords), 1), round(max(x_coords), 1)) if x_coords else (0, 0),
        "y_range": (round(min(y_coords), 1), round(max(y_coords), 1)) if y_coords else (0, 0),
        "has_diagonal": has_diagonal,
    }


# ── 3D Scene ──────────────────────────────────────────────────

def _draw_3d_scene(ax, config=None):
    """Draw static machine elements: base, board, feeders, camera, placements."""
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    # Machine base plate outline
    bx, by = 130, 170
    for x0, y0, x1, y1 in [(0, 0, bx, 0), (bx, 0, bx, by), (bx, by, 0, by), (0, by, 0, 0)]:
        ax.plot([x0, x1], [y0, y1], [0, 0], color="#d1d5db", linewidth=0.8, alpha=0.5)

    # Z reference planes (dashed lines at key heights)
    for z, label, color in [(0, "Safe Z=0", "#9ca3af"), (16, "Board Z=16", "#86efac"), (23, "Pick Z=23", "#fca5a5")]:
        for x0, y0, x1, y1 in [(0, 0, bx, 0), (0, by, bx, by)]:
            ax.plot([x0, x1], [y0, y1], [z, z], color=color, linewidth=0.5, alpha=0.3, linestyle="--")
        ax.text(bx + 2, 0, z, label, fontsize=6, color=color, alpha=0.7)

    if not config:
        return

    # Board surface (semi-transparent green rectangle at Z=board_surface)
    bz = config.z_heights.board_surface
    board_verts = [[(0, 0, bz), (bx, 0, bz), (bx, by, bz), (0, by, bz)]]
    board_poly = Poly3DCollection(board_verts, alpha=0.08, facecolor="#86efac", edgecolor="#22c55e", linewidth=0.5)
    ax.add_collection3d(board_poly)

    # Feeders (small boxes with IC slot tick marks)
    for slot, f in config.feeders.items():
        if not f.enabled:
            continue
        fx, fy, fz = f.ref_hole.x, f.ref_hole.y, f.ref_hole.z
        # Feeder box
        tw = f.tape_width / 2
        ax.bar3d(fx - tw, fy - 2, fz - 1, tw * 2, 4, 1, color="#f97316", alpha=0.3, edgecolor="#ea580c")
        ax.text(fx, fy - 4, fz, slot, fontsize=7, color="#ea580c", ha="center")

        # IC slots (tick marks along tape direction)
        dx = f.last_hole.x - f.ref_hole.x
        dy = f.last_hole.y - f.ref_hole.y
        length = max(0.01, (dx**2 + dy**2)**0.5)
        ux, uy = dx / length, dy / length
        for i in range(10):  # show 10 IC positions
            sx = fx + i * f.pitch * ux
            sy = fy + i * f.pitch * uy
            ax.plot([sx - 1, sx + 1], [sy, sy], [fz, fz], color="#ea580c", linewidth=1, alpha=0.5)

    # Camera position
    cam = config.camera.position
    ax.scatter([cam.x], [cam.y], [0], c="#8b5cf6", s=60, marker="^", zorder=5, label="Camera")
    ax.text(cam.x, cam.y + 3, 0, "CAM", fontsize=6, color="#8b5cf6", ha="center")

    # Placement positions on board
    bz = config.z_heights.board_surface
    for p in config.placements:
        color = "#22c55e" if p.enabled else "#9ca3af"
        marker = "s" if p.enabled else "x"
        ax.scatter([p.x], [p.y], [bz], c=color, s=40, marker=marker, zorder=5)
        ax.text(p.x + 2, p.y, bz, p.ref, fontsize=6, color=color)


def _expand_frames(moves: list[GMove], base_ms: int = 50, speed_scale: float = 1.0) -> list[dict]:
    """Expand moves into interpolated frames for variable-speed animation.

    Returns list of {x, y, z, e, move_idx, t, move_type, color, cmd}.
    """
    frames = []
    for i, m in enumerate(moves):
        dx = m.x1 - m.x0
        dy = m.y1 - m.y0
        dz = m.z1 - m.z0
        dist = max(0.01, (dx**2 + dy**2 + dz**2)**0.5)
        time_ms = (dist / max(1, m.feedrate)) * 60000
        n_frames = max(1, int(time_ms / base_ms * speed_scale))

        for j in range(n_frames):
            t = (j + 1) / n_frames
            frames.append({
                "x": m.x0 + dx * t,
                "y": m.y0 + dy * t,
                "z": m.z0 + dz * t,
                "e": m.e0 + (m.e1 - m.e0) * t,
                "move_idx": i,
                "t": t,
                "move_type": m.move_type,
                "color": COLORS.get(m.move_type, COLORS["other"]),
                "cmd": m.cmd,
                "feedrate": m.feedrate,
            })
    return frames


def plot_gcode_3d(moves: list[GMove], config=None, title: str = "3D Toolpath",
                  save_path: str | None = None) -> plt.Figure:
    """Static 3D plot of the full toolpath with machine elements."""
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection="3d")

    _draw_3d_scene(ax, config)

    # Draw all moves
    for m in moves:
        color = COLORS.get(m.move_type, COLORS["other"])
        if m.move_type in ("vacuum_on", "vacuum_off"):
            ax.scatter([m.x1], [m.y1], [m.z1], c=color, s=50, zorder=5)
            continue
        if m.move_type == "other":
            continue

        lw = 2.5 if m.move_type == "paste" else 1.0
        alpha = 0.9 if m.move_type == "paste" else 0.6
        ax.plot([m.x0, m.x1], [m.y0, m.y1], [m.z0, m.z1],
                color=color, linewidth=lw, alpha=alpha)

    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")
    ax.set_title(title)
    ax.set_xlim(-5, 140)
    ax.set_ylim(-5, 180)
    ax.set_zlim(-2, 28)
    ax.invert_zaxis()  # 0=top, 23=bottom (physical convention)
    ax.view_init(elev=25, azim=-50)

    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")

    return fig


def animate_gcode_3d(moves: list[GMove], config=None, title: str = "3D Toolpath",
                     speed_scale: float = 0.3) -> None:
    """Animated 3D visualization with speed proportional to feedrate."""
    from matplotlib.animation import FuncAnimation

    frames_data = _expand_frames(moves, base_ms=50, speed_scale=speed_scale)

    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection="3d")
    _draw_3d_scene(ax, config)

    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")
    ax.set_title(title)
    ax.set_xlim(-5, 140)
    ax.set_ylim(-5, 180)
    ax.set_zlim(-2, 28)
    ax.invert_zaxis()

    # Nozzle marker
    nozzle = ax.scatter([0], [0], [0], c="#dc2626", s=80, zorder=10, depthshade=False)

    # Status text (2D overlay)
    status = fig.text(0.02, 0.95, "", fontsize=9, fontfamily="monospace",
                      verticalalignment="top",
                      bbox=dict(boxstyle="round", facecolor="white", alpha=0.9))

    trail_lines = []
    prev_frame = None
    base_azim = -50

    def update(frame_idx):
        nonlocal prev_frame, base_azim
        if frame_idx >= len(frames_data):
            return [nozzle, status]

        fd = frames_data[frame_idx]

        # Draw line from previous frame
        if prev_frame and prev_frame["move_idx"] == fd["move_idx"]:
            line, = ax.plot(
                [prev_frame["x"], fd["x"]],
                [prev_frame["y"], fd["y"]],
                [prev_frame["z"], fd["z"]],
                color=fd["color"],
                linewidth=2.5 if fd["move_type"] == "paste" else 1.0,
                alpha=0.8,
            )
            trail_lines.append(line)
        elif prev_frame:
            # New move — draw connection line
            line, = ax.plot(
                [prev_frame["x"], fd["x"]],
                [prev_frame["y"], fd["y"]],
                [prev_frame["z"], fd["z"]],
                color=fd["color"],
                linewidth=2.5 if fd["move_type"] == "paste" else 1.0,
                alpha=0.8,
            )
            trail_lines.append(line)

        # Vacuum event markers
        if fd["move_type"] in ("vacuum_on", "vacuum_off"):
            ax.scatter([fd["x"]], [fd["y"]], [fd["z"]], c=fd["color"], s=60, zorder=6)

        # Update nozzle position
        nozzle._offsets3d = ([fd["x"]], [fd["y"]], [fd["z"]])

        # Slow camera rotation
        base_azim += 0.15
        ax.view_init(elev=25, azim=base_azim)

        # Status
        status.set_text(
            f"Frame {frame_idx + 1}/{len(frames_data)}  "
            f"Move {fd['move_idx'] + 1}/{len(moves)}\n"
            f"X={fd['x']:.1f}  Y={fd['y']:.1f}  Z={fd['z']:.1f}  "
            f"F={fd['feedrate']:.0f}\n"
            f"{fd['cmd']}"
        )

        prev_frame = fd
        return [nozzle, status] + trail_lines

    anim = FuncAnimation(fig, update, frames=len(frames_data),
                         interval=50, blit=False, repeat=False)
    plt.tight_layout()
    plt.show()


# ── CLI ───────────────────────────────────────────────────────

def main():
    import argparse
    parser = argparse.ArgumentParser(description="G-code toolpath visualizer")
    parser.add_argument("file", help="G-code file to visualize")
    parser.add_argument("--3d", dest="three_d", action="store_true", help="3D visualization")
    parser.add_argument("--animate", action="store_true", help="Animate step-by-step")
    parser.add_argument("--speed", type=float, default=0.3, help="Animation speed scale (0.1=fast, 1.0=realtime)")
    parser.add_argument("--save", help="Save plot to image file")
    parser.add_argument("--stats", action="store_true", help="Print summary statistics")
    parser.add_argument("--config", default=None, help="config.json path for machine geometry")
    args = parser.parse_args()

    text = open(args.file).read()
    moves = parse_gcode(text)

    # Load config for 3D scene elements
    config = None
    if args.config:
        from nanopnp.config import load_config
        config = load_config(args.config)

    if args.stats or not args.animate:
        stats = summarize(moves)
        print(f"G-code: {args.file}")
        print(f"  Total moves: {stats['total_moves']}")
        for mtype, count in sorted(stats["move_counts"].items()):
            print(f"    {mtype}: {count}")
        print(f"  XY travel: {stats['travel_mm']} mm")
        print(f"  Paste extrusion: {stats['paste_mm']} mm")
        print(f"  Z travel: {stats['z_travel_mm']} mm")
        print(f"  X range: {stats['x_range']}")
        print(f"  Y range: {stats['y_range']}")
        if stats["has_diagonal"]:
            print(f"  WARNING: diagonal Z+XY moves detected!")
        print()

    if args.three_d:
        if args.animate:
            animate_gcode_3d(moves, config=config, title=f"3D: {args.file}", speed_scale=args.speed)
        else:
            fig = plot_gcode_3d(moves, config=config, title=f"3D: {args.file}", save_path=args.save)
            if not args.save:
                plt.show()
    else:
        if args.animate:
            animate_gcode(moves, title=f"Toolpath: {args.file}", interval_ms=int(args.speed * 1000))
        else:
            fig = plot_gcode(moves, title=f"Toolpath: {args.file}")
            if args.save:
                fig.savefig(args.save, dpi=150, bbox_inches="tight")
                print(f"Saved to {args.save}")
            else:
                plt.tight_layout()
                plt.show()


if __name__ == "__main__":
    main()
