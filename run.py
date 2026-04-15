#!/usr/bin/env python3
"""NanoPnP — Pick-and-place machine controller.

Usage:
    python run.py [--config CONFIG] [--dry-run] [--output OUT] <command> ...

Commands:
    home     Home all axes
    jog      Jog to X Y [Z] position
    paste    Dispense paste (requires --edge-cuts and --paste)
    place    Pick and place (requires --pos)
    job      Full run: paste then place (requires all three)
    gui      Launch graphical interface

Examples:
    python run.py paste --dry-run \\
        --edge-cuts board-Edge_Cuts.gbr --paste board-F_Paste.gbr -o paste.gcode

    python run.py place --dry-run --pos board-top.pos -o place.gcode

    python run.py job --dry-run \\
        --edge-cuts board-Edge_Cuts.gbr --paste board-F_Paste.gbr \\
        --pos board-top.pos -o full.gcode
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from nanopnp import __version__
from nanopnp.config import load_config, ConfigError


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="nanopnp",
        description="NanoPnP pick-and-place controller",
    )
    parser.add_argument("--version", action="version", version=f"%(prog)s {__version__}")
    parser.add_argument("--config", default="config.json", help="Path to config.json")
    parser.add_argument("--dry-run", action="store_true", help="Simulate without serial")
    parser.add_argument("--vision", action="store_true", help="Enable bottom-camera vision alignment")
    parser.add_argument("--output", "-o", default=None, help="Save G-code to file (e.g. output/job.gcode)")
    parser.add_argument("--gui", action="store_true", help="Launch graphical interface")

    sub = parser.add_subparsers(dest="command")
    sub.add_parser("home", help="Home all axes")

    jog = sub.add_parser("jog", help="Jog to a position")
    jog.add_argument("x", type=float)
    jog.add_argument("y", type=float)
    jog.add_argument("z", type=float, nargs="?", default=None)

    paste = sub.add_parser("paste", help="Dispense paste from Edge_Cuts + F_Paste gerbers")
    paste.add_argument("--edge-cuts", type=Path, required=True, help="Path to Edge_Cuts.gbr")
    paste.add_argument("--paste", dest="paste_file", type=Path, required=True, help="Path to F_Paste.gbr")

    place = sub.add_parser("place", help="Pick-and-place from a .pos file")
    place.add_argument("--pos", type=Path, required=True, help="Path to top.pos")
    place.add_argument("--edge-cuts", type=Path, default=None,
                       help="Optional Edge_Cuts.gbr — enables auto origin detection when paired with --paste")
    place.add_argument("--paste", dest="paste_file", type=Path, default=None,
                       help="Optional F_Paste.gbr — enables auto origin detection when paired with --edge-cuts")

    job = sub.add_parser("job", help="Full job: paste + place")
    job.add_argument("--edge-cuts", type=Path, required=True, help="Path to Edge_Cuts.gbr")
    job.add_argument("--paste", dest="paste_file", type=Path, required=True, help="Path to F_Paste.gbr")
    job.add_argument("--pos", type=Path, required=True, help="Path to top.pos")

    manual = sub.add_parser("manual", help="Manual joystick control via external controller")
    manual.add_argument("--controller-port", default=None,
                        help="Controller serial port (overrides config)")
    manual.add_argument("--controller-baud", type=int, default=None,
                        help="Controller baud rate (overrides config)")

    return parser


def _make_motion(config, dry_run: bool, gcode_file: str | None = None):
    """Create and connect a MotionController."""
    from nanopnp.serial_comm import SerialConnection
    from nanopnp.motion import MotionController

    serial = SerialConnection(config.machine.serial_port, config.machine.baud_rate,
                              dry_run=dry_run, gcode_file=gcode_file)
    serial.connect(disable_endstops=config.machine.disable_software_endstops)
    return MotionController(serial, config)


def _make_vision(config, enable: bool):
    """Create and open a VisionSystem if --vision was passed."""
    if not enable:
        return None
    try:
        from nanopnp.vision import VisionSystem
    except ImportError:
        print("Error: --vision requires opencv-python (pip install opencv-python)", file=sys.stderr)
        sys.exit(1)
    vs = VisionSystem(config)
    vs.open()
    return vs


def cmd_home(config, dry_run: bool, output: str | None = None) -> None:
    motion = _make_motion(config, dry_run, gcode_file=output)
    try:
        motion.home()
        pos = motion.get_position()
        print(f"Homed. Position: X={pos['X']:.2f} Y={pos['Y']:.2f} Z={pos['Z']:.2f} E={pos['E']:.2f}")
    finally:
        motion.serial.close()


def cmd_jog(config, dry_run: bool, x: float, y: float, z: float | None, output: str | None = None) -> None:
    motion = _make_motion(config, dry_run, gcode_file=output)
    try:
        motion.safe_move_to(x, y, z=z)
        pos = motion.get_position()
        print(f"Moved. Position: X={pos['X']:.2f} Y={pos['Y']:.2f} Z={pos['Z']:.2f} E={pos['E']:.2f}")
    finally:
        motion.serial.close()


def cmd_paste(config, dry_run: bool, edge_cuts: Path, paste_file: Path,
              output: str | None = None) -> None:
    from nanopnp.paste_dispenser import PasteDispenser

    for p in (edge_cuts, paste_file):
        if not p.exists():
            print(f"Error: file not found: {p}", file=sys.stderr)
            sys.exit(2)

    motion = _make_motion(config, dry_run, gcode_file=output)
    try:
        dispenser = PasteDispenser(motion, config)
        count = dispenser.dispense_paste(edge_cuts, paste_file)
        print(f"Paste dispensed for {count} components")
    finally:
        motion.serial.close()


def cmd_place(config, dry_run: bool, pos: Path,
              edge_cuts: Path | None = None, paste_file: Path | None = None,
              output: str | None = None, enable_vision: bool = False) -> None:
    from nanopnp.board_parser import load_placements_from_pos
    from nanopnp.feeder import FeederManager
    from nanopnp.pnp_engine import PnPEngine

    if not pos.exists():
        print(f"Error: .pos file not found: {pos}", file=sys.stderr)
        sys.exit(2)

    placements = load_placements_from_pos(
        pos,
        origin_xy=(config.board.origin.x, config.board.origin.y),
        edge_cuts_path=edge_cuts,
        paste_path=paste_file,
        package_map=config.board.package_map,
        skip_refs_prefixes=config.board.skip_refs_prefixes,
    )
    if not placements:
        print("No placements parsed from .pos file")
        return

    vision = _make_vision(config, enable_vision)
    motion = _make_motion(config, dry_run, gcode_file=output)
    try:
        feeders = FeederManager(config)
        engine = PnPEngine(config, motion, feeders, vision=vision,
                           vision_enabled=enable_vision)
        result = engine.run_job(placements)
        print(f"Placed {result.placed}/{result.total} in {result.duration_s}s ({result.cph:.0f} CPH)")
        for pr in result.placements:
            status = "OK" if pr.success else f"FAIL: {pr.error}"
            print(f"  {pr.ref}: {status} corrections=({pr.corrections[0]:.3f}, {pr.corrections[1]:.3f}, {pr.corrections[2]:.1f})")
    finally:
        if vision:
            vision.close()
        motion.serial.close()


def cmd_job(config, dry_run: bool, edge_cuts: Path, paste_file: Path, pos: Path,
            output: str | None = None, enable_vision: bool = False) -> None:
    from nanopnp.board_parser import load_placements_from_pos
    from nanopnp.feeder import FeederManager
    from nanopnp.paste_dispenser import PasteDispenser
    from nanopnp.pnp_engine import PnPEngine

    for p in (edge_cuts, paste_file, pos):
        if not p.exists():
            print(f"Error: file not found: {p}", file=sys.stderr)
            sys.exit(2)

    placements = load_placements_from_pos(
        pos,
        origin_xy=(config.board.origin.x, config.board.origin.y),
        edge_cuts_path=edge_cuts,
        paste_path=paste_file,
        package_map=config.board.package_map,
        skip_refs_prefixes=config.board.skip_refs_prefixes,
    )
    if not placements:
        print("No placements parsed from .pos file")
        return

    # Store loaded placements into config so the paste dispenser can see them
    config.placements.clear()
    config.placements.extend(placements)

    vision = _make_vision(config, enable_vision)
    motion = _make_motion(config, dry_run, gcode_file=output)
    try:
        dispenser = PasteDispenser(motion, config)
        dispenser.dispense_paste(edge_cuts, paste_file)

        feeders = FeederManager(config)
        engine = PnPEngine(config, motion, feeders, vision=vision,
                           vision_enabled=enable_vision)
        result = engine.run_job(placements)
        print(f"Job done: {result.placed}/{result.total} placed in {result.duration_s}s ({result.cph:.0f} CPH)")
    finally:
        if vision:
            vision.close()
        motion.serial.close()


def cmd_manual(config, controller_port: str | None, controller_baud: int | None) -> None:
    from nanopnp.manual_control import ControllerConfig as MCControllerConfig, run as manual_run

    # Build controller config: CLI args override config.json
    cfg = MCControllerConfig(
        port=controller_port or config.controller.port,
        baud=controller_baud or config.controller.baud,
        dead_low=config.controller.dead_low,
        dead_high=config.controller.dead_high,
        joy_max=config.controller.joy_max,
        max_step_xy=config.controller.max_step_xy,
        max_step_yaw=config.controller.max_step_yaw,
        fine_xy=config.controller.fine_xy,
        fine_z=config.controller.fine_z,
        fine_yaw=config.controller.fine_yaw,
        feed_rate=config.controller.feed_rate,
        feed_rate_slow=config.controller.feed_rate_slow,
    )

    if not cfg.port:
        print("Error: no controller port. Set controller.port in config.json or pass --controller-port",
              file=sys.stderr)
        sys.exit(1)

    manual_run(cfg, config.machine.serial_port, config.machine.baud_rate)


def main(argv: list[str] | None = None) -> int:
    # Surface INFO-level log messages so the watcher can tail progress
    # (e.g. "[paste] (3/9) IC1") from stdout/stderr. Without this,
    # Python's root logger defaults to WARNING and info lines vanish.
    import logging
    import signal
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    signal.signal(signal.SIGTERM, lambda sig, frame: sys.exit(0))

    parser = build_parser()
    args = parser.parse_args(argv)

    try:
        config = load_config(args.config)
    except (FileNotFoundError, ConfigError) as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1

    if args.gui:
        from nanopnp.gui import launch_gui
        launch_gui(config, config_path=args.config)
        return 0

    if args.command is None:
        parser.print_help()
        return 0

    out = args.output
    if out:
        import os
        os.makedirs(os.path.dirname(out) or ".", exist_ok=True)

    if args.command == "home":
        cmd_home(config, args.dry_run, out)
    elif args.command == "jog":
        cmd_jog(config, args.dry_run, args.x, args.y, args.z, out)
    elif args.command == "paste":
        cmd_paste(config, args.dry_run, args.edge_cuts, args.paste_file, out)
    elif args.command == "place":
        cmd_place(config, args.dry_run, args.pos,
                  edge_cuts=args.edge_cuts, paste_file=args.paste_file,
                  output=out, enable_vision=args.vision)
    elif args.command == "job":
        cmd_job(config, args.dry_run, args.edge_cuts, args.paste_file, args.pos,
                output=out, enable_vision=args.vision)
    elif args.command == "manual":
        cmd_manual(config, args.controller_port, args.controller_baud)
    return 0


if __name__ == "__main__":
    sys.exit(main())
