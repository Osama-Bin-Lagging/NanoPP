#!/usr/bin/env python3
"""NanoPnP — Pick-and-place machine controller.

Usage:
    python run.py [--config CONFIG] [--dry-run] <command>

Commands:
    home     Home all axes
    jog      Jog to X Y [Z] position
    paste    Run paste dispensing for enabled placements
    place    Run pick-and-place for enabled placements
    job      Full run: paste then place
"""

from __future__ import annotations

import argparse
import sys

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
    parser.add_argument("--output", "-o", default=None, help="Save G-code to file (e.g. output/job.gcode)")
    parser.add_argument("--gui", action="store_true", help="Launch graphical interface")

    sub = parser.add_subparsers(dest="command")
    sub.add_parser("home", help="Home all axes")

    jog = sub.add_parser("jog", help="Jog to a position")
    jog.add_argument("x", type=float)
    jog.add_argument("y", type=float)
    jog.add_argument("z", type=float, nargs="?", default=None)

    sub.add_parser("paste", help="Dispense paste on enabled placements")
    sub.add_parser("place", help="Pick-and-place enabled placements")
    sub.add_parser("job", help="Full job: paste + place")

    return parser


def _make_motion(config, dry_run: bool, gcode_file: str | None = None):
    """Create and connect a MotionController."""
    from nanopnp.serial_comm import SerialConnection
    from nanopnp.motion import MotionController

    serial = SerialConnection(config.machine.serial_port, config.machine.baud_rate,
                              dry_run=dry_run, gcode_file=gcode_file)
    serial.connect(disable_endstops=config.machine.disable_software_endstops)
    return MotionController(serial, config)


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


def cmd_paste(config, dry_run: bool, output: str | None = None) -> None:
    from nanopnp.paste_dispenser import PasteDispenser

    placements = config.enabled_placements()
    if not placements:
        print("No enabled placements")
        return

    motion = _make_motion(config, dry_run, gcode_file=output)
    try:
        dispenser = PasteDispenser(motion, config)
        count = dispenser.dispense_board(placements)
        print(f"Paste dispensed for {count}/{len(placements)} components")
    finally:
        motion.serial.close()


def cmd_place(config, dry_run: bool, output: str | None = None) -> None:
    from nanopnp.feeder import FeederManager
    from nanopnp.pnp_engine import PnPEngine

    placements = config.enabled_placements()
    if not placements:
        print("No enabled placements")
        return

    motion = _make_motion(config, dry_run, gcode_file=output)
    try:
        feeders = FeederManager(config)
        engine = PnPEngine(config, motion, feeders, vision=None)
        result = engine.run_job(placements)
        print(f"Placed {result.placed}/{result.total} in {result.duration_s}s ({result.cph:.0f} CPH)")
        for pr in result.placements:
            status = "OK" if pr.success else f"FAIL: {pr.error}"
            print(f"  {pr.ref}: {status} corrections=({pr.corrections[0]:.3f}, {pr.corrections[1]:.3f}, {pr.corrections[2]:.1f})")
    finally:
        motion.serial.close()


def cmd_job(config, dry_run: bool, output: str | None = None) -> None:
    from nanopnp.feeder import FeederManager
    from nanopnp.paste_dispenser import PasteDispenser
    from nanopnp.pnp_engine import PnPEngine

    placements = config.enabled_placements()
    if not placements:
        print("No enabled placements")
        return

    motion = _make_motion(config, dry_run, gcode_file=output)
    try:
        feeders = FeederManager(config)
        paste = PasteDispenser(motion, config)
        engine = PnPEngine(config, motion, feeders, vision=None)
        result = engine.run_job(placements, paste_dispenser=paste, paste_first=True)
        print(f"Job done: {result.placed}/{result.total} placed in {result.duration_s}s ({result.cph:.0f} CPH)")
    finally:
        motion.serial.close()


def main(argv: list[str] | None = None) -> int:
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

    dispatch = {
        "home":  lambda: cmd_home(config, args.dry_run, out),
        "jog":   lambda: cmd_jog(config, args.dry_run, args.x, args.y, args.z, out),
        "paste": lambda: cmd_paste(config, args.dry_run, out),
        "place": lambda: cmd_place(config, args.dry_run, out),
        "job":   lambda: cmd_job(config, args.dry_run, out),
    }

    handler = dispatch.get(args.command)
    if handler:
        handler()
    return 0


if __name__ == "__main__":
    sys.exit(main())
