"""Manual joystick control via external controller (ESP32 / Arduino).

Reads button + joystick state from a controller on a dedicated serial port.
When the MANUAL switch is active, opens the machine port directly in G91
(relative) mode and streams G1 commands from joystick/button input.

Protocol from controller:
    B:<hex_buttons> X1:<hex_joy_x> Y1:<hex_joy_y> X2:<hex_joy_yaw>

Button bits (LSB-first):
    0=UP 1=DOWN 2=LEFT 3=RIGHT 4=FOR 5=BACK
    6=YAW_L 7=YAW_R 8=SEL1 9=SEL2 10=MANUAL
"""

from __future__ import annotations

import logging
import os
import re
import signal
import subprocess
import time
from dataclasses import dataclass

logger = logging.getLogger(__name__)

# Button names, ordered by bit position in the protocol byte
BTN_NAMES = [
    "UP", "DOWN", "LEFT", "RIGHT", "FOR", "BACK",
    "YAW_L", "YAW_R", "SEL1", "SEL2", "MANUAL",
]

_FRAME_RE = re.compile(r"B:(\w+) X1:(\w+) Y1:(\w+) X2:(\w+)")


@dataclass(slots=True)
class ControllerConfig:
    """Settings for the external joystick controller."""
    port: str                    # controller serial port
    baud: int = 115200           # controller baud rate
    dead_low: int = 1900         # joystick deadzone lower bound
    dead_high: int = 2150        # joystick deadzone upper bound
    joy_max: int = 4095          # joystick ADC max
    max_step_xy: float = 2.0     # max joystick XY step per frame (mm)
    max_step_yaw: float = 1.0    # max joystick yaw step per frame (deg)
    fine_xy: float = 0.2         # button fine XY step (mm)
    fine_z: float = 2.0          # button fine Z step (mm)
    fine_yaw: float = 0.5        # button fine yaw step (deg)
    feed_rate: int = 1000        # G1 feedrate for XY moves
    feed_rate_slow: int = 200    # G1 feedrate for fine/Z moves


def parse_buttons(byte_val: int) -> dict[str, bool]:
    """Decode button bitfield into a name→bool dict."""
    return {BTN_NAMES[i]: bool((byte_val >> i) & 1) for i in range(len(BTN_NAMES))}


def joy_to_step(raw: int, max_step: float, cfg: ControllerConfig) -> float:
    """Map a joystick ADC value to a step size, respecting deadzone."""
    if raw < cfg.dead_low:
        factor = (cfg.dead_low - raw) / cfg.dead_low
        return -factor * max_step
    elif raw > cfg.dead_high:
        factor = (raw - cfg.dead_high) / (cfg.joy_max - cfg.dead_high)
        return factor * max_step
    return 0.0


def build_gcode(dx: float, dy: float, dz: float, dyaw: float,
                t_state: int, slow: bool, cfg: ControllerConfig) -> str:
    """Build a G1 command from axis deltas. Returns empty string if no motion."""
    # Invert axes to match machine coordinate convention
    dx *= -1
    dy *= -1
    dz *= -1
    dyaw *= -1

    parts = []
    if abs(dx)   > 1e-6: parts.append(f"X{dx:.3f}")
    if abs(dy)   > 1e-6: parts.append(f"Y{dy:.3f}")
    if abs(dz)   > 1e-6: parts.append(f"Z{dz:.3f}")
    if abs(dyaw) > 1e-6: parts.append(f"E{dyaw:.3f}")

    if not parts:
        return ""

    # Pick feedrate: slow for Z-only or fine buttons, fast yaw when paste tool active
    if slow:
        feed = cfg.feed_rate_slow
    elif all(p.startswith("Z") for p in parts):
        feed = cfg.feed_rate_slow
    elif all(p.startswith("E") for p in parts) and t_state:
        feed = 7000
    else:
        feed = cfg.feed_rate

    return f"G1 {' '.join(parts)} F{feed}"


def _get_nanopnp_pids() -> list[int]:
    """Find running NanoPnP processes (excluding this one)."""
    my_pid = os.getpid()
    try:
        out = subprocess.check_output(
            ["pgrep", "-f", r"python.*run\.py.*(gui|home|jog|paste|place|job)"],
            universal_newlines=True,
        )
        return [int(p) for p in out.strip().splitlines() if int(p) != my_pid]
    except subprocess.CalledProcessError:
        return []


def _wait_for_pids_exit(pids: list[int], timeout: float = 5.0) -> None:
    """Poll until all PIDs have exited, or timeout."""
    deadline = time.monotonic() + timeout
    remaining = list(pids)
    while remaining and time.monotonic() < deadline:
        still_alive = []
        for pid in remaining:
            try:
                os.kill(pid, 0)  # check if alive
                still_alive.append(pid)
            except ProcessLookupError:
                pass
        remaining = still_alive
        if remaining:
            time.sleep(0.1)
    if remaining:
        logger.warning(f"PIDs still alive after {timeout}s: {remaining}")


def kill_nanopnp() -> list[int]:
    """Send SIGTERM to any running NanoPnP processes. Returns signaled PIDs."""
    pids = _get_nanopnp_pids()
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
            logger.info(f"Sent SIGTERM to NanoPnP PID {pid}")
        except ProcessLookupError:
            pass
    return pids


def _send(mach, line: str) -> None:
    """Send a G-code line to the machine (fire-and-forget)."""
    logger.debug(f">> {line}")
    if mach is not None:
        try:
            mach.write((line + "\n").encode())
        except Exception as e:
            logger.warning(f"Machine write error: {e}")


def _try_open_machine(port: str, baud: int):
    """Try to open the machine serial port. Returns serial object or None."""
    import serial
    try:
        s = serial.Serial(port, baud, timeout=1)
        logger.info(f"Connected to machine on {port} @ {baud}")
        return s
    except Exception as e:
        logger.warning(f"Could not open {port}: {e}")
        return None


def run(controller_cfg: ControllerConfig, machine_port: str, machine_baud: int) -> None:
    """Main manual control loop. Blocks until Ctrl-C.

    Reads frames from the controller. When MANUAL switch goes high,
    kills any running NanoPnP process, opens the machine port, and
    enters relative-mode jogging. When MANUAL goes low, releases
    the machine port.
    """
    import serial

    cfg = controller_cfg
    logger.info(f"Opening controller on {cfg.port} @ {cfg.baud}")

    with serial.Serial(cfg.port, cfg.baud, timeout=1) as ser:
        logger.info("Controller connected, waiting for MANUAL switch...")

        t_state = 0       # tool: 0=nozzle, 1=paste
        v_state = 0       # vacuum: 0=off, 1=on
        prev_sel1 = False
        prev_sel2 = False
        prev_manual = False
        mach = None

        try:
            while True:
                raw = ser.readline().decode(errors="ignore").strip()
                if not raw:
                    continue

                m = _FRAME_RE.match(raw)
                if not m:
                    continue

                buttons_byte = int(m.group(1), 16)
                x1 = int(m.group(2), 16)
                y1 = int(m.group(3), 16)
                x2 = int(m.group(4), 16)

                btn = parse_buttons(buttons_byte)
                manual = btn["MANUAL"]

                # MANUAL rising edge: kill NanoPnP, open machine port
                if manual and not prev_manual:
                    killed = kill_nanopnp()
                    if killed:
                        _wait_for_pids_exit(killed)
                    # Retry opening machine port (OS may need time to release it)
                    for attempt in range(5):
                        mach = _try_open_machine(machine_port, machine_baud)
                        if mach:
                            _send(mach, "G91")
                            break
                        time.sleep(0.5)
                    else:
                        logger.error("Failed to open machine port after 5 attempts")

                # MANUAL falling edge: close machine port
                if not manual and prev_manual:
                    if mach is not None:
                        mach.close()
                        mach = None
                        logger.info(f"Disconnected from {machine_port}")

                prev_manual = manual

                if not manual:
                    continue

                # SEL1: toggle tool on rising edge
                if btn["SEL1"] and not prev_sel1:
                    t_state = 1 - t_state
                    _send(mach, f"T{t_state}")
                prev_sel1 = btn["SEL1"]

                # SEL2: toggle vacuum on rising edge
                if btn["SEL2"] and not prev_sel2:
                    v_state = 1 - v_state
                    _send(mach, "M42 P4 S0" if v_state else "M42 P4 S255")
                prev_sel2 = btn["SEL2"]

                # Joystick coarse control
                dx   =  joy_to_step(x1, cfg.max_step_xy, cfg)
                dy   = -joy_to_step(y1, cfg.max_step_xy, cfg)
                dyaw =  joy_to_step(x2, cfg.max_step_yaw * (1 if t_state == 0 else 10), cfg)
                dz   = 0.0

                # Button fine control
                if btn["LEFT"]:   dx   -= cfg.fine_xy
                if btn["RIGHT"]:  dx   += cfg.fine_xy
                if btn["FOR"]:    dy   += cfg.fine_xy
                if btn["BACK"]:   dy   -= cfg.fine_xy
                if btn["UP"]:     dz   += cfg.fine_z
                if btn["DOWN"]:   dz   -= cfg.fine_z
                if btn["YAW_L"]:  dyaw += cfg.fine_yaw
                if btn["YAW_R"]:  dyaw -= cfg.fine_yaw

                slow = btn["FOR"] or btn["BACK"] or btn["LEFT"] or btn["RIGHT"]

                line = build_gcode(dx, dy, dz, dyaw, t_state, slow, cfg)
                if line:
                    _send(mach, line)

        except KeyboardInterrupt:
            logger.info("Manual control stopped")
        finally:
            if mach is not None:
                mach.close()


if __name__ == "__main__":
    import argparse
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    parser = argparse.ArgumentParser(description="NanoPnP manual joystick control")
    parser.add_argument("--controller-port", required=True, help="Controller serial port")
    parser.add_argument("--controller-baud", type=int, default=115200)
    parser.add_argument("--machine-port", required=True, help="Machine serial port")
    parser.add_argument("--machine-baud", type=int, default=250000)
    args = parser.parse_args()

    cfg = ControllerConfig(port=args.controller_port, baud=args.controller_baud)
    run(cfg, args.machine_port, args.machine_baud)
