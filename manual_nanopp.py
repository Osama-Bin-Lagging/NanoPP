import serial
import re
import subprocess
import time
import os
import signal

# --- Config ---
PORT      = "/dev/tty.usbmodem14301"
BAUD      = 115200
MACH_PORT = "/dev/tty.usbserial-120"
MACH_BAUD = 250000

BTN_NAMES = ["UP", "DOWN", "LEFT", "RIGHT", "FOR", "BACK", "YAW_L", "YAW_R", "SEL1", "SEL2", "MANUAL"]

DEAD_LOW  = 1900
DEAD_HIGH = 2150
JOY_MAX   = 4095

MAX_STEP_XY  = 2.0
MAX_STEP_YAW = 1.0

FINE_XY  = 0.2
FINE_Z   = 2.0
FINE_YAW = 0.5

FEED_RATE = 1000
FEED_RATE_SLOW = 200


def parse_buttons(byte_val):
    return {BTN_NAMES[i]: bool((byte_val >> i) & 1) for i in range(len(BTN_NAMES))}


def joy_to_step(raw, max_step):
    if raw < DEAD_LOW:
        factor = (DEAD_LOW - raw) / DEAD_LOW
        return -factor * max_step
    elif raw > DEAD_HIGH:
        factor = (raw - DEAD_HIGH) / (JOY_MAX - DEAD_HIGH)
        return factor * max_step
    return 0.0


def build_gcode(dx, dy, dz, dyaw, t_state,slow = False):
    parts = []
    dx *= -1
    dy *= -1
    dz *= -1
    dyaw *= -1
    if abs(dx)   > 1e-6: parts.append(f"X{dx:.3f}")
    if abs(dy)   > 1e-6: parts.append(f"Y{dy:.3f}")
    if abs(dz)   > 1e-6: parts.append(f"Z{dz:.3f}")
    if abs(dyaw) > 1e-6: parts.append(f"E{dyaw:.3f}")
    if not parts:
        return f"G1 X0 Y0 F500"
    feed = FEED_RATE_SLOW if all(p.startswith("Z") for p in parts) else FEED_RATE
    feed = 7000 if (all(p.startswith("E") for p in parts) and t_state) else feed
    feed = FEED_RATE_SLOW if slow else feed
    return f"G1 {' '.join(parts)} F{feed}"

def get_openpnp_pids():
    try:
        out = subprocess.check_output(["pgrep", "-f", "openpnp"], universal_newlines=True)
        return [int(p) for p in out.strip().splitlines()]
    except subprocess.CalledProcessError:
        return []


def kill_openpnp():
    pids = get_openpnp_pids()
    if not pids:
        return
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
            print(f"; Sent SIGTERM to openpnp PID {pid}")
        except ProcessLookupError:
            pass


def send(mach, line):
    print(line)
    if mach is not None:
        try:
            mach.write((line + "\n").encode())
        except serial.SerialException as e:
            print(f"; USB0 write error: {e}")


def try_open_machine():
    try:
        s = serial.Serial(MACH_PORT, MACH_BAUD, timeout=1)
        print(f"; Connected to machine on {MACH_PORT} at {MACH_BAUD} baud")
        return s
    except serial.SerialException as e:
        print(f"; Could not open {MACH_PORT}: {e}")
        return None


def main():
    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        print(f"; Connected to {PORT} at {BAUD} baud")

        t_state     = 0
        v_state     = 0
        kill_done   = False
        prev_sel1   = False
        prev_sel2   = False
        prev_manual = False
        mach        = None

        while True:
            raw = ser.readline().decode(errors="ignore").strip()
            if not raw:
                continue

            m = re.match(r"B:(\w+) X1:(\w+) Y1:(\w+) X2:(\w+)", raw)
            if not m:
                continue

            buttons_byte = int(m.group(1), 16)
            x1           = int(m.group(2), 16)
            y1           = int(m.group(3), 16)
            x2           = int(m.group(4), 16)

            btn    = parse_buttons(buttons_byte)
            manual = btn["MANUAL"]

            # ── MANUAL rising edge: kill openpnp, open machine port ───────────
            if manual and not prev_manual:
                kill_openpnp()
                kill_done = True
                mach = try_open_machine()
                time.sleep(2)
                send(mach, "G91")

            # ── MANUAL falling edge: close machine port ───────────────────────
            if not manual and prev_manual:
                kill_done = False
                if mach is not None:
                    mach.close()
                    mach = None
                    print(f"; Disconnected from {MACH_PORT}")

            prev_manual = manual

            # ── Everything below is manual-mode only ──────────────────────────
            if not manual:
                continue

            # ── SEL1: toggle T state on rising edge ───────────────────────────
            if btn["SEL1"] and not prev_sel1:
                t_state = 1 - t_state
                # send(mach, f"T{t_state}")
                send(mach, "G28")
            prev_sel1 = btn["SEL1"]

            # ── SEL2: toggle V state on rising edge ───────────────────────────
            if btn["SEL2"] and not prev_sel2:
                v_state = 1 - v_state
                send(mach, "M42 P4 S255" if v_state == 1 else "M42 P4 S0")
            prev_sel2 = btn["SEL2"]

            # ── Joystick coarse control ───────────────────────────────────────
            dx   =  joy_to_step(x1, MAX_STEP_XY)
            dy   = -joy_to_step(y1, MAX_STEP_XY)
            dyaw =  joy_to_step(x2, MAX_STEP_YAW * (1 if t_state == 0 else 10))
            dz   = 0.0

            # ── Button fine control ───────────────────────────────────────────
            if btn["LEFT"]:   dx   -= FINE_XY
            if btn["RIGHT"]:  dx   += FINE_XY
            if btn["FOR"]:    dy   += FINE_XY
            if btn["BACK"]:   dy   -= FINE_XY
            if btn["UP"]:     dz   += FINE_Z
            if btn["DOWN"]:   dz   -= FINE_Z
            if btn["YAW_L"]:  dyaw += FINE_YAW
            if btn["YAW_R"]:  dyaw -= FINE_YAW

            slow = btn["FOR"] or btn["BACK"] or btn["LEFT"] or btn["RIGHT"]

            line = build_gcode(dx, dy, dz, dyaw, t_state, slow)
            send(mach, line)


if __name__ == "__main__":
    main()

