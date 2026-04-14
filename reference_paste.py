import re
import sys
import math
import time
import serial

# --- Machine Config ---
# MACH_PORT   = "/dev/ttyUSB0"
MACH_PORT   = "/dev/tty.usbserial-2130"
MACH_BAUD   = 250000

FEED_XY     = 1000
FEED_XY_SLOW= 100
FEED_Z      = 500
FEED_PASTE  = 5000

Z_TRAVEL    = 5.0
Z_DISPENSE  = 0.2

E_RATE      = 400
E_RETRACT   = 50

# Coordinates are used exactly as found in the GBR file
X_OFFSET    = 0.0
Y_OFFSET    = 0.0
Y_FLIP      = False  # Disabled: Use Gerber Y as is

# --- GBR Parsing ---
def parse_gbr(filename):
    try:
        with open(filename) as f:
            content = f.read()
    except FileNotFoundError:
        return None

    components = {}
    # Extract pads by footprint reference
    for m in re.finditer(r'%TO\.C,(\S+?)\*%(.+?)%TD\*%', content, re.DOTALL):
        ref, block = m.group(1), m.group(2)
        pads = []
        for pad in re.finditer(r'X(-?\d+)Y(-?\d+)D03', block):
            x = int(pad.group(1)) / 1e6  # 4.6 format scaling
            y = int(pad.group(2)) / 1e6
            pads.append((x, y))
        if pads:
            components.setdefault(ref, []).extend(pads)
    return components

# --- Geometry & Row Grouping ---
def dist(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

def pca_direction(pads):
    n = len(pads)
    mx, my = sum(p[0] for p in pads)/n, sum(p[1] for p in pads)/n
    cxx = sum((p[0]-mx)**2 for p in pads)
    cyy = sum((p[1]-my)**2 for p in pads)
    cxy = sum((p[0]-mx)*(p[1]-my) for p in pads)
    if abs(cxy) > 1e-10:
        disc = math.sqrt(max(0, ((cxx-cyy)/2)**2 + cxy**2))
        l1 = (cxx+cyy)/2 + disc
        vx, vy = l1 - cyy, cxy
        mag = math.sqrt(vx**2 + vy**2)
        return vx/mag, vy/mag
    return (1.0, 0.0) if cxx >= cyy else (0.0, 1.0)

def group_into_rows(pads):
    if len(pads) <= 1: return [list(pads)]
    min_d = min(dist(pads[i], pads[j]) for i in range(len(pads)) for j in range(i+1, len(pads)))

    n = len(pads); parent = list(range(n))
    def find(i):
        if parent[i] == i: return i
        parent[i] = find(parent[i]); return parent[i]
    def union(i, j):
        root_i, root_j = find(i), find(j)
        if root_i != root_j: parent[root_i] = root_j

    threshold = min_d * 1.8
    for i in range(n):
        for j in range(i+1, n):
            if dist(pads[i], pads[j]) <= threshold: union(i, j)

    groups = {}
    for i in range(n): groups.setdefault(find(i), []).append(pads[i])

    rows = []
    for g in groups.values():
        n_g = len(g)
        mx, my = sum(p[0] for p in g)/n_g, sum(p[1] for p in g)/n_g
        vx, vy = pca_direction(g)
        rows.append(sorted(g, key=lambda p: (p[0]-mx)*vx + (p[1]-my)*vy))

    # Global sort: prioritize Y coordinate (as given) then X
    rows.sort(key=lambda r: (sum(p[1] for p in r)/len(r), sum(p[0] for p in r)/len(r)))
    return rows

def transform(x, y):
    # Pure passthrough of coordinates
    new_x = x + X_OFFSET
    new_y = (-y if Y_FLIP else y) + Y_OFFSET
    return new_x, new_y

# --- G-Code Generation ---
def generate_commands(components):
    cmds = [
        "G21 ; Units: mm",
        "G90 ; Absolute Positioning",
        "M302 S0 ; Marlin: Allow Cold Extrusion",
        "M82 ; Absolute E",
        "G92 E0 ; Reset Extruder",
        f"G1 Z{Z_TRAVEL:.4f} F{FEED_Z} ; Safe height",
        "T1"
    ]

    curr_e = 0.0
    for ref, pads in components.items():
        rows = group_into_rows(pads)
        for row in rows:
            x0, y0 = transform(*row[0])
            xn, yn = transform(*row[-1])
            row_len = dist((x0, y0), (xn, yn))

            # 1. Travel to start of pad/row
            cmds.append(f"G0 X{x0:.4f} Y{y0:.4f} F{FEED_XY}")
            # 2. Lower needle
            # cmds.append(f"G1 Z{Z_DISPENSE:.4f} F{FEED_Z}")

            # 3. Dispense
            e_move = E_RATE
            curr_e += e_move
            cmds.append(f"G1 E{curr_e:04.0f} F{FEED_PASTE}")
            cmds.append(f"G1 X{xn:.4f} Y{yn:.4f} F{FEED_XY_SLOW}")

            # 4. Retract and Lift
            curr_e -= E_RETRACT
            cmds.append(f"G1 E{curr_e:04.0f} F{FEED_PASTE}")
            # cmds.append(f"G1 Z{Z_TRAVEL:.4f} F{FEED_Z}")

    cmds += [f"G1 X0 Y0 F{FEED_XY}", "M30 ; End of program"]
    return cmds

# --- Serial Uploader ---
def main():
    if len(sys.argv) < 2:
        print("Usage: python script.py <input.gbr>")
        return

    components = parse_gbr(sys.argv[1])
    if not components:
        print("Error: Could not parse Gerber.")
        return

    gcode_list = generate_commands(components)

    # Output to file for verification
    with open("reference_output.gcode", "w") as f:
        f.write("\n".join(gcode_list))

    print(f"Generated reference_output.gcode ({len(gcode_list)} lines)")

    # Skip serial upload in this test mode
    return

if __name__ == "__main__":
    main()
