# NanoPnP

Custom pick-and-place machine controller for MKS Gen L V2.1 + Marlin firmware. Built as a simpler, more controllable replacement for OpenPnP.

## Why

OpenPnP's NullMotionPlanner merges Z and XY into diagonal G1 commands, dragging parts across the machine after pick. It also sends duplicate vacuum commands causing serial timeouts, and vision calibration is broken. NanoPnP fixes all three by controlling G-code directly — Z and XY are always separate commands.

## Features

- **Safe motion**: Z retracts before XY travel, always. Never diagonal.
- **Solder paste dispensing**: Draws paste lines for SOIC8, SOT-23-6, LQFN16 packages via E1 extruder
- **Pick and place**: Vacuum pick from tape feeders, optional bottom vision alignment, corrected placement
- **Bottom vision**: OpenCV pipeline (HSV mask → threshold → contour → MinAreaRect) with per-part configs imported from OpenPnP
- **GUI**: Light-themed tkinter interface with jog controls, camera preview, config editing, job execution
- **CLI**: Headless operation via `run.py` commands
- **G-code export**: Save job output to file without connecting to hardware
- **Cross-platform**: macOS + Ubuntu 18+

## Hardware

- MKS Gen L V2.1 (ATmega2560)
- Marlin 2.1.2.7
- Serial @ 250000 baud
- Juki vacuum nozzle on E0 (rotation via M82 absolute)
- Solder paste dispenser on E1 (via M83 relative)
- USB bottom camera
- Vacuum pump: M42 P4 S0 (on) / S255 (off) — active-low relay

## Setup

```bash
cd NanoPnP
uv venv --python python3.12
source .venv/bin/activate
uv pip install pyserial opencv-python Pillow gerbonara
```

## Usage

### GUI
```bash
python run.py --gui
```

1. Select serial port from dropdown (or check "Dry Run")
2. Click Connect
3. Use jog controls / arrow keys to move
4. Load board from Gerber directory or use config placements
5. Run Solder Only / Pick & Place Only / Full Job

### CLI
```bash
python run.py --dry-run home              # home all axes
python run.py --dry-run jog 50 30         # move to X50 Y30
python run.py --dry-run paste             # dispense solder paste
python run.py --dry-run place             # pick and place
python run.py --dry-run job               # paste + place
python run.py --dry-run -o out.gcode job  # save G-code to file
```

### Keyboard Shortcuts (GUI)
| Key | Action |
|-----|--------|
| Arrow keys | Jog XY |
| Page Up/Down | Jog Z |
| Home | Home all axes |
| Escape | E-STOP |
| Up/Down (in console) | Command history |

## Configuration

All parameters are in `config.json` and editable from the GUI's Setup tab:

- **Machine**: serial port, feedrate (XY: 1000, Z: 500), axis limits
- **Z heights**: safe (0), board (16), feeder (23), discard (23)
- **Feeders**: position, pitch, tape direction, feed count tracking
- **Parts**: SOIC8/SOT236/LQFN16 with heights and pad layouts from OpenPnP packages.xml
- **Vision**: HSV thresholds, blur, mask diameter, convergence tolerance
- **Camera**: device index, position, units-per-pixel (calibrate via GUI)

Changes made in the GUI auto-save to `config.json`.

## Architecture

```
run.py                 CLI entry point
nanopnp/
  config.py            JSON config loader + saver (dataclasses)
  serial_comm.py       Thread-safe serial I/O, dry-run mode, G-code file export
  motion.py            Safe Z sequencing, vacuum control, tool select, axis limits
  board_parser.py      KiCad Gerber/.pos parser (uses gerbonara)
  paste_dispenser.py   Solder paste line generation for 3 package types
  feeder.py            Tape feeder position calculation + state persistence
  vision.py            OpenCV bottom vision pipeline + UPP calibration
  pnp_engine.py        Pick-align-place orchestrator with retry + drift tracking
  gui.py               Tkinter GUI (light theme, cross-platform)
```

## Pick-and-Place Cycle

```
For each enabled placement:
  1. PICK:   safe_move to feeder → lower Z → vacuum ON → retract
  2. ROTATE: compensate for tape orientation (E axis)
  3. ALIGN:  (if vision on) move to camera → detect → correct (up to 3 passes)
  4. PLACE:  safe_move to board → lower Z (board + part height) → vacuum OFF ��� retract
  5. ADVANCE feeder count
```

## Vision Calibration

The bottom camera needs correct units-per-pixel values for accurate alignment:

1. Place a known-size component (e.g. SOIC8 body: 3.9 x 4.9 mm) on the camera
2. Move nozzle to camera Z = part height
3. Camera tab → "Calibrate UPP..." → enter dimensions → Detect & Calibrate
4. Review measured vs known → Apply → saves to config

## License

MIT
