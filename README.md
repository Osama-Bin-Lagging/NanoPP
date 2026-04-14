# NanoPnP

Custom pick-and-place machine controller for MKS Gen L V2.1 + Marlin firmware. Built as a simpler, more controllable replacement for OpenPnP.

## Why

OpenPnP's NullMotionPlanner merges Z and XY into diagonal G1 commands, dragging parts across the machine after pick. It also sends duplicate vacuum commands causing serial timeouts, and vision calibration is broken. NanoPnP fixes all three by controlling G-code directly — Z and XY are always separate commands.

## Features

- **Safe motion**: Z retracts before XY travel, always. Never diagonal.
- **Solder paste dispensing**: Draws paste lines for SOIC8, SOT-23-6, LQFN16 packages via E1 extruder
- **Pick and place**: Vacuum pick from tape feeders, bottom vision alignment, corrected placement
- **Bottom vision**: Per-part OpenCV pipelines — blob detection for QFN/SOT, multi-pad threshold for SOIC8 — with density clustering and MinAreaRect
- **GUI**: Tkinter interface with jog controls, camera preview, config editing, job file pickers, local/remote target selector
- **CLI**: Headless operation via `run.py` commands
- **G-code export**: Save job output to file without connecting to hardware
- **G-code visualizer**: 2D/3D path plots, animation, and travel statistics
- **Remote execution**: Send jobs to a Jetson (or any Linux host) over SCP, monitor status from GUI
- **Manual joystick control**: Pico-based controller with joystick + buttons for direct machine jog, overrides running jobs via MANUAL switch
- **Watcher daemon**: Systemd service that picks up file-drop jobs, runs them, archives results
- **Cross-platform**: macOS + Ubuntu 18+

## Hardware

- MKS Gen L V2.1 (ATmega2560)
- Marlin 2.1.2.7
- Serial @ 250000 baud (auto-retry on connect, up to 5 attempts)
- Juki vacuum nozzle on E0 (rotation via M82 absolute)
- Solder paste dispenser on E1 (via M83 relative)
- USB bottom camera
- Vacuum pump: M42 P4 S0 (on) / S255 (off) — active-low relay

## Setup

```bash
cd NanoPnP
uv venv --python python3.12
source .venv/bin/activate
uv pip install pyserial opencv-python Pillow gerbonara matplotlib
```

## Usage

### GUI
```bash
python run.py --gui
```

1. Select serial port from dropdown (or check "Dry Run")
2. Click Connect
3. Use jog controls / arrow keys to move
4. Load job files (Edge_Cuts, F_Paste, .pos) via the Job tab file pickers
5. Choose target: Local or Jetson (remote)
6. Run Solder Only / Pick & Place Only / Full Job

### CLI
```bash
python run.py --dry-run home                # home all axes
python run.py --dry-run jog 50 30           # move to X50 Y30

# paste dispensing (from Gerber files)
python run.py --dry-run paste \
    --edge-cuts board-Edge_Cuts.gbr --paste board-F_Paste.gbr

# pick and place (from .pos file, optional vision)
python run.py --dry-run place --pos board-top.pos
python run.py --dry-run --vision place --pos board-top.pos

# full job: paste then place
python run.py --dry-run job \
    --edge-cuts board-Edge_Cuts.gbr --paste board-F_Paste.gbr \
    --pos board-top.pos

# export G-code to file
python run.py --dry-run -o out.gcode job \
    --edge-cuts board-Edge_Cuts.gbr --paste board-F_Paste.gbr \
    --pos board-top.pos
```

### G-code Visualizer
```bash
python -m nanopnp.visualizer output.gcode             # 2D plot
python -m nanopnp.visualizer --3d output.gcode         # 3D machine scene
python -m nanopnp.visualizer --animate output.gcode    # step-by-step playback
python -m nanopnp.visualizer --stats output.gcode      # travel summary
```

### Manual Joystick Control
```bash
# uses controller.port from config.json
python run.py manual

# or specify the controller port explicitly
python run.py manual --controller-port /dev/tty.usbmodem1401
```

Requires a Pico controller sending `B:<hex_buttons> X1:<hex> Y1:<hex> X2:<hex>` frames over serial (11 buttons: 8 push, 2 joystick click, 1 MANUAL switch + 3 joystick ADC values).

Flip the MANUAL switch to take over the machine — any running NanoPnP process (GUI or CLI job) is gracefully terminated via SIGTERM, then the machine port is opened in relative (G91) mode. Joysticks provide coarse XY/yaw, buttons provide fine step control. SEL1 toggles tool (T0/T1), SEL2 toggles vacuum. Flip MANUAL off to release the port.

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

- **Machine**: serial port, feedrate (XY: 1000, Z: 1000), axis limits
- **Z heights**: safe (0), retract (30), board surface (18), feeder pick (23), discard (23)
- **Paste dispensing**: feedrates (XY, Z, E), extrusion rate, prime/retract amounts, travel Z
- **Feeders**: position, pitch, tape direction, feed count tracking, max count
- **Parts**: SOIC8/SOT236/LQFN16 with heights, pad layouts, yaw limits
- **Board**: origin, place Z, package map (.pos package names -> feeder part IDs), skip prefixes
- **Vision**: threshold, per-part area limits, mask diameter, convergence tolerance, max passes
- **Camera**: device index, position offset, units-per-pixel
- **Controller** (optional): joystick port, baud, deadzone, step sizes, feedrates for manual mode
- **Remote** (optional): SSH host, user, key, incoming/status paths for Jetson target

Changes made in the GUI auto-save to `config.json`.

## Architecture

```
run.py                   CLI entry point
nanopnp/
  config.py              JSON config loader + saver (dataclasses)
  serial_comm.py         Thread-safe serial I/O, dry-run mode, G-code file export
  motion.py              Safe Z sequencing, vacuum control, tool select, axis limits
  board_parser.py        KiCad Gerber/.pos parser (uses gerbonara)
  paste_dispenser.py     Solder paste line generation for 3 package types
  feeder.py              Tape feeder position calculation + state persistence
  vision.py              OpenCV bottom vision — per-part pipelines + UPP calibration
  pnp_engine.py          Pick-align-place orchestrator with retry + drift tracking
  gui.py                 Tkinter GUI (light theme, cross-platform)
  job_inputs.py          Job file classification (Edge_Cuts, F_Paste, .pos -> mode)
  remote.py              SCP job upload + SSH status polling for Jetson targets
  manual_control.py      Joystick/button manual jog via external controller
  visualizer.py          G-code path plotting, animation, travel stats
  watcher.py             File-drop daemon: watch incoming/ -> run job -> archive
scripts/
  install-watcher.sh     Systemd service installer for the watcher daemon
  nanopnp-watcher.service  Systemd unit template
tune_vision.py           Vision threshold tuning across captured images
annotate.py              3-point rotated bbox annotation tool for vision ground truth
```

## Vision Pipelines

Each part type has a tuned detection strategy:

| Part | Strategy | Key params |
|------|----------|------------|
| **SOIC-8** | Grayscale threshold, 8 pads, density clustering | thresh=190, no mask |
| **SOT-23-6** | Grayscale threshold + center mask, 6 pads | thresh=190, 200px mask |
| **LQFN-16** | High threshold blob detection (pads merge) | thresh=230, 250px mask |

**SOIC-8**: Large gull-wing leads are bright and well-separated. Simple threshold at 190 detects individual pads, density clustering selects the 8 most tightly grouped.

**SOT-23-6**: Smaller pads, same threshold approach but with a circular mask (200px radius) centered on the image to exclude metal arm reflections.

**LQFN-16**: Tiny QFN pads are saturated bright (254) and merge into one large blob at threshold 230. Instead of separating individual pads, the pipeline finds the single largest bright blob near center — its MinAreaRect directly gives center, size, and angle.

### Vision Tuning

```bash
# Annotate ground truth bboxes on captures
python annotate.py capture_*.png

# Test thresholds across all captures
python tune_vision.py --part lqfn16
```

## Pick-and-Place Cycle

```
For each enabled placement:
  1. PICK:    safe_move to feeder -> lower Z -> vacuum ON -> retract to Z_retract
  2. CAMERA:  safe_move to camera position (Z retracts automatically)
  3. ROTATE:  compensate for tape orientation at camera (E axis)
  4. ALIGN:   (if vision) two-phase: correct rotation first, then XY (up to 3 passes)
  5. PLACE:   safe_move to board with final rotation -> lower Z -> vacuum OFF -> retract
  6. ADVANCE: increment feeder count
```

Nozzle rotation happens at the camera position (not at the feeder) so the part is in its placement orientation for vision alignment. The `safe_move_to` function always resets E to 0 during XY travel, so rotation is applied after arriving at each waypoint.

## Remote / Jetson Deployment

NanoPnP supports a two-tier setup: laptop GUI for design & monitoring, remote host (e.g. Jetson) for headless execution.

**On the Jetson:**
```bash
cd ~/NanoPnP
sudo scripts/install-watcher.sh
```

This installs a systemd service that watches `~/nanopnp/incoming/` for job files. When a complete set (Edge_Cuts + F_Paste + .pos) arrives, the watcher runs the job, writes progress to `status.json`, and archives results to `~/nanopnp/done/` (or `~/nanopnp/failed/` on error). Retries up to 3 times with exponential backoff.

**From the laptop GUI:**
1. Configure the Jetson target in Setup tab (host, user, SSH key)
2. Select "Jetson" target in the toolbar
3. Load job files and click Run — files are uploaded via SCP
4. Status bar polls `status.json` for live progress

## Vision Calibration

The bottom camera needs correct units-per-pixel values for accurate alignment:

1. Place a known-size component (e.g. SOIC8 body: 3.9 x 4.9 mm) on the camera
2. Move nozzle to camera Z = part height
3. Camera tab -> "Calibrate UPP..." -> enter dimensions -> Detect & Calibrate
4. Review measured vs known -> Apply -> saves to config

## License

MIT
