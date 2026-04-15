# NanoPnP User Manual

## Quick Start

1. Power on the machine (MKS Gen L board + vacuum pump)
2. Connect USB serial cable and bottom camera
3. Launch: `python run.py --gui`
4. Select serial port from dropdown, click **Connect**
5. Machine homes automatically (G28)

## Workflow Overview

A typical job has two phases:

1. **Solder paste** — dispense paste onto pads before placing components
2. **Pick and place** — pick components from tape feeders, align with vision, place on board

You can run either phase independently or both together.

## Board Setup

### Physical Setup

1. Secure the PCB to the bed using tape or clamps
2. Note the board's bottom-left corner position in machine coordinates
3. Jog the nozzle to the board origin (bottom-left of the PCB outline) using arrow keys or jog buttons
4. Enter the coordinates in **Setup > Board Origin X/Y**
5. Set **Board Place Z** — jog Z down until the nozzle just touches the board surface

### Loading Job Files

Go to the **Job** tab. You need up to three files from KiCad:

| File | Purpose | KiCad export |
|------|---------|--------------|
| **Edge_Cuts** (.gbr) | Board outline — defines paste boundaries | Plot > Edge.Cuts layer |
| **F_Paste** (.gbr) | Paste pad shapes — defines where to dispense | Plot > F.Paste layer |
| **Component .pos** (.pos) | Component positions and rotations | Fabrication Outputs > Footprint Position File |

Click each file picker to load them. The status bar shows which mode is available (Paste Only / Place Only / Full Job) based on which files are loaded.

## Solder Paste Dispensing

### Feeder Setup for Paste

1. Install the solder paste syringe on the **T1** tool mount
2. In the GUI, the paste dispenser uses the E1 extruder in relative mode (M83)

### Running Paste

1. Load Edge_Cuts + F_Paste files
2. Click **Solder Only**
3. The machine will:
   - Switch to T1 (paste tool)
   - Move to each pad location
   - Lower to paste Z, extrude a line across the pad, retract
   - Repeat for all pads

### Paste Parameters (Setup tab)

- **Feed XY / Feed Z**: travel speeds for paste moves
- **E Rate**: extrusion rate (mm of paste per mm of travel)
- **E Prime**: amount to prime before starting a line (compensates for syringe backlash)
- **E Retract**: amount to retract after a line (prevents dripping)
- **Z Travel**: safe height between pads

### Manual Paste Touchup

If you need to redo a single pad:
1. Jog to the pad position
2. Switch to T1 paste tool via console: `T1` then `M83`
3. Lower Z to paste height
4. Send manual extrude: `G1 E<amount> F<rate>`

## Pick and Place

### Feeder Setup

1. Load component tape into feeders
2. In **Setup > Feeders**, configure each feeder:
   - **Part ID**: matches the package map (e.g. IC_SOIC8)
   - **Ref Hole**: XYZ of the first tape sprocket hole
   - **Last Hole**: XYZ of a later sprocket hole (defines tape direction)
   - **Max Count**: number of parts in the tape
   - **Rotation**: part orientation in tape (per EIA-481)
3. Click **Reset Count** to zero the feed counter when loading new tape

### Package Map

The .pos file uses KiCad footprint names (e.g. "SOIC-8"). The **Package Map** in Setup translates these to feeder part IDs:

```
SOIC-8    -> IC_SOIC8
SOT-23-6  -> IC_SOT236
LQFN-16   -> IC_LQFN16
```

Add entries for any new footprints.

### Running Pick and Place

1. Load the .pos file
2. Enable/disable individual placements in the **Placements** table
3. Click **Pick & Place Only**
4. The machine will for each component:
   - Move to feeder, lower, vacuum pick, retract
   - Move to bottom camera, rotate to placement angle
   - (If vision enabled) detect part offset and correct
   - Move to board position, lower, release vacuum, retract
   - Advance feeder count

### Vision Alignment

Enable vision with the **Vision** checkbox in the toolbar.

- **Test Vision**: Camera tab > select part type > click **Test Vision** to see the detection result without moving the machine
- **Apply Correction**: after Test Vision, click to actually move the nozzle by the detected offset
- **Vision Apply** toggle: when off, vision detects but doesn't correct — useful for verifying detection without affecting placement

Vision runs up to 3 passes per part. First pass corrects rotation, subsequent passes correct XY.

### Calibrating the Camera

1. Pick up a known-size part (e.g. SOIC8: 3.9 x 4.9mm)
2. Move to camera position: Camera tab > **Go to Camera**
3. Click **Calibrate UPP** > enter the known width and height > **Detect & Calibrate**
4. Review the measured vs expected sizes > **Apply**

## Running a Full Job

1. Load all three files (Edge_Cuts, F_Paste, .pos)
2. Click **Full Job (paste + place)**
3. Phase 1: paste is dispensed on all pads
4. Phase 2: components are picked and placed

## Controls During a Job

- **Pause / Resume**: pause the job at the current step
- **STOP**: abort the job, machine returns home
- **E-STOP** (Escape key): immediate halt, sends M112

## Console

The **Console** tab shows all G-code sent and received. You can type commands directly:

```
G1 X50 Y30 F1000     # move to position
M119                  # check endstop states
M114                  # report current position
G28                   # home all axes
```

## Tips

- Always do a **Dry Run** first to verify the job without moving the machine
- Use **Go To** on a feeder to verify its position before running
- Check **Reset Count** before loading fresh tape
- If a placement fails, the engine retries up to 3 times with discard between attempts
- The G-code visualizer (Visualizer tab or CLI) previews the full toolpath before running
