# NanoPnP Production Manual

NanoPnP runs headless on a Jetson connected to the PnP machine. Jobs are submitted by dropping files into a folder — either locally on the Jetson or from a laptop over SCP. A laptop GUI can optionally connect for monitoring and manual control.

## Jetson Setup

### Install

```bash
git clone <repo> ~/NanoPnP
cd ~/NanoPnP
pip install pyserial opencv-python gerbonara
```

### Configure

Edit `config.json` with your machine's values:

- **serial_port**: your USB serial device (e.g. `/dev/ttyUSB0`)
- **baud_rate**: 250000 for MKS Gen L
- **board.origin**: XY of the PCB's bottom-left corner in machine coords
- **board.place_z**: Z height where nozzle touches the board surface
- **feeders**: one entry per tape feeder with ref_hole position, pitch, part ID, max_count
- **board.package_map**: maps KiCad footprint names to feeder part IDs (e.g. `"SOIC-8": "IC_SOIC8"`)

### Install the Watcher Service

```bash
cd ~/NanoPnP
sudo scripts/install-watcher.sh
```

This creates:
- `~/nanopnp/incoming/` — drop job files here
- `~/nanopnp/done/` — completed jobs archived here
- `~/nanopnp/failed/` — failed jobs moved here with error logs
- `~/nanopnp/status.json` — live status (polled by laptop GUI)

The watcher starts automatically on boot via systemd.

Check it's running:
```bash
journalctl -u nanopnp-watcher -f
```

## Submitting Jobs

### Job Files

Export three files from KiCad:

| File | KiCad export | What it does |
|------|-------------|--------------|
| `*-Edge_Cuts.gbr` | Plot > Edge.Cuts layer | Board outline for paste boundaries |
| `*-F_Paste.gbr` | Plot > F.Paste layer | Pad shapes for paste dispensing |
| `*-top.pos` | Fabrication > Footprint Position File | Component positions for pick and place |

The watcher detects the job mode from which files are present:
- Edge_Cuts + F_Paste = **paste only**
- .pos only = **pick and place only**
- All three = **full job** (paste then place)

### Drop Files on the Jetson

Copy files directly:
```bash
cp board-Edge_Cuts.gbr board-F_Paste.gbr board-top.pos ~/nanopnp/incoming/
```

The watcher waits 5 seconds after the last file arrives (debounce), then runs the job.

### Drop Files from Laptop

SCP the files:
```bash
scp board-Edge_Cuts.gbr board-F_Paste.gbr board-top.pos user@jetson:~/nanopnp/incoming/
```

Or use the GUI (see Laptop Monitoring below).

## What Happens During a Job

### Paste Phase

1. Machine homes (G28)
2. Switches to T1 (paste syringe)
3. For each pad: moves to position, lowers Z, extrudes paste line, retracts
4. Returns home

### Pick and Place Phase

1. Machine homes, switches to T0 (vacuum nozzle)
2. For each enabled placement:
   - Moves to feeder, lowers Z, vacuum picks the part, retracts
   - Moves to bottom camera, rotates nozzle to placement angle
   - If vision enabled: detects part offset, corrects position (up to 3 passes)
   - Moves to board position, lowers, releases vacuum, retracts
   - Advances feeder tape count
3. Returns home

### After the Job

- **Success**: files archived to `~/nanopnp/done/<timestamp>/`
- **Failure**: retries up to 3 times with backoff, then moves to `~/nanopnp/failed/<timestamp>/` with error log

Check status:
```bash
cat ~/nanopnp/status.json
```

## CLI Usage (on the Jetson)

Run jobs directly without the watcher:

```bash
# Paste only
python run.py paste --edge-cuts board-Edge_Cuts.gbr --paste board-F_Paste.gbr

# Pick and place only
python run.py place --pos board-top.pos

# Pick and place with vision
python run.py --vision place --pos board-top.pos

# Full job
python run.py job --edge-cuts board-Edge_Cuts.gbr --paste board-F_Paste.gbr --pos board-top.pos

# Dry run (no machine movement, simulates everything)
python run.py --dry-run job --edge-cuts board-Edge_Cuts.gbr --paste board-F_Paste.gbr --pos board-top.pos

# Export G-code to file
python run.py --dry-run -o output.gcode job --edge-cuts board-Edge_Cuts.gbr --paste board-F_Paste.gbr --pos board-top.pos
```

## Laptop Monitoring (Optional)

The laptop GUI connects to the Jetson for monitoring and manual control.

### Setup

```bash
# On the laptop
cd NanoPnP
uv venv --python python3.12 && source .venv/bin/activate
uv pip install pyserial opencv-python Pillow gerbonara matplotlib
python run.py --gui
```

### Connect to Jetson

1. In the GUI toolbar, select **Jetson** target
2. Configure in Setup tab: host IP, username, SSH key path
3. The status bar polls `status.json` and shows live job progress

### Submit Jobs from GUI

1. Load job files in the Job tab (Edge_Cuts, F_Paste, .pos)
2. Select **Jetson** target
3. Click Run — files are uploaded via SCP, watcher picks them up

### Direct Machine Control

To connect the laptop directly to the machine (for setup, calibration, manual work):

1. Select **Local** target, enter the serial port
2. Click **Connect**
3. Use jog controls, camera preview, feeder management, vision testing

## Changing Feeder Tape

1. Load new tape into the feeder slot
2. Either:
   - **Jetson CLI**: edit `config.json`, set `feed_count: 0` and `max_count: <tape length>` for that feeder
   - **Laptop GUI**: Setup tab > select feeder > **Reset Count**, edit max_count
3. Verify position: jog to the feeder's ref_hole to confirm alignment

## Changing the Board

1. Secure new PCB to the bed
2. Jog nozzle to the board's bottom-left corner
3. Update `board.origin` in `config.json` with the new XY coordinates
4. Update `board.place_z` if board thickness changed
5. Update `board.package_map` if the new board has different footprints

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| Watcher not picking up files | `journalctl -u nanopnp-watcher -f` — check for errors |
| Serial timeout | Check USB connection, try unplugging and reconnecting |
| Parts placed in wrong position | Re-check `board.origin` — jog nozzle to corner and verify |
| Vision failing | Check camera connection, run `Test Vision` from laptop GUI |
| Feeder picks missing | Verify `ref_hole` position — use GUI's **Go To** on the feeder |
| Paste not extruding | Check syringe, verify T1 tool is selected, check E rate in config |
