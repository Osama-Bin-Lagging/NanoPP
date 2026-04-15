# NanoPnP User Manual

## How It Works

NanoPnP runs on a Jetson connected to the pick-and-place machine. You submit jobs by dropping files into a folder. The machine does the rest.

## Submitting a Job

Export two files from KiCad:

- **F_Paste gerber** (`*-F_Paste.gbr`) — paste pad shapes
- **Position file** (`*-top.pos`) — component positions and rotations

Copy them to the Jetson:
```bash
scp board-F_Paste.gbr board-top.pos user@jetson:~/nanopnp/incoming/
```

The watcher picks up the files after 5 seconds and starts the job automatically.

## Job Modes

What runs depends on which files you drop:

| Files dropped | What happens |
|--------------|--------------|
| F_Paste only | Solder paste dispensing |
| .pos only | Pick and place |
| Both | Paste first, then pick and place |

## Monitoring

Check job status:
```bash
ssh user@jetson cat ~/nanopnp/status.json
```

Or connect the laptop GUI (`python run.py --gui`) and select **Jetson** target — the toolbar shows live progress.

## After the Job

- **Success** — files archived to `~/nanopnp/done/<timestamp>/`
- **Failure** — retries 3 times, then moves to `~/nanopnp/failed/` with error log

View logs:
```bash
journalctl -u nanopnp-watcher -f
```

## Changing Tape

When a feeder runs out, load new tape and reset the count in `config.json`:

```json
"F1": {
  "feed_count": 0,
  "max_count": 5000
}
```

## Changing the Board

Update `board.origin` in `config.json` with the new PCB's bottom-left corner position. Update `board.place_z` if the board thickness changed.

## Laptop GUI (Optional)

```bash
python run.py --gui
```

Use for: manual jog, camera preview, vision testing, feeder calibration, config editing. Connect to Jetson target to monitor remote jobs, or Local target to control the machine directly.
