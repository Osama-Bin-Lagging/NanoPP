# NanoPnP — Detailed Implementation Plans

## Module 1: Configuration & Project Structure

### Files to Create
- `nanopnp/__init__.py` — version string
- `config.json` — all machine parameters
- `nanopnp/config.py` — dataclass loader + validation
- `run.py` — CLI entry point with subcommands
- Stub files for all other modules

### config.json Structure
```
machine       → serial_port, baud_rate, axis_limits, travel_feedrate
z_heights     → safe(0), board_surface(16), feeder_pick(23), discard(23)
camera        → device_index(1), position(10.5, 32.7), units_per_pixel, settle_time_ms
vacuum        → on_cmd(M42 P4 S0), off_cmd(M42 P4 S255), pick/place_dwell_ms(250)
paste         → travel_feedrate(3000), dispense_feedrate(300), extrude_per_mm(0.05)
board         → origin(0,0), place_z(16)
vision        → max_passes(3), max_linear(1.0mm), max_angular(10°), hsv thresholds
parts         → IC_SOIC8, IC_SOT236, IC_LQFN16 with heights + pad layouts
feeders       → F1(SOIC8), F3(SOT236) with positions, pitch, counts
placements    → IC1-IC4 with positions, rotations, enabled flags
```

### config.py Design
- Flat dataclasses with `slots=True` and `from_dict()` classmethods
- Primitives: XY, XYZ, AxisRange
- Sections: MachineConfig, ZHeights, CameraConfig, VacuumConfig, PasteConfig, BoardConfig, VisionConfig
- Components: Pad, PartDef, FeederConfig, Placement
- Root: NanoPnPConfig with convenience methods (feeder_for_part, enabled_placements)
- Loader: load_config(path) with ConfigError for validation

### run.py Design
- argparse with --config, --dry-run, --gui flags
- Subcommands: home, jog, paste, place, job
- Dispatch dict pattern, main() returns exit code

---

## Module 2: Board Parser

### Strategy: Adapt EDL Code
Reuse functions from ~/Desktop/EDL/ as private helpers in board_parser.py:
- `get_board_bounds_from_edge_cuts()` — board bounding box from Edge_Cuts Gerber
- `detect_pos_origin()` — auto-detect .pos-to-Gerber coordinate offset
- `parse_pos_files()` — parse .pos placement files
- `parse_all()` from parse_gerbonara — full SMD data with per-pad coordinates

### Public API
```python
def detect_prefix(gerber_dir: Path) -> str
def build_file_map(gerber_dir: Path, prefix: str) -> dict[str, Path]
def parse_board(gerber_dir: Path) -> tuple[list[Component], dict]
def transform_to_machine(components, origin_x, origin_y) -> list[Component]
def load_board(gerber_dir, origin_x=0, origin_y=0) -> tuple[list[Component], dict]
```

### Component dataclass
reference, part_id, package, value, machine_x, machine_y, rotation, side, pads[]

---

## Module 3: Serial Communication & Motion Control

### serial_comm.py — SerialConnection class
```python
connect(port, baud, timeout) -> str       # Open + init sequence (G21,G90,M82,M211 S0,M114)
send(cmd, timeout=20) -> str              # Send line, wait for ok/error
send_and_wait(cmd, timeout=60) -> str     # send + M400
get_position() -> dict[str,float]          # M114 + parse regex
close()                                    # Release port
```
- Thread-safe via threading.Lock on serial access
- Dry-run mode: log commands, return synthetic responses
- Position regex: `^X:(?P<X>...)\\s+Y:(?P<Y>...)\\s+Z:(?P<Z>...)\\s+E:(?P<E>...).*`

### motion.py — MotionController class
```python
home()                                     # M400 + G28
move_to(x,y,z,e,feedrate)                 # Raw G1 (only specified axes)
safe_move_to(x,y,z,e,feedrate)            # THE critical function:
                                           #   1. G1 Z{safe} + M400
                                           #   2. G1 X{x} Y{y} [E{e}] + M400
                                           #   3. G1 Z{z} + M400 (if z given)
vacuum_on()                                # M42 P4 S0 + dwell (tracks state)
vacuum_off()                               # M42 P4 S255 + dwell (only if on)
tool_select(n)                             # T{n} + M82/M83
get_position() -> dict                     # Proxy to serial
wait()                                     # M400
```

Key: safe_move_to NEVER combines Z+XY in same G1. This solves OpenPnP's diagonal motion bug.

---

## Module 4: Solder Paste Dispensing

### PasteDispenser class
```python
dispense_board(components) -> int          # Full job: T1 → dispense all → T0
dispense_component(comp) -> bool           # Dispatch by package substring match
dispense_soic8(cx, cy, rot)                # 2 lines: left row, right row
dispense_sot236(cx, cy, rot)               # 2 lines: left row, right row
dispense_lqfn16(cx, cy, rot)               # 4 lines + center EP dot
_draw_line(sx, sy, ex, ey)                 # safe_move to start → lower → extrude → raise
_dispense_dot(x, y, amount)                # safe_move → lower → extrude in place → raise
_rotate_offset(dx, dy, angle, cx, cy)      # 2D rotation: cos/sin transform + center
```

### Package Dispatch (substring match)
- "SOIC-8" / "SO08" → dispense_soic8
- "SOT-23" / "SOT-236" → dispense_sot236
- "LQFN-16" / "QFN-16" → dispense_lqfn16

### Paste Line Geometry
- SOIC8: rows at x=±2.7, y spans ±1.905 → line length 3.81mm
- SOT236: rows at x=±1.15, y spans ±0.95 → line length 1.9mm
- LQFN16: 4 sides at ±1.525, spans ±0.75 → line length 1.5mm each + EP dot

### Motion Pattern
_draw_line uses safe_move_to for travel, raw move_to for Z descent/extrusion/ascent.
Extrusion: E = line_length * extrude_per_mm (relative, M83 active)

---

## Module 5: Feeder Management

### Feeder dataclass
id, name, part_id, enabled, ref_hole(xyz), last_hole(xyz), pitch, tape_width,
rotation, feed_count, max_count, eia_481, rotation_in_tape

### FeederManager class
```python
get_feeder(part_id) -> Feeder              # First enabled feeder with remaining > 0
get_pick_position(feeder) -> (x,y,z)       # ref_hole + feed_count * pitch * direction
tape_direction(feeder) -> (dx,dy)          # Normalized vector ref→last hole
advance(feeder)                            # feed_count += 1
remaining(feeder) -> int                   # max_count - feed_count
get_pick_rotation(feeder) -> float         # rotation + EIA-481 compensation
save_state(path) / load_state(path)        # Persist feed_counts across restarts
```

### Math
- tape_direction = normalize(last_hole - ref_hole)
- pick_pos = ref_hole + feed_count * pitch * tape_direction
- F1 example: (18.7, 76.3, 23) + 0 * 8 * (0,1) = (18.7, 76.3, 23) ✓

---

## Module 6: Vision System

### OpenCV Pipeline (matches OpenPnP BVS_Stock)
1. Capture frame (with settle time + buffer flush)
2. GaussianBlur(k=9)
3. MaskCircle(d=525px) — crop to center ROI
4. BGR→HSV_FULL → MaskHsv(hue 60-130, sat 32-255, val 64-255) → mask
5. Threshold(100, binary)
6. FindContours(RETR_LIST) → FilterContours(min_area)
7. MinAreaRect on all points → center, size, angle

### VisionSystem class
```python
open() / close()                           # Camera lifecycle
capture() -> np.ndarray                    # Grab frame with settle time
detect_part(expected_w, expected_h) -> VisionResult   # Full pipeline
align_part(motion, part_config) -> AlignmentResult    # Multi-pass alignment loop
get_frame_for_preview() -> np.ndarray      # Non-blocking for GUI
calibrate_units_per_pixel(known, measured)  # Recalibration helper
compute_placement_correction(dx, dy, drot) # Static: rotation-aware XY correction
```

### Multi-Pass Alignment
1. Detect → get (dx, dy, dθ)
2. If converged (< 0.05mm, < 0.5°) → done
3. Move nozzle by -offset → re-detect → repeat (max 3 passes)
4. Reject if |linear| > 1.0mm or |angle| > 10°

### Correction Math
When nozzle rotates by -dθ to straighten part, offset vector also rotates:
```
corrected_dx = dx * cos(-dθ) - dy * sin(-dθ)
corrected_dy = dx * sin(-dθ) + dy * cos(-dθ)
```

---

## Module 7: Pick-and-Place Engine

### PnPEngine class
```python
run_job(components, paste_dispenser=None) -> JobResult
pick_and_place(component) -> PlacementResult
discard_part()                             # Move to discard, vacuum off
pause() / resume() / stop()               # Threading controls
get_drift_stats() -> dict                  # Accumulated error tracking
```

### Pick-and-Place Sequence (from PnP_TODO.md)
1. **Pick**: safe_move to feeder XY → lower to pick Z → vacuum ON → dwell → raise
2. **Pre-rotate**: G1 E{expected_angle}
3. **Align**: safe_move to camera (Z=part_height) → multi-pass vision → corrections
4. **Place**: safe_move to corrected XY+E → lower to board Z → vacuum OFF → dwell → raise
5. **Advance**: increment feeder count
6. **Log**: corrections, accumulated drift

### Error Handling
- Max 3 pick retries per component
- Failed pick → discard → retry from feeder
- Vision exceeds tolerance → discard → retry

### PlacementResult
reference, nominal_pos, corrected_pos, corrections, vision_passes, success, duration_s

---

## Module 8: Debug GUI (tkinter)

### Layout (3×2 grid)
```
┌───────────────────┬───────────────────┐
│ Connection        │ Position Display  │
│ [port] [connect]  │ X:0 Y:0 Z:0 E:0  │
├───────────────────┼───────────────────┤
│ Jog Controls      │ Camera View       │
│ XY pad, Z±, E±    │ [live preview]    │
│ Step: 0.1/1/10    │ [capture]         │
│ [Home] [GoTo]     │                   │
├───────────────────┼───────────────────┤
│ Vacuum & Tools    │ G-code Console    │
│ [VacON] [VacOFF]  │ > commands        │
│ [T0] [T1]         │ < responses       │
│ [Paste][Pick][Job]│ [entry] [send]    │
│ [Pause] [Stop]    │                   │
└───────────────────┴───────────────────┘
```

### Threading Strategy
- Main thread: tkinter mainloop, all widget updates
- Worker thread: serial commands via queue.Queue
- Camera preview: root.after(200ms) frame refresh (5 FPS)
- Position polling: root.after(500ms) M114 queries
- Job execution: separate thread with progress callback
- Communication: command queue → worker → result queue → main thread polls

### Design Principle
All GUI functions call the same module APIs as CLI. No GUI-only logic.
`python run.py --gui` for GUI, `python run.py <command>` for headless.

---

## Implementation Order
1. Module 1: Config + structure
2. Module 3: Serial + motion
3. Module 8: Debug GUI (jog/console/camera for hardware debugging)
4. Module 2: Board parser
5. Module 4: Solder paste
6. Module 5: Feeder manager
7. Module 6: Vision system
8. Module 7: PnP engine

## Dependencies
- pyserial (serial communication)
- opencv-python (vision pipeline)
- numpy (vision)
- Pillow (GUI camera display)
- gerbonara (Gerber parsing)
- tkinter (GUI, stdlib)
