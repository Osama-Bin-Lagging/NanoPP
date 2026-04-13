"""NanoPnP configuration loader.

All coordinates in mm. All feedrates in mm/min. All times in ms.
Z axis: 0 = safe/top, higher values = physically lower toward board.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path


# ── Primitives ────────────────────────────────────────────────

@dataclass(slots=True)
class XY:
    x: float
    y: float


@dataclass(slots=True)
class XYZ:
    x: float
    y: float
    z: float


@dataclass(slots=True)
class AxisRange:
    min: float
    max: float


@dataclass(slots=True)
class AxisLimits:
    x: AxisRange
    y: AxisRange

    @classmethod
    def from_dict(cls, d: dict) -> AxisLimits:
        return cls(x=AxisRange(**d["x"]), y=AxisRange(**d["y"]))


# ── Top-level sections ───────────────────────────────────────

@dataclass(slots=True)
class MachineConfig:
    serial_port: str
    baud_rate: int
    axis_limits: AxisLimits
    travel_feedrate: float
    z_feedrate: float
    disable_software_endstops: bool

    @classmethod
    def from_dict(cls, d: dict) -> MachineConfig:
        return cls(
            serial_port=d["serial_port"],
            baud_rate=d["baud_rate"],
            axis_limits=AxisLimits.from_dict(d["axis_limits"]),
            travel_feedrate=d["travel_feedrate"],
            z_feedrate=d.get("z_feedrate", 500),
            disable_software_endstops=d.get("disable_software_endstops", True),
        )


@dataclass(slots=True)
class ZHeights:
    safe: float
    board_surface: float
    feeder_pick: float
    discard: float
    retract: float = 30.0  # intermediate Z after pick / during vision

    @classmethod
    def from_dict(cls, d: dict) -> ZHeights:
        return cls(
            safe=d["safe"],
            board_surface=d["board_surface"],
            feeder_pick=d["feeder_pick"],
            discard=d["discard"],
            retract=d.get("retract", 30.0),
        )


@dataclass(slots=True)
class CameraConfig:
    device_index: int
    position: XY
    units_per_pixel: XY
    settle_time_ms: int

    @classmethod
    def from_dict(cls, d: dict) -> CameraConfig:
        return cls(
            device_index=d["device_index"],
            position=XY(**d["position"]),
            units_per_pixel=XY(**d["units_per_pixel"]),
            settle_time_ms=d["settle_time_ms"],
        )


@dataclass(slots=True)
class VacuumConfig:
    on_cmd: str
    off_cmd: str
    pick_dwell_ms: int
    place_dwell_ms: int

    @classmethod
    def from_dict(cls, d: dict) -> VacuumConfig:
        return cls(**d)


@dataclass(slots=True)
class PasteConfig:
    feed_xy: float          # XY travel feedrate (mm/min)
    feed_xy_slow: float     # XY feedrate during dispense
    feed_z: float           # Z feedrate
    feed_e0: float          # extruder 0 (nozzle) max feedrate
    feed_paste: float       # extruder 1 (paste) feedrate
    z_travel: float         # safe Z above work surface
    e_prime: float          # paste prime push before first row
    e_rate: float           # E units per row
    e_retract: float        # E retract after each row

    @classmethod
    def from_dict(cls, d: dict) -> PasteConfig:
        return cls(
            feed_xy=d.get("feed_xy", d.get("travel_feedrate", 2000)),
            feed_xy_slow=d.get("feed_xy_slow", d.get("dispense_feedrate", 2000)),
            feed_z=d.get("feed_z", 2000),
            feed_e0=d.get("feed_e0", 160),
            feed_paste=d.get("feed_paste", 6000),
            z_travel=d.get("z_travel", d.get("safe_z", 5.0)),
            e_prime=d.get("e_prime", 1000),
            e_rate=d.get("e_rate", d.get("extrude_per_mm", 400)),
            e_retract=d.get("e_retract", 50),
        )


@dataclass(slots=True)
class BoardConfig:
    origin: XY             # machine XY of the PCB's bottom-left corner
    first_pad_offset: XY   # distance from board corner to the corner-most pad
    place_z: float
    # Ordered list of (substring, part_id) pairs — first substring found
    # in a .pos `Package` column wins. Lets you map KiCad library names
    # like "SOIC-8_3.9x4.9mm_P1.27mm" → your abstract feeder part id
    # "IC_SOIC8" without touching code. Empty dict = passthrough.
    package_map: dict[str, str] = field(default_factory=dict)
    # Reference-designator prefixes to skip when loading a .pos file.
    # Defaults match the common KiCad conventions for non-pickable items:
    # FID = fiducials, H = mounting holes, TP = test points, MH = mounts.
    skip_refs_prefixes: list[str] = field(default_factory=lambda: ["FID", "TP", "MH", "H"])

    @classmethod
    def from_dict(cls, d: dict) -> BoardConfig:
        return cls(
            origin=XY(**d["origin"]),
            first_pad_offset=XY(**d.get("first_pad_offset", {"x": 0.0, "y": 0.0})),
            place_z=d["place_z"],
            package_map=dict(d.get("package_map", {})),
            skip_refs_prefixes=list(d.get("skip_refs_prefixes", ["FID", "TP", "MH", "H"])),
        )


@dataclass(slots=True)
class ControllerConfig:
    """External joystick/button controller for manual mode.

    Optional — absent from config.json means `run.py manual` requires
    --controller-port on the command line.
    """
    port: str = ""
    baud: int = 115200
    dead_low: int = 1900
    dead_high: int = 2150
    joy_max: int = 4095
    max_step_xy: float = 2.0
    max_step_yaw: float = 1.0
    fine_xy: float = 0.2
    fine_z: float = 2.0
    fine_yaw: float = 0.5
    feed_rate: int = 1000
    feed_rate_slow: int = 200

    @classmethod
    def from_dict(cls, d: dict) -> ControllerConfig:
        return cls(**{k: v for k, v in d.items() if k in cls.__slots__})

    def is_configured(self) -> bool:
        return bool(self.port)


@dataclass(slots=True)
class RemoteConfig:
    """Jetson target for the GUI's Target selector.

    Optional — absent from config.json means the GUI hides the selector
    and runs everything locally. The watcher on the Jetson does NOT read
    this (it's a laptop-side concern only).
    """
    host: str = ""
    user: str = ""
    incoming_path: str = "~/nanopnp/incoming"
    status_path: str = "~/nanopnp/status.json"
    ssh_key: str = ""         # "" = default identity / ssh-agent
    port: int = 22

    @classmethod
    def from_dict(cls, d: dict) -> RemoteConfig:
        return cls(
            host=d.get("host", ""),
            user=d.get("user", ""),
            incoming_path=d.get("incoming_path", "~/nanopnp/incoming"),
            status_path=d.get("status_path", "~/nanopnp/status.json"),
            ssh_key=d.get("ssh_key", ""),
            port=d.get("port", 22),
        )

    def is_configured(self) -> bool:
        return bool(self.host and self.user)


@dataclass(slots=True)
class HSVRange:
    hue_min: int
    hue_max: int
    sat_min: int
    sat_max: int
    val_min: int
    val_max: int


@dataclass(slots=True)
class VisionConfig:
    max_passes: int
    max_linear_offset_mm: float
    max_angular_offset_deg: float
    hsv: HSVRange
    threshold: int
    blur_kernel: int
    min_contour_area: float
    mask_diameter: int
    converge_mm: float
    converge_deg: float

    @classmethod
    def from_dict(cls, d: dict) -> VisionConfig:
        return cls(
            max_passes=d["max_passes"],
            max_linear_offset_mm=d["max_linear_offset_mm"],
            max_angular_offset_deg=d["max_angular_offset_deg"],
            hsv=HSVRange(**d["hsv"]),
            threshold=d["threshold"],
            blur_kernel=d["blur_kernel"],
            min_contour_area=d["min_contour_area"],
            mask_diameter=d["mask_diameter"],
            converge_mm=d.get("converge_mm", 0.05),
            converge_deg=d.get("converge_deg", 0.5),
        )


# ── Component data ───────────────────────────────────────────

@dataclass(slots=True)
class Pad:
    id: int | str
    x: float
    y: float
    width: float
    height: float


@dataclass(slots=True)
class PartDef:
    name: str
    height: float
    body_width: float
    body_length: float
    pads: list[Pad]

    @classmethod
    def from_dict(cls, name: str, d: dict) -> PartDef:
        return cls(
            name=name,
            height=d["height"],
            body_width=d["body"]["width"],
            body_length=d["body"]["length"],
            pads=[Pad(**p) for p in d["pads"]],
        )


@dataclass(slots=True)
class FeederConfig:
    slot: str
    id: str
    part_id: str
    enabled: bool
    ref_hole: XYZ
    last_hole: XYZ
    pitch: float
    tape_width: float
    rotation: float
    feed_count: int
    max_count: int

    @classmethod
    def from_dict(cls, slot: str, d: dict) -> FeederConfig:
        return cls(
            slot=slot,
            id=d["id"],
            part_id=d["part_id"],
            enabled=d["enabled"],
            ref_hole=XYZ(**d["ref_hole"]),
            last_hole=XYZ(**d["last_hole"]),
            pitch=d["pitch"],
            tape_width=d["tape_width"],
            rotation=d["rotation"],
            feed_count=d["feed_count"],
            max_count=d["max_count"],
        )


@dataclass(slots=True)
class Placement:
    ref: str
    part_id: str
    x: float
    y: float
    rotation: float
    enabled: bool

    @classmethod
    def from_dict(cls, d: dict) -> Placement:
        return cls(**d)


# ── Root config ──────────────────────────────────────────────

@dataclass(slots=True)
class NanoPnPConfig:
    machine: MachineConfig
    z_heights: ZHeights
    camera: CameraConfig
    vacuum: VacuumConfig
    paste_dispensing: PasteConfig
    board: BoardConfig
    vision: VisionConfig
    parts: dict[str, PartDef]
    feeders: dict[str, FeederConfig]
    placements: list[Placement]
    controller: ControllerConfig = field(default_factory=ControllerConfig)
    remote: RemoteConfig = field(default_factory=RemoteConfig)

    def feeder_for_part(self, part_id: str) -> FeederConfig | None:
        for f in self.feeders.values():
            if f.part_id == part_id and f.enabled:
                return f
        return None

    def enabled_placements(self) -> list[Placement]:
        return [p for p in self.placements if p.enabled]


# ── Loader ───────────────────────────────────────────────────

_REQUIRED_SECTIONS = [
    "machine", "z_heights", "camera", "vacuum",
    "paste_dispensing", "board", "vision", "parts",
    "feeders", "placements",
]


class ConfigError(Exception):
    pass


def load_config(path: str | Path = "config.json") -> NanoPnPConfig:
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"Config file not found: {path}")

    try:
        raw = json.loads(path.read_text())
    except json.JSONDecodeError as e:
        raise ConfigError(f"Invalid JSON in {path}: {e}") from e

    missing = [s for s in _REQUIRED_SECTIONS if s not in raw]
    if missing:
        raise ConfigError(f"Missing config sections: {', '.join(missing)}")

    try:
        return NanoPnPConfig(
            machine=MachineConfig.from_dict(raw["machine"]),
            z_heights=ZHeights.from_dict(raw["z_heights"]),
            camera=CameraConfig.from_dict(raw["camera"]),
            vacuum=VacuumConfig.from_dict(raw["vacuum"]),
            paste_dispensing=PasteConfig.from_dict(raw["paste_dispensing"]),
            board=BoardConfig.from_dict(raw["board"]),
            vision=VisionConfig.from_dict(raw["vision"]),
            parts={
                name: PartDef.from_dict(name, pdata)
                for name, pdata in raw["parts"].items()
            },
            feeders={
                slot: FeederConfig.from_dict(slot, fdata)
                for slot, fdata in raw["feeders"].items()
            },
            placements=[Placement.from_dict(p) for p in raw["placements"]],
            controller=ControllerConfig.from_dict(raw.get("controller", {})),
            remote=RemoteConfig.from_dict(raw.get("remote", {})),
        )
    except (KeyError, TypeError) as e:
        raise ConfigError(f"Config structure error: {e}") from e


def save_config(config: NanoPnPConfig, path: str | Path = "config.json") -> None:
    """Serialize NanoPnPConfig back to JSON."""
    raw = {
        "machine": {
            "serial_port": config.machine.serial_port,
            "baud_rate": config.machine.baud_rate,
            "axis_limits": {
                "x": {"min": config.machine.axis_limits.x.min, "max": config.machine.axis_limits.x.max},
                "y": {"min": config.machine.axis_limits.y.min, "max": config.machine.axis_limits.y.max},
            },
            "travel_feedrate": config.machine.travel_feedrate,
            "z_feedrate": config.machine.z_feedrate,
            "disable_software_endstops": config.machine.disable_software_endstops,
        },
        "z_heights": {
            "safe": config.z_heights.safe,
            "board_surface": config.z_heights.board_surface,
            "feeder_pick": config.z_heights.feeder_pick,
            "discard": config.z_heights.discard,
            "retract": config.z_heights.retract,
        },
        "camera": {
            "device_index": config.camera.device_index,
            "position": {"x": config.camera.position.x, "y": config.camera.position.y},
            "units_per_pixel": {"x": config.camera.units_per_pixel.x, "y": config.camera.units_per_pixel.y},
            "settle_time_ms": config.camera.settle_time_ms,
        },
        "vacuum": {
            "on_cmd": config.vacuum.on_cmd,
            "off_cmd": config.vacuum.off_cmd,
            "pick_dwell_ms": config.vacuum.pick_dwell_ms,
            "place_dwell_ms": config.vacuum.place_dwell_ms,
        },
        "paste_dispensing": {
            "feed_xy": config.paste_dispensing.feed_xy,
            "feed_xy_slow": config.paste_dispensing.feed_xy_slow,
            "feed_z": config.paste_dispensing.feed_z,
            "feed_e0": config.paste_dispensing.feed_e0,
            "feed_paste": config.paste_dispensing.feed_paste,
            "z_travel": config.paste_dispensing.z_travel,
            "e_prime": config.paste_dispensing.e_prime,
            "e_rate": config.paste_dispensing.e_rate,
            "e_retract": config.paste_dispensing.e_retract,
        },
        "board": {
            "origin": {"x": config.board.origin.x, "y": config.board.origin.y},
            "first_pad_offset": {
                "x": config.board.first_pad_offset.x,
                "y": config.board.first_pad_offset.y,
            },
            "place_z": config.board.place_z,
            "package_map": dict(config.board.package_map),
            "skip_refs_prefixes": list(config.board.skip_refs_prefixes),
        },
        "vision": {
            "max_passes": config.vision.max_passes,
            "max_linear_offset_mm": config.vision.max_linear_offset_mm,
            "max_angular_offset_deg": config.vision.max_angular_offset_deg,
            "converge_mm": config.vision.converge_mm,
            "converge_deg": config.vision.converge_deg,
            "hsv": {
                "hue_min": config.vision.hsv.hue_min, "hue_max": config.vision.hsv.hue_max,
                "sat_min": config.vision.hsv.sat_min, "sat_max": config.vision.hsv.sat_max,
                "val_min": config.vision.hsv.val_min, "val_max": config.vision.hsv.val_max,
            },
            "threshold": config.vision.threshold,
            "blur_kernel": config.vision.blur_kernel,
            "min_contour_area": config.vision.min_contour_area,
            "mask_diameter": config.vision.mask_diameter,
        },
        "parts": {
            name: {
                "height": p.height,
                "body": {"width": p.body_width, "length": p.body_length},
                "pads": [{"id": pad.id, "x": pad.x, "y": pad.y, "width": pad.width, "height": pad.height}
                         for pad in p.pads],
            }
            for name, p in config.parts.items()
        },
        "feeders": {
            slot: {
                "id": f.id, "part_id": f.part_id, "enabled": f.enabled,
                "ref_hole": {"x": f.ref_hole.x, "y": f.ref_hole.y, "z": f.ref_hole.z},
                "last_hole": {"x": f.last_hole.x, "y": f.last_hole.y, "z": f.last_hole.z},
                "pitch": f.pitch, "tape_width": f.tape_width, "rotation": f.rotation,
                "feed_count": f.feed_count, "max_count": f.max_count,
            }
            for slot, f in config.feeders.items()
        },
        "placements": [
            {"ref": p.ref, "part_id": p.part_id, "x": p.x, "y": p.y,
             "rotation": p.rotation, "enabled": p.enabled}
            for p in config.placements
        ],
    }
    if config.controller.is_configured():
        raw["controller"] = {
            "port": config.controller.port,
            "baud": config.controller.baud,
            "dead_low": config.controller.dead_low,
            "dead_high": config.controller.dead_high,
            "joy_max": config.controller.joy_max,
            "max_step_xy": config.controller.max_step_xy,
            "max_step_yaw": config.controller.max_step_yaw,
            "fine_xy": config.controller.fine_xy,
            "fine_z": config.controller.fine_z,
            "fine_yaw": config.controller.fine_yaw,
            "feed_rate": config.controller.feed_rate,
            "feed_rate_slow": config.controller.feed_rate_slow,
        }
    if config.remote.is_configured():
        raw["remote"] = {
            "host": config.remote.host,
            "user": config.remote.user,
            "incoming_path": config.remote.incoming_path,
            "status_path": config.remote.status_path,
            "ssh_key": config.remote.ssh_key,
            "port": config.remote.port,
        }
    Path(path).write_text(json.dumps(raw, indent=2) + "\n")
