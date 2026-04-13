"""Parse KiCad Gerber exports into component placement data.

Reads .pos placement files and (optionally) Gerber paste layers to produce
a list of Component objects with board-relative coordinates (bottom-left origin).

Coordinate chain:
    KiCad .pos   →  detect_pos_origin  →  bottom-left origin  →  machine coords
    (design coords)   (auto offset)       (0,0 at board corner)   (+ board.origin)

Adapted from ~/Desktop/EDL/ gerber_parser.py, parse_gerbonara.py, and
solder_paste_dispenser.py.
"""

from __future__ import annotations

import logging
import re
from dataclasses import dataclass, field, replace
from pathlib import Path

log = logging.getLogger(__name__)


# ── Exceptions ──────────────────────────────────────────────────


class BoardParserError(Exception):
    """Raised when Gerber/pos parsing fails."""


# ── Dataclasses ─────────────────────────────────────────────────


@dataclass(slots=True)
class PadInfo:
    """A single pad with absolute board-relative position."""
    pad_number: int | str
    x: float
    y: float
    width: float = 0.0
    height: float = 0.0
    aperture_type: str = ""
    net: str = ""


@dataclass(slots=True)
class Component:
    """A component parsed from KiCad Gerber output."""
    reference: str
    value: str
    package: str
    x: float
    y: float
    rotation: float
    side: str
    pads: list[PadInfo] = field(default_factory=list)


@dataclass(slots=True)
class BoardCorner:
    """Axis-aligned board bounding box in raw Gerber coordinates (mm).

    KiCad Gerber uses Y-down (pad Y values are negative). These fields
    store `abs(y)` for readability, so bigger `max_y_abs` means "further
    toward the bottom of the drawing" = further south on the board.

    The physical bottom-left corner of the PCB in raw (signed) Gerber
    space is `(min_x, -max_y_abs)`.
    """
    min_x: float
    max_x: float
    min_y_abs: float
    max_y_abs: float

    @property
    def width(self) -> float:
        return round(self.max_x - self.min_x, 3)

    @property
    def height(self) -> float:
        return round(self.max_y_abs - self.min_y_abs, 3)

    def shift_for_machine(self, origin_x: float, origin_y: float) -> tuple[float, float]:
        """Compute (shift_x, shift_y) so that a raw Gerber pad coordinate
        lands at `origin_xy` when the pad is at the bottom-left corner.

        Applied as:  machine_x = gerber_x + shift_x
                     machine_y = gerber_y + shift_y    (gerber_y is negative)

        Derivation:
            machine_x = (gerber_x - min_x)      + origin_x
                      = gerber_x + (-min_x + origin_x)
            machine_y = (max_y_abs - |gerber_y|) + origin_y
                      = (max_y_abs + gerber_y) + origin_y   [since gerber_y<0]
                      = gerber_y + (max_y_abs + origin_y)
        """
        return (-self.min_x + origin_x, self.max_y_abs + origin_y)

    def as_bounds_dict(self) -> dict:
        """Legacy dict form for the private helpers that still use it."""
        return {
            "min_x": self.min_x,
            "max_x": self.max_x,
            "min_y_abs": self.min_y_abs,
            "max_y_abs": self.max_y_abs,
        }


# ── Private helpers: coordinate math ────────────────────────────
# Adapted from ~/Desktop/EDL/gerber_parser.py


def _get_board_bounds(edge_cuts_path: Path) -> dict:
    """Parse Edge_Cuts Gerber for board bounding box.

    Returns {min_x, max_x, min_y_abs, max_y_abs} in mm.
    KiCad Gerber uses integer coords with 6 decimal places (format 4.6).
    Y coordinates are negative; we take abs for consistent math.
    """
    if not edge_cuts_path.exists():
        raise BoardParserError(f"Edge_Cuts not found: {edge_cuts_path}")

    xs: list[float] = []
    ys: list[float] = []
    with open(edge_cuts_path) as f:
        for line in f:
            m = re.match(r'X(-?\d+)Y(-?\d+)D0[12]\*', line.strip())
            if m:
                xs.append(int(m.group(1)) / 1e6)
                ys.append(abs(int(m.group(2)) / 1e6))

    if not xs or not ys:
        raise BoardParserError(f"No coordinates found in {edge_cuts_path.name}")

    return {
        "min_x": min(xs),
        "max_x": max(xs),
        "min_y_abs": min(ys),
        "max_y_abs": max(ys),
    }


def parse_edge_cuts(path: str | Path) -> BoardCorner:
    """Parse an Edge_Cuts.gbr profile layer into a bounding rectangle.

    Public wrapper around `_get_board_bounds` that returns the typed
    dataclass. Used by `paste_dispenser.dispense_paste()` to anchor pad
    coordinates to the physical board corner.
    """
    bounds = _get_board_bounds(Path(path))
    return BoardCorner(
        min_x=bounds["min_x"],
        max_x=bounds["max_x"],
        min_y_abs=bounds["min_y_abs"],
        max_y_abs=bounds["max_y_abs"],
    )


def _gerber_to_bl(x: float, y_abs: float, bounds: dict) -> tuple[float, float]:
    """Convert Gerber absolute coords to bottom-left origin.

    (0,0) at board bottom-left corner, Y increases upward.
    """
    bl_x = x - bounds["min_x"]
    bl_y = bounds["max_y_abs"] - y_abs
    return round(bl_x, 6), round(bl_y, 6)


def _pos_to_bl(
    pos_x: float, pos_y: float, bounds: dict, pos_origin: dict,
) -> tuple[float, float]:
    """Convert .pos file coords to bottom-left origin.

    Two-stage: .pos → Gerber absolute → bottom-left.
    pos_origin: {x_offset, y_sum} where
        gerber_x     = pos_x + x_offset
        gerber_y_abs = y_sum - pos_y
    """
    gerber_x = pos_x + pos_origin["x_offset"]
    gerber_y_abs = pos_origin["y_sum"] - pos_y
    return _gerber_to_bl(gerber_x, gerber_y_abs, bounds)


def _detect_pos_origin(file_map: dict[str, Path], bounds: dict) -> dict:
    """Auto-detect .pos-to-Gerber coordinate offset.

    Cross-references F_Paste pad centroids with .pos component positions
    to compute the transform parameters {x_offset, y_sum}.
    """
    fallback = {
        "x_offset": 0.0,
        "y_sum": bounds.get("max_y_abs", 0) + bounds.get("min_y_abs", 0),
    }

    paste_path = file_map.get("F_Paste")
    if not paste_path or not paste_path.exists():
        return fallback

    # Parse F_Paste for component pad centroids (raw Gerber coords)
    comp_pads: dict[str, list[tuple[float, float]]] = {}
    current_comp = None
    with open(paste_path) as f:
        for line in f:
            line = line.strip()
            m = re.match(r'%TO\.C,(.+)\*%', line)
            if m:
                current_comp = m.group(1)
                continue
            if line == '%TD*%':
                current_comp = None
                continue
            m = re.match(r'X(-?\d+)Y(-?\d+)D03\*', line)
            if m and current_comp:
                x = int(m.group(1)) / 1e6
                y = abs(int(m.group(2)) / 1e6)
                comp_pads.setdefault(current_comp, []).append((x, y))

    # Parse .pos files for component positions
    pos_comps: dict[str, tuple[float, float]] = {}
    for key in ("pos_top", "pos_bottom"):
        path = file_map.get(key)
        if not path or not path.exists():
            continue
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                parts = line.split()
                if len(parts) < 7:
                    continue
                pos_comps[parts[0]] = (float(parts[-4]), float(parts[-3]))

    # Compute offset by averaging across matched components
    x_offsets: list[float] = []
    y_sums: list[float] = []
    for ref, pads in comp_pads.items():
        if ref not in pos_comps:
            continue
        gerber_cx = sum(p[0] for p in pads) / len(pads)
        gerber_cy = sum(p[1] for p in pads) / len(pads)
        pos_x, pos_y = pos_comps[ref]
        x_offsets.append(gerber_cx - pos_x)
        y_sums.append(gerber_cy + pos_y)

    if x_offsets and y_sums:
        return {
            "x_offset": round(sum(x_offsets) / len(x_offsets), 6),
            "y_sum": round(sum(y_sums) / len(y_sums), 6),
        }

    return fallback


# ── Private helpers: file parsing ───────────────────────────────


def _parse_pos_files(
    file_map: dict[str, Path], bounds: dict, pos_origin: dict,
) -> list[dict]:
    """Parse KiCad .pos placement files with BL coordinate transform.

    Returns list of dicts: {reference, value, package, x, y, rotation, side}.
    """
    placements: list[dict] = []
    for key in ("pos_top", "pos_bottom"):
        path = file_map.get(key)
        if not path or not path.exists():
            continue
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                parts = line.split()
                if len(parts) < 7:
                    continue

                ref = parts[0]
                val = parts[1]
                # Package may contain spaces — everything between val and the
                # last 4 fields (PosX, PosY, Rot, Side)
                package = " ".join(parts[2:-4])
                pos_x = float(parts[-4])
                pos_y = float(parts[-3])
                rot = float(parts[-2])
                side = parts[-1].lower()

                bl_x, bl_y = _pos_to_bl(pos_x, pos_y, bounds, pos_origin)

                placements.append({
                    "reference": ref,
                    "value": val,
                    "package": package,
                    "x": bl_x,
                    "y": bl_y,
                    "rotation": rot,
                    "side": side,
                })
    return placements


def _parse_gbrjob(file_map: dict[str, Path]) -> dict | None:
    """Extract project metadata from .gbrjob file."""
    path = file_map.get("job")
    if not path or not path.exists():
        return None

    import json

    with open(path) as f:
        job = json.load(f)

    header = job.get("Header", {})
    gen_sw = header.get("GenerationSoftware", {})
    specs = job.get("GeneralSpecs", {})
    project = specs.get("ProjectId", {})
    size = specs.get("Size", {})

    return {
        "project_name": project.get("Name", ""),
        "revision": project.get("Revision", ""),
        "creation_date": header.get("CreationDate", ""),
        "generation_software": f"{gen_sw.get('Application', '')} {gen_sw.get('Version', '')}".strip(),
        "board_thickness_mm": specs.get("BoardThickness", 0),
        "layer_count": specs.get("LayerNumber", 0),
    }


# ── Private helpers: gerbonara pad enrichment ───────────────────


def _enrich_with_pad_data(
    components: list[Component],
    file_map: dict[str, Path],
    bounds: dict,
) -> bool:
    """Populate Component.pads from Gerber paste/copper layers via gerbonara.

    Returns True if pad data was successfully added, False otherwise.
    """
    try:
        from gerbonara.rs274x import GerberFile
        from gerbonara.graphic_objects import Flash
        from gerbonara.apertures import (
            CircleAperture, RectangleAperture, ObroundAperture,
            ApertureMacroInstance,
        )
    except ImportError:
        log.info("gerbonara not installed — skipping pad data enrichment")
        return False

    # Build ref→Component lookup
    comp_map = {c.reference: c for c in components}

    # ── Parse paste layers for pad positions ──
    paste_pads: dict[str, list[dict]] = {}  # ref → list of pad dicts

    for layer_name in ("F_Paste", "B_Paste"):
        path = file_map.get(layer_name)
        if not path or not path.exists():
            continue

        try:
            gf = GerberFile.open(str(path))
        except Exception as e:
            log.warning("Failed to parse %s: %s", layer_name, e)
            continue

        for obj in gf.objects:
            if not isinstance(obj, Flash):
                continue

            # Get component reference from attributes
            comp_ref = None
            if hasattr(obj, 'attrs') and obj.attrs:
                c_val = obj.attrs.get('.C')
                if c_val:
                    comp_ref = c_val[0] if isinstance(c_val, tuple) else str(c_val)

            if not comp_ref:
                continue

            bx, by = _gerber_to_bl(float(obj.x), abs(float(obj.y)), bounds)

            pad_data: dict = {"x": bx, "y": by}

            # Extract aperture dimensions
            ap = obj.aperture
            if ap:
                pad_data["aperture_type"] = type(ap).__name__
                if isinstance(ap, RectangleAperture):
                    pad_data["width"] = float(ap.w)
                    pad_data["height"] = float(ap.h)
                elif isinstance(ap, CircleAperture):
                    pad_data["width"] = float(ap.diameter)
                    pad_data["height"] = float(ap.diameter)
                elif isinstance(ap, ObroundAperture):
                    pad_data["width"] = float(ap.w)
                    pad_data["height"] = float(ap.h)
                elif isinstance(ap, ApertureMacroInstance):
                    pad_data["aperture_type"] = "ApertureMacroInstance"

            paste_pads.setdefault(comp_ref, []).append(pad_data)

    # ── Parse copper layers for net names ──
    pin_nets: dict[tuple[str, str], str] = {}  # (ref, pin) → net

    for layer_name in ("F_Cu", "B_Cu"):
        path = file_map.get(layer_name)
        if not path or not path.exists():
            continue
        try:
            gf = GerberFile.open(str(path))
        except Exception:
            continue

        for obj in gf.objects:
            if not isinstance(obj, Flash) or not hasattr(obj, 'attrs') or not obj.attrs:
                continue
            p_val = obj.attrs.get('.P')
            n_val = obj.attrs.get('.N')
            if p_val and isinstance(p_val, tuple) and len(p_val) >= 2 and n_val:
                comp_ref = p_val[0]
                pin = p_val[1]
                net = n_val[0] if isinstance(n_val, tuple) else str(n_val)
                pin_nets[(comp_ref, pin)] = net

    # ── Merge pad data into Components ──
    for ref, pads_raw in paste_pads.items():
        comp = comp_map.get(ref)
        if not comp:
            continue

        pads: list[PadInfo] = []
        for i, pd in enumerate(pads_raw, 1):
            net = pin_nets.get((ref, str(i)), "")
            pads.append(PadInfo(
                pad_number=i,
                x=pd["x"],
                y=pd["y"],
                width=pd.get("width", 0.0),
                height=pd.get("height", 0.0),
                aperture_type=pd.get("aperture_type", ""),
                net=net,
            ))

        comp.pads = pads

    return bool(paste_pads)


# ── Public API ──────────────────────────────────────────────────


def detect_prefix(gerber_dir: str | Path) -> str:
    """Scan Gerber directory for *-top.pos / *-bottom.pos and extract board prefix.

    The prefix includes the trailing dash, e.g. "U6 Pico Board-".
    """
    gerber_dir = Path(gerber_dir)
    if not gerber_dir.is_dir():
        raise BoardParserError(f"Not a directory: {gerber_dir}")

    for suffix in ("-top.pos", "-bottom.pos"):
        for match in gerber_dir.glob(f"*{suffix}"):
            # Prefix is everything before -top.pos / -bottom.pos,
            # including the trailing dash.  e.g. "Board-top.pos" → "Board-"
            return match.name[: -len(suffix) + 1]

    raise BoardParserError(f"No *-top.pos or *-bottom.pos found in {gerber_dir}")


def build_file_map(gerber_dir: str | Path, prefix: str) -> dict[str, Path]:
    """Build layer-name → filepath dict for standard KiCad Gerber output."""
    d = Path(gerber_dir)
    return {
        "F_Cu":         d / f"{prefix}F_Cu.gbr",
        "B_Cu":         d / f"{prefix}B_Cu.gbr",
        "F_Silkscreen": d / f"{prefix}F_Silkscreen.gbr",
        "B_Silkscreen": d / f"{prefix}B_Silkscreen.gbr",
        "F_Mask":       d / f"{prefix}F_Mask.gbr",
        "B_Mask":       d / f"{prefix}B_Mask.gbr",
        "F_Paste":      d / f"{prefix}F_Paste.gbr",
        "B_Paste":      d / f"{prefix}B_Paste.gbr",
        "Edge_Cuts":    d / f"{prefix}Edge_Cuts.gbr",
        "PTH":          d / f"{prefix}PTH.drl",
        "NPTH":         d / f"{prefix}NPTH.drl",
        "job":          d / f"{prefix}job.gbrjob",
        "pos_top":      d / f"{prefix}top.pos",
        "pos_bottom":   d / f"{prefix}bottom.pos",
    }


def parse_board(gerber_dir: str | Path) -> tuple[list[Component], dict]:
    """Parse a KiCad Gerber directory into components with board-relative coords.

    Returns (components, board_info) where coordinates use bottom-left origin.
    Tries gerbonara for pad data; falls back to pos-only if unavailable.
    """
    gerber_dir = Path(gerber_dir)

    # 1. Detect board prefix and build file map
    prefix = detect_prefix(gerber_dir)
    file_map = build_file_map(gerber_dir, prefix)

    # 2. Get board bounds from Edge_Cuts
    edge_cuts = file_map["Edge_Cuts"]
    bounds = _get_board_bounds(edge_cuts)

    # 3. Auto-detect .pos origin offset
    pos_origin = _detect_pos_origin(file_map, bounds)

    # 4. Parse .pos files into placement dicts
    raw = _parse_pos_files(file_map, bounds, pos_origin)
    if not raw:
        raise BoardParserError(f"No components found in .pos files in {gerber_dir}")

    # 5. Build Component objects
    components = [
        Component(
            reference=p["reference"],
            value=p["value"],
            package=p["package"],
            x=p["x"],
            y=p["y"],
            rotation=p["rotation"],
            side=p["side"],
        )
        for p in raw
    ]

    # 6. Enrich with pad data from Gerber layers (optional)
    has_pads = _enrich_with_pad_data(components, file_map, bounds)

    # 7. Build board info
    width = round(bounds["max_x"] - bounds["min_x"], 3)
    height = round(bounds["max_y_abs"] - bounds["min_y_abs"], 3)

    board_info: dict = {
        "width_mm": width,
        "height_mm": height,
        "prefix": prefix,
        "coordinate_origin": "bottom-left",
        "has_pad_data": has_pads,
        "component_count": len(components),
        "top_count": sum(1 for c in components if c.side == "top"),
        "bottom_count": sum(1 for c in components if c.side == "bottom"),
        "metadata": _parse_gbrjob(file_map),
    }

    return components, board_info


def transform_to_machine(
    components: list[Component],
    origin_x: float,
    origin_y: float,
) -> list[Component]:
    """Shift coordinates from board-relative to machine coordinates.

    Returns a new list — does not mutate the originals.
    machine_x = board_x + origin_x
    machine_y = board_y + origin_y
    """
    result: list[Component] = []
    for c in components:
        shifted_pads = [
            PadInfo(
                pad_number=p.pad_number,
                x=round(p.x + origin_x, 6),
                y=round(p.y + origin_y, 6),
                width=p.width,
                height=p.height,
                aperture_type=p.aperture_type,
                net=p.net,
            )
            for p in c.pads
        ]
        result.append(replace(
            c,
            x=round(c.x + origin_x, 6),
            y=round(c.y + origin_y, 6),
            pads=shifted_pads,
        ))
    return result


def load_board(
    gerber_dir: str | Path,
    origin_x: float = 0.0,
    origin_y: float = 0.0,
) -> tuple[list[Component], dict]:
    """Parse board and transform to machine coordinates in one call.

    Convenience wrapper: parse_board() + transform_to_machine().
    """
    components, board_info = parse_board(gerber_dir)
    if origin_x != 0.0 or origin_y != 0.0:
        components = transform_to_machine(components, origin_x, origin_y)
    return components, board_info


def load_placements_from_pos(
    pos_path: str | Path,
    origin_xy: tuple[float, float],
    edge_cuts_path: str | Path | None = None,
    paste_path: str | Path | None = None,
    package_map: dict[str, str] | None = None,
    skip_refs_prefixes: list[str] | None = None,
) -> list["Placement"]:
    """Parse a .pos file into `Placement` objects in machine coordinates.

    Two modes, picked automatically by which optional files are supplied:

    1. **Full auto-detect** (pos + edge_cuts + paste):
       Uses `_detect_pos_origin` to cross-reference F_Paste pad centroids
       with .pos component positions, deriving the KiCad page-origin
       offset. Produces bottom-left-relative coords, then adds `origin_xy`.

    2. **Bottom-left convention** (pos only, or missing any of the other
       two files): Trusts that the .pos coordinates are already
       board-relative (KiCad drill/place origin was set to the PCB
       corner). Adds `origin_xy` directly.

    Only top-side placements are returned by default, with rotation and
    part_id derived from the `Package` column of the .pos file.

    Raises `BoardParserError` if `pos_path` is unreadable or empty.
    """
    from nanopnp.config import Placement  # local to avoid circular import

    pos_path = Path(pos_path)
    if not pos_path.exists():
        raise BoardParserError(f".pos file not found: {pos_path}")

    origin_x, origin_y = origin_xy

    file_map: dict[str, Path] = {"pos_top": pos_path}
    raw: list[dict]

    have_both = edge_cuts_path is not None and paste_path is not None
    if have_both:
        # Mode 1: full auto-detect
        ep = Path(edge_cuts_path)
        pp = Path(paste_path)
        if not ep.exists() or not pp.exists():
            raise BoardParserError(
                f"Missing auxiliary files: {ep.name}, {pp.name}"
            )
        file_map["Edge_Cuts"] = ep
        file_map["F_Paste"] = pp
        bounds = _get_board_bounds(ep)
        pos_origin = _detect_pos_origin(file_map, bounds)
        raw = _parse_pos_files(file_map, bounds, pos_origin)
        log.info(
            "[place] auto-detect origin: x_offset=%.3f y_sum=%.3f",
            pos_origin["x_offset"], pos_origin["y_sum"],
        )
    else:
        # Mode 2: trust bottom-left convention — skip the transform,
        # read raw .pos coords and treat them as already board-relative.
        raw = _parse_pos_raw(pos_path)
        log.info(
            "[place] .pos-only mode (no Edge_Cuts/F_Paste) — trusting "
            "bottom-left convention"
        )

    if not raw:
        raise BoardParserError(f"No placements parsed from {pos_path.name}")

    pkg_map = package_map or {}
    skip_prefixes = tuple(skip_refs_prefixes or [])

    placements: list[Placement] = []
    skipped_refs: list[str] = []
    for p in raw:
        if p.get("side", "top") != "top":
            continue

        ref = p["reference"]
        # Skip fiducials, test points, mounting holes — anything the
        # user's config flags as "not pickable".
        if skip_prefixes and ref.startswith(skip_prefixes):
            skipped_refs.append(ref)
            continue

        # Translate the KiCad package string to an abstract feeder
        # part_id via substring match. First matching substring wins.
        # No match → pass through the raw package name (the engine will
        # then fail with a clear "No feeder for 'X'" which is the right
        # signal to add a new mapping entry).
        part_id = p["package"]
        for pattern, abstract_id in pkg_map.items():
            if pattern in part_id:
                part_id = abstract_id
                break

        placements.append(Placement(
            ref=ref,
            part_id=part_id,
            x=round(p["x"] + origin_x, 6),
            y=round(p["y"] + origin_y, 6),
            rotation=p["rotation"],
            enabled=True,
        ))

    if skipped_refs:
        log.info("[place] skipped %d non-pickable refs: %s",
                 len(skipped_refs), ", ".join(skipped_refs[:8]) + ("..." if len(skipped_refs) > 8 else ""))
    log.info("[place] loaded %d top-side placements from %s",
             len(placements), pos_path.name)
    return placements


def _parse_pos_raw(pos_path: Path) -> list[dict]:
    """Parse a .pos file without any coordinate transform.

    Used by `load_placements_from_pos` when no Edge_Cuts/F_Paste are
    available — the caller is responsible for adding any machine-space
    offset.
    """
    rows: list[dict] = []
    with open(pos_path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 7:
                continue
            rows.append({
                "reference": parts[0],
                "value": parts[1],
                "package": " ".join(parts[2:-4]),
                "x": float(parts[-4]),
                "y": float(parts[-3]),
                "rotation": float(parts[-2]),
                "side": parts[-1].lower(),
            })
    return rows


def write_pos_file(path: str | Path, rows: list[dict]) -> None:
    """Write a KiCad-compatible .pos file.

    Each row is a dict with keys: reference, value, package, x, y, rotation, side.
    The output preserves KiCad's ASCII header format so it round-trips
    through `_parse_pos_raw`. Column alignment is simpler than KiCad's
    original (single-space separated) — the parser tolerates either.

    Only top-side rows are emitted by default if no side is given.
    """
    from datetime import datetime

    path = Path(path)
    sides = sorted({r.get("side", "top").lower() for r in rows})
    side_line = "top" if sides == ["top"] else ("bottom" if sides == ["bottom"] else "top & bottom")

    header = [
        f"### Footprint positions - written by NanoPnP on {datetime.now().isoformat(timespec='seconds')} ###",
        "### (round-trip compatible with KiCad .pos format)",
        "## Unit = mm, Angle = deg.",
        f"## Side : {side_line}",
        "# Ref     Val       Package                                           PosX       PosY       Rot  Side",
    ]

    lines = list(header)
    for r in rows:
        ref = r["reference"]
        val = r.get("value", "")
        pkg = r.get("package", "")
        x = float(r["x"])
        y = float(r["y"])
        rot = float(r.get("rotation", 0.0))
        side = r.get("side", "top").lower()
        lines.append(
            f"{ref:<9} {val:<9} {pkg:<49} {x:9.4f}  {y:9.4f}  {rot:8.4f}  {side}"
        )
    lines.append("## End")

    path.write_text("\n".join(lines) + "\n")
    log.info("Wrote %d placements to %s", len(rows), path.name)
