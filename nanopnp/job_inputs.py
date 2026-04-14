"""Input-file bundle for a NanoPnP job.

A job is defined by up to three files:
    Edge_Cuts.gbr  — board outline (corner reference for paste dispensing)
    F_Paste.gbr    — per-component pad stencil (what to dispense)
    top.pos        — component centroids + rotations (where to place)

Paste mode needs edge_cuts + paste.
Place mode needs pos.
Full mode needs all three.

The same bundle is used by the GUI, the headless CLI, and the Jetson watcher,
so the classification logic lives here once.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

JobMode = Literal["paste", "place", "full", "none"]


@dataclass(frozen=True, slots=True)
class JobInputs:
    edge_cuts: Path | None = None
    paste: Path | None = None
    pos: Path | None = None

    @property
    def can_paste(self) -> bool:
        return self.paste is not None

    @property
    def can_place(self) -> bool:
        return self.pos is not None

    @property
    def mode(self) -> JobMode:
        if self.can_paste and self.can_place:
            return "full"
        if self.can_paste:
            return "paste"
        if self.can_place:
            return "place"
        return "none"

    def as_list(self) -> list[Path]:
        return [p for p in (self.edge_cuts, self.paste, self.pos) if p is not None]


# Gerber X2 file-function markers. Only need the first ~1 KB of the file to
# find these — we never read the whole Gerber just to classify it.
_RE_EDGE_CUTS = re.compile(r"%TF\.FileFunction,Profile[,*]")
_RE_F_PASTE = re.compile(r"%TF\.FileFunction,Paste,Top[,*]")


def _sniff_gerber(path: Path) -> str | None:
    """Return 'edge_cuts' / 'paste' / None for a .gbr path.

    Reads at most 2 KB — KiCad puts %TF headers at the top of the file.
    """
    try:
        with open(path, "r", errors="replace") as f:
            head = f.read(2048)
    except OSError:
        return None

    if _RE_EDGE_CUTS.search(head):
        return "edge_cuts"
    if _RE_F_PASTE.search(head):
        return "paste"
    return None


def classify(paths: list[Path] | list[str]) -> JobInputs:
    """Build a JobInputs from an unordered set of file paths.

    Silently ignores unrecognized files (wrong extension, wrong Gerber type).
    If multiple paths match the same slot, the last one wins — callers that
    care about ambiguity should detect duplicates before calling.
    """
    edge_cuts: Path | None = None
    paste: Path | None = None
    pos: Path | None = None

    for raw in paths:
        p = Path(raw)
        if not p.exists():
            continue

        suffix = p.suffix.lower()
        if suffix == ".pos":
            pos = p
            continue
        if suffix == ".gbr":
            kind = _sniff_gerber(p)
            if kind == "edge_cuts":
                edge_cuts = p
            elif kind == "paste":
                paste = p

    return JobInputs(edge_cuts=edge_cuts, paste=paste, pos=pos)
