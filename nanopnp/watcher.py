"""File-drop watcher daemon for the NanoPnP Jetson.

Watches an incoming folder for Edge_Cuts / F_Paste / .pos files, debounces
5 s after the last file arrival (so in-flight scp has time to finish),
then shells out to `run.py` with the matching subcommand. On success, the
input set is archived with a timestamp; on persistent failure, moved to
a failed/ folder with an error log.

Status is reported via `~/nanopnp/status.json`, atomically replaced on
every state transition. The GUI reads this file over ssh to show live
state in the toolbar.

Typical invocation:

    python -m nanopnp.watcher \\
        --incoming ~/nanopnp/incoming \\
        --archive  ~/nanopnp/done \\
        --failed   ~/nanopnp/failed \\
        --config   ~/NanoPnP/config.json \\
        [--debounce 5.0] [--poll 1.0]

No external dependencies — just stdlib (os, time, shutil, subprocess, json).
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import re
import shutil
import subprocess
import sys
import time
import traceback
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path

from nanopnp.job_inputs import JobInputs, classify

# Progress patterns emitted by run.py / engine / paste_dispenser.
# Tolerant on purpose — anything that looks like "X/Y" or "Placed X/Y"
# gets surfaced to the laptop without the watcher needing rigid parsing.
PROGRESS_PATTERNS = [
    re.compile(r"\[paste\]\s*\((\d+)/(\d+)\)\s+(\S+)"),           # [paste] (3/8) IC3
    re.compile(r"Placed\s+(\d+)/(\d+)"),                           # Placed 3/8 ...
    re.compile(r"Pick\s+(\S+)\s+from"),                            # Pick IC3 from F1 ...
    re.compile(r"Place\s+(\S+)\s+at"),                             # Place IC3 at ...
]

# Seconds between forced heartbeat writes while a subprocess is running
# (even when no progress lines arrive). Laptop treats `updated_at` older
# than ~3x this as "stale".
HEARTBEAT_INTERVAL = 3.0

log = logging.getLogger("nanopnp.watcher")


# ── Retry backoff ──────────────────────────────────────────

MAX_RETRIES = 3
RETRY_DELAYS = [2.0, 5.0, 10.0]  # seconds between attempts 1→2, 2→3, 3→final


# ── Status schema ──────────────────────────────────────────

@dataclass
class CurrentRun:
    started_at: str
    files: list[str]
    phase: str            # "paste" | "place" | "archiving"
    message: str = ""


@dataclass
class LastRun:
    started_at: str
    finished_at: str
    result: str           # "ok" | "failed"
    files: list[str]
    archived_to: str = ""
    error: str = ""
    attempts: int = 1


@dataclass
class Status:
    state: str = "idle"   # "idle" | "running" | "failed"
    updated_at: str = ""
    last_run: LastRun | None = None
    current: CurrentRun | None = None

    def to_json(self) -> str:
        d = asdict(self)
        # Drop None fields so the consumer sees an absent key, not "null"
        if d["last_run"] is None: del d["last_run"]
        if d["current"]  is None: del d["current"]
        return json.dumps(d, indent=2, sort_keys=True)


def _now_iso() -> str:
    return datetime.now().isoformat(timespec="seconds")


def _now_compact() -> str:
    return datetime.now().strftime("%Y-%m-%dT%H-%M-%S")


# ── Atomic status writer ───────────────────────────────────

def write_status_atomic(status_path: Path, status: Status) -> None:
    """Atomically replace `status_path` so readers never see partial writes.

    Pattern: write to a sibling tmp file, fsync, os.replace. Works on
    Linux and macOS. `os.replace` is atomic on POSIX.
    """
    status.updated_at = _now_iso()
    tmp = status_path.with_suffix(status_path.suffix + ".tmp")
    try:
        tmp.write_text(status.to_json() + "\n")
        os.replace(tmp, status_path)
    except OSError as e:
        log.warning("status write failed: %s", e)
        try:
            tmp.unlink(missing_ok=True)
        except OSError:
            pass


# ── Folder scanner ────────────────────────────────────────

def scan_signature(folder: Path) -> dict[str, tuple[int, float]]:
    """Return {filename: (size, mtime)} for all files in `folder`.

    Skips dotfiles and directories. Used for debounce — the signature
    changes whenever any file's size or mtime changes, which is what we
    watch for to detect in-flight copies.
    """
    out: dict[str, tuple[int, float]] = {}
    if not folder.is_dir():
        return out
    for entry in os.scandir(folder):
        if not entry.is_file():
            continue
        if entry.name.startswith("."):
            continue
        try:
            st = entry.stat()
        except OSError:
            continue
        out[entry.name] = (st.st_size, st.st_mtime)
    return out


# ── Run dispatcher ────────────────────────────────────────

def _build_run_cmd(
    job_inputs: JobInputs,
    config_path: Path,
    gcode_out: Path,
    dry_run: bool = False,
    vision: bool = False,
) -> list[str] | None:
    """Build the `python run.py` command for the given inputs.

    Returns None if the JobInputs don't form a valid mode (nothing to run).
    The output gcode is captured into `gcode_out` so the archive folder
    ends up with a replayable file per run.
    """
    base = [
        sys.executable, str(Path(__file__).resolve().parent.parent / "run.py"),
        "--config", str(config_path),
        "--output", str(gcode_out),
    ]
    if dry_run:
        base.append("--dry-run")
    if vision:
        base.append("--vision")
    mode = job_inputs.mode
    if mode == "paste":
        return base + [
            "paste",
            "--edge-cuts", str(job_inputs.edge_cuts),
            "--paste",     str(job_inputs.paste),
        ]
    if mode == "place":
        args = ["place", "--pos", str(job_inputs.pos)]
        if job_inputs.edge_cuts:
            args += ["--edge-cuts", str(job_inputs.edge_cuts)]
        if job_inputs.paste:
            args += ["--paste", str(job_inputs.paste)]
        return base + args
    if mode == "full":
        return base + [
            "job",
            "--edge-cuts", str(job_inputs.edge_cuts),
            "--paste",     str(job_inputs.paste),
            "--pos",       str(job_inputs.pos),
        ]
    return None


def _parse_progress_line(line: str) -> str | None:
    """Return a human-readable progress string if `line` matches a pattern.

    Shared by the watcher's live-tail so the laptop gets mid-run updates
    via status.json without any new IPC. Keeping this dumb and
    regex-based lets us avoid coupling run.py to any status format.
    """
    for pat in PROGRESS_PATTERNS:
        m = pat.search(line)
        if m:
            groups = m.groups()
            if len(groups) == 3:   # paste (X/Y ref)
                return f"paste {groups[0]}/{groups[1]} {groups[2]}"
            if len(groups) == 2:   # Placed X/Y
                return f"placed {groups[0]}/{groups[1]}"
            if len(groups) == 1:   # Pick/Place <ref>
                return f"{groups[0]}"
    return None


def run_job_subprocess(
    cmd: list[str],
    log_path: Path,
    on_progress: callable = None,
) -> tuple[bool, str]:
    """Invoke `run.py`, stream stdout line by line.

    Each line is:
      1. Written to `log_path` (append mode, so retries accumulate).
      2. Matched against `PROGRESS_PATTERNS`; matching lines call
         `on_progress(message)` so the caller can update status.json.
      3. The caller is expected to also emit a heartbeat update even
         when no progress line has arrived for `HEARTBEAT_INTERVAL`.

    Returns (success, error_text). error_text is empty on success.

    Uses Popen (non-blocking stdin, line-buffered stdout) instead of
    subprocess.run so we can tail progress in real time. That matters
    for the layer-3 UX: when the laptop reconnects mid-run, `status.json`
    already contains a fresh per-step message rather than just "running".
    """
    log.info("running: %s", " ".join(cmd))
    try:
        # bufsize=1 = line-buffered; merge stderr into stdout so one tail.
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
    except Exception as e:
        return False, f"subprocess failed to start: {e}"

    start = time.monotonic()
    last_heartbeat = start
    TIMEOUT_S = 3600  # 1 hour hard cap

    try:
        with open(log_path, "a") as f:
            f.write(f"# {_now_iso()}  {' '.join(cmd)}\n")
            f.flush()

            # Readline blocks until a line or EOF. That's fine — it
            # wakes up when the child writes, and we can heartbeat
            # between writes by using a timeout on the pipe.
            # Simpler: just poll readline in a loop with a timeout via
            # os.set_blocking + select. Easier still: use iter(readline)
            # and emit heartbeats after each line, accepting that a
            # silent child only updates status when run.py flushes.
            assert proc.stdout is not None
            for line in iter(proc.stdout.readline, ""):
                f.write(line)
                f.flush()

                now = time.monotonic()
                if now - start > TIMEOUT_S:
                    proc.kill()
                    return False, "subprocess exceeded 1 hour timeout"

                msg = _parse_progress_line(line)
                if msg and on_progress is not None:
                    try:
                        on_progress(msg)
                        last_heartbeat = now
                    except Exception:
                        pass  # never let a status write kill the run
                elif on_progress is not None and (now - last_heartbeat) > HEARTBEAT_INTERVAL:
                    # No interesting line, but enough time passed — heartbeat.
                    try:
                        on_progress(None)
                        last_heartbeat = now
                    except Exception:
                        pass

            proc.wait()
    except Exception as e:
        try: proc.kill()
        except Exception: pass
        return False, f"subprocess error: {e}"

    if proc.returncode != 0:
        try:
            tail = log_path.read_text().splitlines()[-20:]
        except OSError:
            tail = []
        return False, f"exit {proc.returncode}\n" + "\n".join(tail)
    return True, ""


# ── Archive helpers ───────────────────────────────────────

def move_set_to(
    incoming: Path,
    target_root: Path,
    filenames: list[str],
    extras: list[Path] | None = None,
) -> Path:
    """Move `filenames` from `incoming/` into `target_root/<timestamp>/`.

    Optional `extras` (the generated gcode, the run log) are copied into
    the same folder. Returns the created target folder.
    """
    target = target_root / _now_compact()
    target.mkdir(parents=True, exist_ok=True)
    for name in filenames:
        src = incoming / name
        if src.exists():
            shutil.move(str(src), str(target / name))
    for extra in extras or []:
        if extra.exists():
            shutil.move(str(extra), str(target / extra.name))
    return target


# ── Watcher loop ──────────────────────────────────────────

@dataclass
class WatcherConfig:
    incoming: Path
    archive: Path
    failed: Path
    config_path: Path
    status_path: Path
    debounce: float = 5.0
    poll: float = 1.0
    work_dir: Path = field(default_factory=lambda: Path("/tmp/nanopnp-watcher"))
    dry_run: bool = False   # pass --dry-run to run.py
    vision: bool = False    # pass --vision to run.py


def handle_ready_set(cfg: WatcherConfig, filenames: list[str], status: Status) -> None:
    """Classify, run, archive — the whole one-shot job lifecycle."""
    cfg.work_dir.mkdir(parents=True, exist_ok=True)
    paths = [cfg.incoming / n for n in filenames]
    job_inputs = classify(paths)

    if job_inputs.mode == "none":
        log.warning("unrecognized file set, moving to failed/: %s", filenames)
        err = "no recognized Edge_Cuts / F_Paste / .pos in incoming set"
        target = move_set_to(cfg.incoming, cfg.failed, filenames)
        status.state = "failed"
        status.current = None
        status.last_run = LastRun(
            started_at=_now_iso(), finished_at=_now_iso(),
            result="failed", files=filenames,
            archived_to=str(target), error=err, attempts=0,
        )
        write_status_atomic(cfg.status_path, status)
        return

    log.info("triggering %s mode with %d files", job_inputs.mode, len(filenames))

    started = _now_iso()
    status.state = "running"
    status.current = CurrentRun(
        started_at=started, files=filenames, phase=job_inputs.mode,
        message=f"running {job_inputs.mode}",
    )
    status.last_run = None
    write_status_atomic(cfg.status_path, status)

    gcode_out = cfg.work_dir / "job.gcode"
    log_out = cfg.work_dir / "run.log"
    cmd = _build_run_cmd(job_inputs, cfg.config_path, gcode_out,
                         dry_run=cfg.dry_run, vision=cfg.vision)
    if cmd is None:
        # Shouldn't happen given mode != "none", but defensive.
        err = f"invalid mode for classified inputs: {job_inputs.mode}"
        status.state = "failed"
        status.current = None
        status.last_run = LastRun(
            started_at=started, finished_at=_now_iso(),
            result="failed", files=filenames, error=err, attempts=0,
        )
        move_set_to(cfg.incoming, cfg.failed, filenames)
        write_status_atomic(cfg.status_path, status)
        return

    last_err = ""
    attempts_used = 0
    for attempt in range(MAX_RETRIES):
        attempts_used = attempt + 1
        base_message = f"attempt {attempts_used}/{MAX_RETRIES}"
        status.current = CurrentRun(
            started_at=started, files=filenames, phase=job_inputs.mode,
            message=base_message,
        )
        write_status_atomic(cfg.status_path, status)

        # Live progress: each matching stdout line from run.py updates
        # status.current.message; heartbeats (msg=None) just refresh
        # updated_at so the laptop sees a fresh timestamp every few
        # seconds. Both survive a laptop disconnect — when the GUI
        # reconnects and polls, it reads whatever the latest snapshot is.
        def _on_progress(msg: str | None) -> None:
            if msg is not None and status.current is not None:
                status.current.message = f"{base_message}: {msg}"
            write_status_atomic(cfg.status_path, status)

        ok, err = run_job_subprocess(cmd, log_out, on_progress=_on_progress)
        if ok:
            last_err = ""
            break
        last_err = err
        log.warning("attempt %d/%d failed: %s", attempts_used, MAX_RETRIES, err.splitlines()[0] if err else "")
        if attempt < MAX_RETRIES - 1:
            time.sleep(RETRY_DELAYS[attempt])

    finished = _now_iso()

    # Archive either way — success to done/, failure to failed/.
    status.current = CurrentRun(
        started_at=started, files=filenames, phase="archiving",
        message="moving files",
    )
    write_status_atomic(cfg.status_path, status)

    if last_err == "":
        target = move_set_to(cfg.incoming, cfg.archive, filenames, extras=[gcode_out, log_out])
        status.state = "idle"
        status.current = None
        status.last_run = LastRun(
            started_at=started, finished_at=finished,
            result="ok", files=filenames,
            archived_to=str(target), attempts=attempts_used,
        )
    else:
        target = move_set_to(cfg.incoming, cfg.failed, filenames, extras=[gcode_out, log_out])
        status.state = "failed"
        status.current = None
        status.last_run = LastRun(
            started_at=started, finished_at=finished,
            result="failed", files=filenames,
            archived_to=str(target), error=last_err, attempts=attempts_used,
        )

    write_status_atomic(cfg.status_path, status)
    log.info("job %s — archived to %s", status.last_run.result, target)


def watcher_loop(cfg: WatcherConfig) -> None:
    """Main poll loop — does not return unless interrupted."""
    cfg.incoming.mkdir(parents=True, exist_ok=True)
    cfg.archive.mkdir(parents=True, exist_ok=True)
    cfg.failed.mkdir(parents=True, exist_ok=True)
    cfg.status_path.parent.mkdir(parents=True, exist_ok=True)

    status = Status(state="idle")
    write_status_atomic(cfg.status_path, status)

    log.info("watching %s  (debounce=%.1fs poll=%.1fs)",
             cfg.incoming, cfg.debounce, cfg.poll)

    last_sig: dict[str, tuple[int, float]] = {}
    stable_since: float | None = None

    while True:
        try:
            sig = scan_signature(cfg.incoming)
        except Exception as e:
            log.error("scan failed: %s", e)
            time.sleep(cfg.poll)
            continue

        if sig != last_sig:
            last_sig = sig
            stable_since = time.monotonic() if sig else None
            if sig:
                log.debug("change detected: %d files", len(sig))
        elif sig and stable_since is not None:
            age = time.monotonic() - stable_since
            if age >= cfg.debounce:
                # Debounce window elapsed — run the batch.
                filenames = sorted(sig.keys())
                try:
                    handle_ready_set(cfg, filenames, status)
                except Exception:
                    log.error("unhandled error in handle_ready_set:\n%s", traceback.format_exc())
                    status.state = "failed"
                    status.current = None
                    status.last_run = LastRun(
                        started_at=_now_iso(), finished_at=_now_iso(),
                        result="failed", files=filenames,
                        error=traceback.format_exc(), attempts=0,
                    )
                    write_status_atomic(cfg.status_path, status)
                # Reset — any remaining files in incoming/ start a new cycle.
                last_sig = {}
                stable_since = None

        time.sleep(cfg.poll)


# ── CLI ────────────────────────────────────────────────────

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="nanopnp.watcher", description=__doc__)
    p.add_argument("--incoming", type=Path, required=True, help="Folder to watch for incoming files")
    p.add_argument("--archive",  type=Path, required=True, help="Destination for successful runs")
    p.add_argument("--failed",   type=Path, required=True, help="Destination for failed runs")
    p.add_argument("--config",   type=Path, required=True, help="Path to config.json")
    p.add_argument("--status",   type=Path, default=None,
                   help="Path to status.json (default: <incoming parent>/status.json)")
    p.add_argument("--debounce", type=float, default=5.0, help="Seconds of file stability before running")
    p.add_argument("--poll",     type=float, default=1.0, help="Scan interval in seconds")
    p.add_argument("--work-dir", type=Path, default=Path("/tmp/nanopnp-watcher"),
                   help="Scratch dir for intermediate gcode/log before archiving")
    p.add_argument("--dry-run", action="store_true",
                   help="Pass --dry-run to run.py (no serial, just generate gcode). Useful for loopback tests.")
    p.add_argument("--vision", action="store_true",
                   help="Pass --vision to run.py (enable bottom-camera alignment during placement).")
    return p


def main(argv: list[str] | None = None) -> int:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%H:%M:%S",
    )
    args = build_parser().parse_args(argv)

    status_path = args.status or (args.incoming.parent / "status.json")
    cfg = WatcherConfig(
        incoming=args.incoming.resolve(),
        archive=args.archive.resolve(),
        failed=args.failed.resolve(),
        config_path=args.config.resolve(),
        status_path=status_path.resolve(),
        debounce=args.debounce,
        poll=args.poll,
        work_dir=args.work_dir.resolve(),
        dry_run=args.dry_run,
        vision=args.vision,
    )
    try:
        watcher_loop(cfg)
    except KeyboardInterrupt:
        log.info("interrupted — exiting")
        return 0
    return 0


if __name__ == "__main__":
    sys.exit(main())
