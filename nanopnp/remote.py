"""SCP-based sender for the desktop side of the Jetson pipeline.

The laptop picks up a `JobInputs` bundle in the GUI, then either:
  - runs it locally (current default), or
  - ships it to the Jetson's `incoming/` folder with `send_job()` →
    the watcher daemon there picks it up, debounces, runs, archives.

The laptop also polls `fetch_status()` every few seconds to mirror the
Jetson's state in the toolbar. Both operations shell out to the stock
`ssh` / `scp` binaries — no `paramiko`, no `fabric`, keeping the Jetson's
production dependency surface at exactly `python3 + stdlib`.

`RemoteTarget` is persisted in `config.json` under a new `remote` section
so the GUI can reopen with the last-used target.
"""

from __future__ import annotations

import json
import logging
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path

from nanopnp.job_inputs import JobInputs

log = logging.getLogger(__name__)


# ── Target description ────────────────────────────────────

@dataclass(slots=True)
class RemoteTarget:
    host: str
    user: str
    incoming_path: str = "~/nanopnp/incoming"   # remote shell will expand ~
    status_path: str = "~/nanopnp/status.json"
    ssh_key: Path | None = None
    port: int = 22

    def user_host(self) -> str:
        return f"{self.user}@{self.host}"

    def ssh_base(self) -> list[str]:
        """Args common to every ssh invocation for this target."""
        base = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=3"]
        if self.port != 22:
            base += ["-p", str(self.port)]
        if self.ssh_key is not None:
            base += ["-i", str(self.ssh_key)]
        base.append(self.user_host())
        return base

    def scp_base(self) -> list[str]:
        """Args common to every scp invocation for this target."""
        base = ["scp", "-o", "BatchMode=yes", "-o", "ConnectTimeout=3", "-q"]
        if self.port != 22:
            base += ["-P", str(self.port)]
        if self.ssh_key is not None:
            base += ["-i", str(self.ssh_key)]
        return base


# ── Result types ──────────────────────────────────────────

@dataclass(slots=True)
class FileUploadResult:
    name: str
    ok: bool
    bytes: int
    error: str = ""


@dataclass(slots=True)
class SendResult:
    ok: bool
    files: list[FileUploadResult] = field(default_factory=list)
    duration_s: float = 0.0
    error: str = ""

    @property
    def summary(self) -> str:
        if not self.ok:
            return f"failed: {self.error}"
        total = sum(f.bytes for f in self.files)
        return f"sent {len(self.files)} files ({total} bytes) in {self.duration_s:.1f}s"


# ── Preflight ─────────────────────────────────────────────

def check_reachable(target: RemoteTarget) -> tuple[bool, str]:
    """Quick ssh ping. Returns (reachable, error_msg)."""
    try:
        r = subprocess.run(
            target.ssh_base() + ["true"],
            capture_output=True, text=True, timeout=5,
        )
    except subprocess.TimeoutExpired:
        return False, "ssh timeout"
    except FileNotFoundError:
        return False, "ssh not installed"
    if r.returncode != 0:
        stderr = r.stderr.strip().splitlines()[-1] if r.stderr else f"exit {r.returncode}"
        return False, stderr
    return True, ""


def ensure_incoming_dir(target: RemoteTarget) -> tuple[bool, str]:
    """`mkdir -p` the remote incoming folder. Runs once per session."""
    try:
        r = subprocess.run(
            target.ssh_base() + [f"mkdir -p {target.incoming_path}"],
            capture_output=True, text=True, timeout=5,
        )
    except subprocess.TimeoutExpired:
        return False, "ssh timeout"
    if r.returncode != 0:
        return False, r.stderr.strip() or f"exit {r.returncode}"
    return True, ""


# ── Upload ────────────────────────────────────────────────

def _scp_one(target: RemoteTarget, local: Path) -> FileUploadResult:
    """SCP a single file and report."""
    if not local.exists():
        return FileUploadResult(local.name, False, 0, "local file missing")

    size = local.stat().st_size
    cmd = target.scp_base() + [str(local), f"{target.user_host()}:{target.incoming_path}/"]

    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
    except subprocess.TimeoutExpired:
        return FileUploadResult(local.name, False, size, "scp timeout")
    except FileNotFoundError:
        return FileUploadResult(local.name, False, size, "scp not installed")

    if r.returncode != 0:
        err = r.stderr.strip() or f"exit {r.returncode}"
        return FileUploadResult(local.name, False, size, err)
    return FileUploadResult(local.name, True, size, "")


def send_job(
    target: RemoteTarget,
    inputs: JobInputs,
    on_progress: callable = None,
) -> SendResult:
    """Upload the files in `inputs` to `target.incoming_path`.

    Upload order is deterministic: paste → edge_cuts → pos. The watcher
    debounces on file-set stability, not order, but shipping `.pos` last
    means the "trigger" file arrives after the others — nicer for
    troubleshooting, not semantically required.

    `on_progress(text)` is called for each file before/after upload
    so the GUI can update a status line. Any exception in on_progress
    is swallowed.
    """
    start = time.monotonic()

    ok_reach, err = check_reachable(target)
    if not ok_reach:
        return SendResult(ok=False, error=f"unreachable: {err}",
                          duration_s=round(time.monotonic() - start, 2))

    ok_mkdir, err = ensure_incoming_dir(target)
    if not ok_mkdir:
        return SendResult(ok=False, error=f"mkdir: {err}",
                          duration_s=round(time.monotonic() - start, 2))

    order: list[Path] = []
    if inputs.paste:     order.append(inputs.paste)
    if inputs.edge_cuts: order.append(inputs.edge_cuts)
    if inputs.pos:       order.append(inputs.pos)

    results: list[FileUploadResult] = []
    for p in order:
        def _fire(text: str) -> None:
            if on_progress is None: return
            try: on_progress(text)
            except Exception: pass
        _fire(f"uploading {p.name} ({p.stat().st_size} bytes)...")
        res = _scp_one(target, p)
        results.append(res)
        _fire(f"{p.name}: {'ok' if res.ok else 'FAIL: ' + res.error}")
        if not res.ok:
            # Abort on first failure — partial uploads would trip the
            # watcher's debounce and could run a half-received job.
            return SendResult(
                ok=False, files=results,
                duration_s=round(time.monotonic() - start, 2),
                error=f"{p.name}: {res.error}",
            )

    return SendResult(
        ok=True, files=results,
        duration_s=round(time.monotonic() - start, 2),
    )


# ── Status polling ────────────────────────────────────────

@dataclass(slots=True)
class JetsonStatus:
    """Mirror of the watcher's status.json — only the fields we display."""
    reachable: bool
    state: str = "unknown"         # idle | running | failed | unknown
    updated_at: str = ""
    last_run_result: str = ""      # ok | failed | ""
    last_run_at: str = ""
    current_message: str = ""
    error: str = ""


def fetch_status(target: RemoteTarget) -> JetsonStatus:
    """SSH into the target, read `status.json`, return a parsed snapshot.

    Always returns a `JetsonStatus` — connection failures set
    `reachable=False` with the error in `.error`. Never raises.
    """
    try:
        r = subprocess.run(
            target.ssh_base() + [f"cat {target.status_path}"],
            capture_output=True, text=True, timeout=5,
        )
    except subprocess.TimeoutExpired:
        return JetsonStatus(reachable=False, error="ssh timeout")
    except FileNotFoundError:
        return JetsonStatus(reachable=False, error="ssh not installed")

    if r.returncode != 0:
        err = r.stderr.strip().splitlines()[-1] if r.stderr else f"exit {r.returncode}"
        return JetsonStatus(reachable=False, error=err)

    try:
        d = json.loads(r.stdout)
    except json.JSONDecodeError as e:
        return JetsonStatus(reachable=True, error=f"bad json: {e}")

    last = d.get("last_run") or {}
    cur = d.get("current") or {}
    return JetsonStatus(
        reachable=True,
        state=d.get("state", "unknown"),
        updated_at=d.get("updated_at", ""),
        last_run_result=last.get("result", ""),
        last_run_at=last.get("finished_at", ""),
        current_message=cur.get("message", ""),
    )
