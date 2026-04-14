"""NanoPnP GUI — light-themed, cross-platform (macOS + Ubuntu 18+).

Layout: toolbar + left sidebar (jog/vacuum/tool) + tabbed workspace (Job, Setup, Camera, Console).
All configuration editable from GUI and saveable to config.json.
"""

from __future__ import annotations

import json
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox, simpledialog, filedialog, font as tkfont
from pathlib import Path

from nanopnp.config import NanoPnPConfig, save_config
from nanopnp.serial_comm import SerialConnection
from nanopnp.motion import MotionController
from nanopnp.feeder import FeederManager
from nanopnp.paste_dispenser import PasteDispenser
from nanopnp.pnp_engine import PnPEngine

try:
    import cv2
    from PIL import Image, ImageTk
    HAS_CAMERA = True
except ImportError:
    HAS_CAMERA = False

# ── Light theme ───────────────────────────────────────────────

BG = "#f5f5f7"
BG_PANEL = "#ffffff"
BG_INPUT = "#ffffff"
BORDER = "#d1d1d6"
FG = "#1d1d1f"
FG_DIM = "#86868b"
ACCENT = "#0071e3"
GREEN = "#34c759"
RED = "#ff3b30"
ORANGE = "#ff9500"


def _interp_ratio(f_val: float, table: dict[int, float]) -> float:
    """Linearly interpolate speed ratio from a sparse lookup table.

    Returns the ratio of actual/theoretical speed at the given feedrate.
    Extrapolates at the edges.
    """
    keys = sorted(table.keys())
    if f_val <= keys[0]:
        return table[keys[0]]
    if f_val >= keys[-1]:
        return table[keys[-1]]
    for i in range(len(keys) - 1):
        lo, hi = keys[i], keys[i + 1]
        if lo <= f_val <= hi:
            t = (f_val - lo) / (hi - lo)
            return table[lo] + (table[hi] - table[lo]) * t
    return 1.0


def _pick_font(root, candidates, size=10, weight="normal"):
    available = tkfont.families()
    for name in candidates:
        if name in available:
            return (name, size, weight)
    return ("TkDefaultFont", size, weight)


def _apply_theme(root):
    style = ttk.Style(root)
    style.theme_use("clam")
    style.configure(".", background=BG, foreground=FG, fieldbackground=BG_INPUT, bordercolor=BORDER)
    style.configure("TFrame", background=BG)
    style.configure("TLabel", background=BG, foreground=FG)
    style.configure("TLabelframe", background=BG, foreground=FG, bordercolor=BORDER)
    style.configure("TLabelframe.Label", background=BG, foreground=ACCENT)
    style.configure("TButton", padding=(6, 3))
    style.configure("TNotebook", background=BG, bordercolor=BORDER)
    style.configure("TNotebook.Tab", background=BG_PANEL, foreground=FG_DIM, padding=(14, 4))
    style.map("TNotebook.Tab", background=[("selected", BG)], foreground=[("selected", ACCENT)])
    style.configure("TCheckbutton", background=BG)
    style.configure("TRadiobutton", background=BG)
    style.configure("TEntry", insertcolor=FG)
    style.configure("TCombobox", fieldbackground=BG_INPUT, insertcolor=FG)
    style.configure("Treeview", background=BG_INPUT, fieldbackground=BG_INPUT, rowheight=22)
    style.configure("Treeview.Heading", background=BG_PANEL, foreground=ACCENT)
    style.map("Treeview", background=[("selected", "#d0e0ff")])
    style.configure("Green.TButton", foreground=GREEN)
    style.configure("Red.TButton", foreground=RED)
    style.configure("Accent.TButton", foreground=ACCENT)
    root.configure(bg=BG)


# ── GUI ───────────────────────────────────────────────────────

class NanoPnPGUI:
    def __init__(self, config: NanoPnPConfig, config_path: str = "config.json") -> None:
        self._config = config
        self._config_path = config_path
        self._serial: SerialConnection | None = None
        self._motion: MotionController | None = None
        self._feeders: FeederManager | None = None
        self._paste: PasteDispenser | None = None
        self._engine: PnPEngine | None = None

        self._cmd_queue: queue.Queue = queue.Queue()
        self._result_queue: queue.Queue = queue.Queue()
        self._worker: threading.Thread | None = None
        self._monitor: threading.Thread | None = None
        self._running = False
        self._polling = False
        self._save_timer: str | None = None
        self._cmd_history: list[str] = []
        self._history_idx: int = -1
        self._camera_cap = None
        self._pos = {"X": 0.0, "Y": 0.0, "Z": 0.0, "E": 0.0}

        # File-driven job inputs (see nanopnp.job_inputs.JobInputs).
        # When a .pos is loaded, _pos_source points at the file and
        # _pos_aux keeps the parsed val/side/package columns that don't
        # fit in Placement — they're needed to round-trip writes.
        from nanopnp.job_inputs import JobInputs as _JI
        from pathlib import Path as _Path
        self._job_inputs: _JI = _JI()
        self._pos_source: _Path | None = None
        self._pos_aux: dict[str, dict] = {}

        self._root = tk.Tk()
        self._root.title("NanoPnP")
        self._root.geometry("1120x750")
        self._root.minsize(960, 600)
        _apply_theme(self._root)

        self._font = _pick_font(self._root, ["SF Pro", "Segoe UI", "DejaVu Sans", "Helvetica"])
        self._mono = _pick_font(self._root, ["SF Mono", "Consolas", "DejaVu Sans Mono", "Courier"], size=10)

        self._step_size = tk.DoubleVar(value=1.0)
        self._dry_run_var = tk.BooleanVar(value=False)
        self._tool_var = tk.IntVar(value=0)
        self._vision_enabled_var = tk.BooleanVar(value=False)
        self._vision_apply_var = tk.BooleanVar(value=True)

        self._build_layout()
        self._bind_keys()
        self._update_inputs_state()   # set initial mode label + button gates
        self._poll_results()

        # Auto-reconnect: check for reload env var or last-connection file
        import os
        last_conn = self._load_last_connection()
        should_auto = os.environ.pop("NANOPNP_AUTO_CONNECT", None)
        os.environ.pop("NANOPNP_PORT", None)
        os.environ.pop("NANOPNP_BAUD", None)

        # Always pre-fill with last known port (so dropdown shows the right one)
        if last_conn and last_conn.get("port"):
            self._port_var.set(last_conn["port"])

        # Auto-click Connect only after a reload (not on cold start)
        if should_auto:
            self._root.after(500, self._on_connect)

    # ── Layout ────────────────────────────────────────────────

    def _build_layout(self):
        self._build_toolbar().pack(fill=tk.X, padx=6, pady=(6, 2))
        body = ttk.Frame(self._root)
        body.pack(fill=tk.BOTH, expand=True, padx=6, pady=(0, 6))
        body.columnconfigure(1, weight=1)
        body.rowconfigure(0, weight=1)
        self._build_sidebar(body).grid(row=0, column=0, sticky="ns", padx=(0, 4))
        self._build_tabs(body).grid(row=0, column=1, sticky="nsew")

    # ── Toolbar ───────────────────────────────────────────────

    def _build_toolbar(self) -> ttk.Frame:
        tb = ttk.Frame(self._root)

        # Connection
        self._port_var = tk.StringVar(value=self._config.machine.serial_port)
        ttk.Label(tb, text="Port:").pack(side=tk.LEFT, padx=(0, 2))
        self._port_combo = ttk.Combobox(tb, textvariable=self._port_var, width=18)
        self._port_combo.pack(side=tk.LEFT)
        self._refresh_ports()
        self._port_combo.bind("<Button-1>", lambda e: self._refresh_ports())

        ttk.Checkbutton(tb, text="Dry Run", variable=self._dry_run_var).pack(side=tk.LEFT, padx=6)
        self._connect_btn = ttk.Button(tb, text="Connect", command=self._on_connect, width=9)
        self._connect_btn.pack(side=tk.LEFT)
        self._disconnect_btn = ttk.Button(tb, text="Disconnect", command=self._on_disconnect, width=9, state=tk.DISABLED)
        self._disconnect_btn.pack(side=tk.LEFT, padx=(2, 6))

        self._status_dot = tk.Canvas(tb, width=12, height=12, bg=BG, highlightthickness=0)
        self._status_dot.create_oval(2, 2, 10, 10, fill=RED, outline=BORDER, tags="dot")
        self._status_dot.pack(side=tk.LEFT, padx=(0, 8))

        # Position DROs
        self._pos_labels = {}
        for axis in ["X", "Y", "Z", "E"]:
            ttk.Label(tb, text=f"{axis}:", foreground=FG_DIM, font=self._mono).pack(side=tk.LEFT)
            lbl = ttk.Label(tb, text="  0.000", font=self._mono, width=7, anchor="e")
            lbl.pack(side=tk.LEFT, padx=(0, 6))
            self._pos_labels[axis] = lbl

        # Status
        self._vac_label = ttk.Label(tb, text="VAC:OFF", foreground=FG_DIM, font=self._mono)
        self._vac_label.pack(side=tk.LEFT, padx=4)
        self._tool_label = ttk.Label(tb, text="T0", foreground=FG_DIM, font=self._mono)
        self._tool_label.pack(side=tk.LEFT, padx=4)

        # Right side
        ttk.Button(tb, text="Home", command=self._on_home, width=5).pack(side=tk.LEFT, padx=4)
        ttk.Button(tb, text="↻ Reload", command=self._on_reload, width=9).pack(side=tk.LEFT, padx=4)

        # Target selector: Local ↔ Jetson. Jetson mode hijacks the Run
        # buttons to scp files into the watcher's incoming/ folder and
        # polls remote status.json every few seconds.
        ttk.Separator(tb, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=6)
        ttk.Label(tb, text="Target:", foreground=FG_DIM).pack(side=tk.LEFT, padx=(0, 2))
        self._target_mode = tk.StringVar(value="local")
        ttk.Radiobutton(tb, text="Local", variable=self._target_mode, value="local",
                        command=self._on_target_changed).pack(side=tk.LEFT)
        ttk.Radiobutton(tb, text="Jetson", variable=self._target_mode, value="jetson",
                        command=self._on_target_changed).pack(side=tk.LEFT)
        self._target_status_label = ttk.Label(tb, text="", foreground=FG_DIM, font=self._mono)
        self._target_status_label.pack(side=tk.LEFT, padx=(4, 2))
        ttk.Button(tb, text="⚙", width=2, command=self._open_remote_settings).pack(side=tk.LEFT, padx=(0, 4))

        estop = tk.Button(tb, text="E-STOP", bg=RED, fg="white", activebackground="#cc0000",
                          activeforeground="white", font=("Helvetica", 11, "bold"),
                          relief="raised", bd=2, command=self._on_estop)
        estop.pack(side=tk.RIGHT, padx=2)

        return tb

    # ── Sidebar ───────────────────────────────────────────────

    def _build_sidebar(self, parent) -> ttk.Frame:
        sb = ttk.Frame(parent, width=170)
        sb.grid_propagate(False)

        # Jog
        jog = ttk.LabelFrame(sb, text="Jog")
        jog.pack(fill=tk.X, pady=(0, 4))
        bw = 5
        pad = ttk.Frame(jog)
        pad.pack(pady=2)
        ttk.Button(pad, text="Y+", width=bw, command=lambda: self._jog("Y", 1)).grid(row=0, column=1, padx=1, pady=1)
        ttk.Button(pad, text="X-", width=bw, command=lambda: self._jog("X", -1)).grid(row=1, column=0, padx=1, pady=1)
        ttk.Button(pad, text="X+", width=bw, command=lambda: self._jog("X", 1)).grid(row=1, column=2, padx=1, pady=1)
        ttk.Button(pad, text="Y-", width=bw, command=lambda: self._jog("Y", -1)).grid(row=2, column=1, padx=1, pady=1)

        ze = ttk.Frame(jog)
        ze.pack(pady=2)
        ttk.Button(ze, text="Z+", width=bw, command=lambda: self._jog("Z", 1)).pack(side=tk.LEFT, padx=1)
        ttk.Button(ze, text="Z-", width=bw, command=lambda: self._jog("Z", -1)).pack(side=tk.LEFT, padx=1)
        ttk.Button(ze, text="E+", width=bw, command=lambda: self._jog("E", 1)).pack(side=tk.LEFT, padx=1)
        ttk.Button(ze, text="E-", width=bw, command=lambda: self._jog("E", -1)).pack(side=tk.LEFT, padx=1)

        sf = ttk.Frame(jog)
        sf.pack(pady=2)
        ttk.Label(sf, text="Step:", foreground=FG_DIM).pack(side=tk.LEFT)
        for s in [0.1, 1.0, 10.0]:
            ttk.Radiobutton(sf, text=str(s), variable=self._step_size, value=s).pack(side=tk.LEFT)

        # Vacuum
        vac = ttk.LabelFrame(sb, text="Vacuum")
        vac.pack(fill=tk.X, pady=4)
        vr = ttk.Frame(vac)
        vr.pack(pady=4)
        ttk.Button(vr, text="ON", style="Green.TButton", width=6,
                   command=lambda: self._send_cmd("__VAC_ON__")).pack(side=tk.LEFT, padx=2)
        ttk.Button(vr, text="OFF", style="Red.TButton", width=6,
                   command=lambda: self._send_cmd("__VAC_OFF__")).pack(side=tk.LEFT, padx=2)

        # Tool
        tool = ttk.LabelFrame(sb, text="Tool")
        tool.pack(fill=tk.X, pady=4)
        ttk.Radiobutton(tool, text="T0 Nozzle", variable=self._tool_var, value=0,
                        command=lambda: self._send_cmd("__T0__")).pack(anchor="w", padx=6)
        ttk.Radiobutton(tool, text="T1 Paste", variable=self._tool_var, value=1,
                        command=lambda: self._send_cmd("__T1__")).pack(anchor="w", padx=6)

        # Quick positions
        qp = ttk.LabelFrame(sb, text="Quick Pos")
        qp.pack(fill=tk.X, pady=4)
        cam = self._config.camera.position
        ttk.Button(qp, text="Camera", width=14,
                   command=lambda: self._send_cmd(("__GOTO__", cam.x, cam.y, None))).pack(pady=1, padx=4)
        f1 = self._config.feeders.get("F1")
        if f1:
            ttk.Button(qp, text="Feeder F1", width=14,
                       command=lambda: self._send_cmd(("__GOTO__", f1.ref_hole.x, f1.ref_hole.y, None))).pack(pady=1, padx=4)
        ttk.Button(qp, text="GoTo...", width=14, command=self._on_goto).pack(pady=1, padx=4)
        ttk.Button(qp, text="Go to 0,0,0", width=14,
                   command=lambda: self._send_cmd(("__GOTO__", 0, 0, 0))).pack(pady=1, padx=4)
        ttk.Button(qp, text="Set as 0,0,0", width=14, command=lambda: self._send_cmd("__SET_ORIGIN__")).pack(pady=1, padx=4)

        return sb

    # ── Tabs ──────────────────────────────────────────────────

    def _build_tabs(self, parent) -> ttk.Notebook:
        self._nb = ttk.Notebook(parent)
        self._nb.add(self._build_job_tab(), text="  Job  ")
        self._nb.add(self._build_setup_tab(), text="  Setup  ")
        self._nb.add(self._build_camera_tab(), text="  Camera  ")
        self._nb.add(self._build_visualizer_tab(), text="  Visualizer  ")
        self._nb.add(self._build_console_tab(), text="  Console  ")
        return self._nb

    # ── Tab: Job ──────────────────────────────────────────────

    def _build_job_tab(self) -> ttk.Frame:
        tab = ttk.Frame(self._nb)
        tab.rowconfigure(0, weight=0)           # Inputs panel (fixed)
        tab.rowconfigure(1, weight=2)           # Placements
        tab.rowconfigure(2, weight=0, minsize=40)  # Run controls
        tab.rowconfigure(3, weight=1)           # Results
        tab.columnconfigure(0, weight=1)

        # Inputs panel — three file pickers that drive the run buttons
        in_f = ttk.LabelFrame(tab, text="Inputs")
        in_f.grid(row=0, column=0, sticky="ew", padx=4, pady=(4, 2))
        in_f.columnconfigure(1, weight=1)

        self._input_path_vars: dict[str, tk.StringVar] = {}
        rows = [
            ("edge_cuts", "Edge Cuts", [("Gerber files", "*.gbr"), ("All", "*")]),
            ("paste",     "F Paste",   [("Gerber files", "*.gbr"), ("All", "*")]),
            ("pos",       "Positions", [("Pos files", "*.pos"), ("All", "*")]),
        ]
        for i, (key, label, filetypes) in enumerate(rows):
            ttk.Label(in_f, text=label + ":", foreground=FG_DIM).grid(
                row=i, column=0, sticky="e", padx=(6, 4), pady=1)
            var = tk.StringVar(value="")
            self._input_path_vars[key] = var
            ttk.Entry(in_f, textvariable=var, state="readonly", font=self._mono).grid(
                row=i, column=1, sticky="ew", padx=2, pady=1)
            ttk.Button(
                in_f, text="...", width=3,
                command=lambda k=key, ft=filetypes: self._pick_input(k, ft),
            ).grid(row=i, column=2, padx=1, pady=1)
            ttk.Button(
                in_f, text="✕", width=2,
                command=lambda k=key: self._clear_input(k),
            ).grid(row=i, column=3, padx=(1, 6), pady=1)

        self._mode_label = ttk.Label(in_f, text="Mode: NONE", foreground=FG_DIM, font=self._mono)
        self._mode_label.grid(row=len(rows), column=0, columnspan=4, sticky="w", padx=6, pady=(2, 4))

        # Placements table
        pl_f = ttk.LabelFrame(tab, text="Placements")
        pl_f.grid(row=1, column=0, sticky="nsew", padx=4, pady=(2, 2))
        cols = ("ref", "part", "x", "y", "rot", "enabled")
        self._place_tree = ttk.Treeview(pl_f, columns=cols, show="headings", height=6)
        for c, w in zip(cols, (60, 120, 70, 70, 50, 60)):
            self._place_tree.heading(c, text=c.title())
            self._place_tree.column(c, width=w, anchor="center")
        self._place_tree.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)
        self._place_tree.bind("<Double-1>", lambda e: self._edit_selected_placement())
        self._populate_placements()

        pb = ttk.Frame(pl_f)
        pb.pack(fill=tk.X, padx=2, pady=2)
        ttk.Button(pb, text="Edit", command=self._edit_selected_placement).pack(side=tk.LEFT, padx=2)
        ttk.Button(pb, text="Go To", command=self._goto_selected_placement).pack(side=tk.LEFT, padx=2)
        ttk.Button(pb, text="Toggle Enable", command=self._toggle_placement_enable).pack(side=tk.LEFT, padx=2)
        ttk.Button(pb, text="Enable All", command=lambda: self._set_all_placements(True)).pack(side=tk.LEFT, padx=2)
        ttk.Button(pb, text="Disable All", command=lambda: self._set_all_placements(False)).pack(side=tk.LEFT, padx=2)
        ttk.Separator(pb, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=4)
        ttk.Button(pb, text="Solder Selected", command=self._on_solder_selected).pack(side=tk.LEFT, padx=2)
        ttk.Button(pb, text="Solder Dry", command=self._on_solder_dry).pack(side=tk.LEFT, padx=2)

        # Run controls
        ctrl = ttk.LabelFrame(tab, text="Run")
        ctrl.grid(row=2, column=0, sticky="ew", padx=4, pady=6)
        cr = ttk.Frame(ctrl)
        cr.pack(fill=tk.X, padx=4, pady=6)
        self._btn_paste = ttk.Button(cr, text="Solder Only", style="Accent.TButton",
                                     command=self._on_run_paste, state=tk.DISABLED)
        self._btn_paste.pack(side=tk.LEFT, padx=3)
        self._btn_place = ttk.Button(cr, text="Pick & Place Only", style="Green.TButton",
                                     command=self._on_run_place, state=tk.DISABLED)
        self._btn_place.pack(side=tk.LEFT, padx=3)
        self._btn_full = ttk.Button(cr, text="Full Job (Solder + PnP)", style="Accent.TButton",
                                    command=self._on_run_full_job, state=tk.DISABLED)
        self._btn_full.pack(side=tk.LEFT, padx=3)
        ttk.Separator(cr, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=6)
        ttk.Button(cr, text="STOP", style="Red.TButton", command=self._on_stop_job).pack(side=tk.LEFT, padx=3)
        ttk.Checkbutton(cr, text="Vision", variable=self._vision_enabled_var).pack(side=tk.LEFT, padx=6)
        ttk.Checkbutton(cr, text="Apply", variable=self._vision_apply_var).pack(side=tk.LEFT, padx=2)
        self._progress_var = tk.DoubleVar(value=0)
        ttk.Progressbar(cr, variable=self._progress_var, maximum=100, length=200).pack(side=tk.LEFT, padx=6)
        self._progress_label = ttk.Label(cr, text="Idle", foreground=FG_DIM)
        self._progress_label.pack(side=tk.LEFT, padx=4)

        # Results table
        res_f = ttk.LabelFrame(tab, text="Results")
        res_f.grid(row=3, column=0, sticky="nsew", padx=4, pady=(2, 4))
        rcols = ("ref", "part", "nominal", "corrected", "dx", "dy", "drot", "status")
        self._result_tree = ttk.Treeview(res_f, columns=rcols, show="headings", height=6)
        for c, w in zip(rcols, (50, 80, 100, 100, 55, 55, 55, 70)):
            self._result_tree.heading(c, text=c.title())
            self._result_tree.column(c, width=w, anchor="center")
        self._result_tree.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)
        self._result_tree.tag_configure("ok", foreground=GREEN)
        self._result_tree.tag_configure("fail", foreground=RED)

        return tab

    # ── Tab: Setup ────────────────────────────────────────────

    def _build_setup_tab(self) -> ttk.Frame:
        tab = ttk.Frame(self._nb)
        tab.rowconfigure(0, weight=1)
        tab.rowconfigure(1, weight=0)
        tab.columnconfigure(0, weight=1)
        tab.columnconfigure(1, weight=1)

        # Feeders (top-left)
        fd_f = ttk.LabelFrame(tab, text="Feeders")
        fd_f.grid(row=0, column=0, sticky="nsew", padx=(4, 2), pady=(4, 2))
        fcols = ("slot", "part", "ref_hole", "pitch", "count", "remain", "enabled")
        self._feed_tree = ttk.Treeview(fd_f, columns=fcols, show="headings", height=5)
        for c, w in zip(fcols, (40, 80, 120, 40, 40, 45, 50)):
            self._feed_tree.heading(c, text=c.title())
            self._feed_tree.column(c, width=w, anchor="center")
        self._feed_tree.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)
        self._feed_tree.bind("<Double-1>", lambda e: self._edit_selected_feeder())
        self._populate_feeders()

        fb = ttk.Frame(fd_f)
        fb.pack(fill=tk.X, padx=2, pady=2)
        ttk.Button(fb, text="Edit", command=self._edit_selected_feeder).pack(side=tk.LEFT, padx=2)
        ttk.Button(fb, text="Go To", command=self._goto_selected_feeder).pack(side=tk.LEFT, padx=2)
        ttk.Button(fb, text="Reset Count", command=self._reset_feed_count).pack(side=tk.LEFT, padx=2)

        # Parts (top-right)
        pt_f = ttk.LabelFrame(tab, text="Parts")
        pt_f.grid(row=0, column=1, sticky="nsew", padx=(2, 4), pady=(4, 2))
        pcols = ("name", "height", "body", "pads")
        self._parts_tree = ttk.Treeview(pt_f, columns=pcols, show="headings", height=5)
        for c, w in zip(pcols, (100, 60, 80, 40)):
            self._parts_tree.heading(c, text=c.title())
            self._parts_tree.column(c, width=w, anchor="center")
        self._parts_tree.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)
        for name, p in self._config.parts.items():
            self._parts_tree.insert("", tk.END, values=(name, f"{p.height}", f"{p.body_width}x{p.body_length}", len(p.pads)))

        # Machine config (bottom-left)
        mc_f = ttk.LabelFrame(tab, text="Machine")
        mc_f.grid(row=1, column=0, sticky="nsew", padx=(4, 2), pady=(2, 4))
        self._cfg_vars = {}
        fields = [
            ("Travel Feedrate", "travel_feedrate", self._config.machine.travel_feedrate),
            ("Safe Z", "safe_z", self._config.z_heights.safe),
            ("Board Z", "board_z", self._config.z_heights.board_surface),
            ("Feeder Z", "feeder_z", self._config.z_heights.feeder_pick),
            ("Discard Z", "discard_z", self._config.z_heights.discard),
            ("X Limit", "x_max", self._config.machine.axis_limits.x.max),
            ("Y Limit", "y_max", self._config.machine.axis_limits.y.max),
            ("Board Origin X", "origin_x", self._config.board.origin.x),
            ("Board Origin Y", "origin_y", self._config.board.origin.y),
            ("Board Place Z", "place_z", self._config.board.place_z),
            ("Camera X", "cam_x", self._config.camera.position.x),
            ("Camera Y", "cam_y", self._config.camera.position.y),
        ]
        for i, (label, key, val) in enumerate(fields):
            r, c = divmod(i, 2)
            ttk.Label(mc_f, text=f"{label}:", foreground=FG_DIM).grid(row=r, column=c * 2, sticky="e", padx=(6, 2), pady=1)
            var = tk.StringVar(value=str(val))
            self._cfg_vars[key] = var
            ttk.Entry(mc_f, textvariable=var, width=8).grid(row=r, column=c * 2 + 1, sticky="w", padx=(0, 6), pady=1)

        endstop_var = tk.BooleanVar(value=self._config.machine.disable_software_endstops)
        self._cfg_vars["disable_endstops"] = endstop_var
        ttk.Checkbutton(mc_f, text="Disable Software Endstops (M211 S0)", variable=endstop_var).grid(
            row=len(fields) // 2 + 1, column=0, columnspan=4, sticky="w", padx=6, pady=2)

        # Board & Vision (bottom-right)
        bv_f = ttk.LabelFrame(tab, text="Vision")
        bv_f.grid(row=1, column=1, sticky="nsew", padx=(2, 4), pady=(2, 4))
        vis_fields = [
            ("Max Passes", "max_passes", self._config.vision.max_passes),
            ("Max Linear (mm)", "max_linear", self._config.vision.max_linear_offset_mm),
            ("Max Angular (deg)", "max_angular", self._config.vision.max_angular_offset_deg),
            ("Converge (mm)", "converge_mm", self._config.vision.converge_mm),
            ("Camera Device", "cam_dev", self._config.camera.device_index),
            ("Settle Time (ms)", "settle_ms", self._config.camera.settle_time_ms),
        ]
        for i, (label, key, val) in enumerate(vis_fields):
            r, c = divmod(i, 2)
            ttk.Label(bv_f, text=f"{label}:", foreground=FG_DIM).grid(row=r, column=c * 2, sticky="e", padx=(6, 2), pady=1)
            var = tk.StringVar(value=str(val))
            self._cfg_vars[key] = var
            ttk.Entry(bv_f, textvariable=var, width=8).grid(row=r, column=c * 2 + 1, sticky="w", padx=(0, 6), pady=1)

        # Save button
        save_f = ttk.Frame(tab)
        save_f.grid(row=2, column=0, columnspan=2, sticky="e", padx=4, pady=4)
        ttk.Button(save_f, text="Save Config", style="Accent.TButton", command=self._save_config).pack(side=tk.RIGHT)

        return tab

    # ── Tab: Camera ───────────────────────────────────────────

    def _build_camera_tab(self) -> ttk.Frame:
        tab = ttk.Frame(self._nb)
        tab.columnconfigure(0, weight=1)
        tab.rowconfigure(0, weight=1)

        self._camera_label = tk.Label(tab, text="Camera off" if HAS_CAMERA else "opencv-python not installed",
                                      bg="#e8e8e8", fg=FG_DIM, font=self._mono)
        self._camera_label.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)

        btn = ttk.Frame(tab)
        btn.grid(row=1, column=0, sticky="ew", padx=4, pady=4)
        ttk.Label(btn, text="Device:").pack(side=tk.LEFT, padx=(0, 2))
        self._cam_idx_var = tk.IntVar(value=self._config.camera.device_index)
        cam_spin = ttk.Spinbox(btn, from_=0, to=10, textvariable=self._cam_idx_var, width=3)
        cam_spin.pack(side=tk.LEFT, padx=(0, 6))
        self._cam_toggle_btn = ttk.Button(btn, text="Start Camera", command=self._toggle_camera,
                                          state=tk.NORMAL if HAS_CAMERA else tk.DISABLED)
        self._cam_toggle_btn.pack(side=tk.LEFT, padx=2)
        ttk.Button(btn, text="Capture", command=self._capture_frame,
                   state=tk.NORMAL if HAS_CAMERA else tk.DISABLED).pack(side=tk.LEFT, padx=2)
        cam = self._config.camera.position
        ttk.Button(btn, text="Go to Camera", style="Accent.TButton",
                   command=lambda: self._send_cmd(("__GOTO__", cam.x, cam.y, None))).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn, text="Calibrate UPP...", command=self._on_calibrate_upp).pack(side=tk.LEFT, padx=6)

        # Second row: bbox overlay + test vision
        btn2 = ttk.Frame(tab)
        btn2.grid(row=2, column=0, sticky="ew", padx=4, pady=2)
        self._show_bbox_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(btn2, text="Show detection bbox (live)",
                        variable=self._show_bbox_var).pack(side=tk.LEFT, padx=4)
        ttk.Label(btn2, text="|", foreground=FG_DIM).pack(side=tk.LEFT, padx=4)
        ttk.Label(btn2, text="Test IC:").pack(side=tk.LEFT, padx=(6, 2))
        self._test_ic_var = tk.StringVar(value=list(self._config.parts.keys())[0] if self._config.parts else "")
        test_combo = ttk.Combobox(btn2, textvariable=self._test_ic_var,
                                  values=list(self._config.parts.keys()), width=12, state="readonly")
        test_combo.pack(side=tk.LEFT, padx=2)
        ttk.Button(btn2, text="Test Vision", style="Accent.TButton",
                   command=self._on_test_vision).pack(side=tk.LEFT, padx=4)
        ttk.Button(btn2, text="Apply Correction", style="Green.TButton",
                   command=self._on_apply_correction).pack(side=tk.LEFT, padx=4)
        ttk.Button(btn2, text="Tune Vision...",
                   command=self._on_tune_vision).pack(side=tk.LEFT, padx=4)

        # UPP + detection info display
        upp = self._config.camera.units_per_pixel
        self._upp_label = ttk.Label(tab, text=f"UPP: X={upp.x:.8f}  Y={upp.y:.8f} mm/px", foreground=FG_DIM)
        self._upp_label.grid(row=3, column=0, sticky="w", padx=8, pady=2)
        self._detection_label = ttk.Label(tab, text="", foreground=ACCENT, font=self._mono)
        self._detection_label.grid(row=4, column=0, sticky="w", padx=8, pady=2)

        return tab

    # ── Tab: Visualizer ───────────────────────────────────────

    def _build_visualizer_tab(self) -> ttk.Frame:
        tab = ttk.Frame(self._nb)
        tab.columnconfigure(0, weight=1)
        tab.rowconfigure(0, weight=1)

        # Top: 3D matplotlib canvas embedded in tkinter
        viz_f = ttk.LabelFrame(tab, text="3D Toolpath Preview")
        viz_f.grid(row=0, column=0, sticky="nsew", padx=4, pady=(4, 2))
        self._viz_canvas_frame = viz_f
        self._viz_canvas = None
        self._viz_label = ttk.Label(viz_f, text="Run a dry-run job or load G-code to preview",
                                    foreground=FG_DIM)
        self._viz_label.pack(expand=True)

        # Bottom: Timing controls
        time_f = ttk.LabelFrame(tab, text="Timing Estimation")
        time_f.grid(row=1, column=0, sticky="ew", padx=4, pady=(2, 4))

        # Timing table
        tr = ttk.Frame(time_f)
        tr.pack(fill=tk.X, padx=4, pady=4)
        tcols = ("feedrate", "theoretical", "measured", "actual_mm_s")
        self._timing_tree = ttk.Treeview(tr, columns=tcols, show="headings", height=4)
        for c, w, text in zip(tcols, (80, 100, 100, 100),
                              ("Feedrate", "Theoretical mm/s", "Measured mm/s", "Ratio")):
            self._timing_tree.heading(c, text=text)
            self._timing_tree.column(c, width=w, anchor="center")
        self._timing_tree.pack(fill=tk.X)

        # Pre-populate with dummy values accounting for acceleration loss.
        # Real-world stepper motion reaches ~50-82% of theoretical due to
        # accel/decel ramps and firmware overhead. Values interpolated from
        # a sparse lookup table.
        self._timing_data = {}
        self._dummy_ratios = {
            100: 0.50, 300: 0.60, 500: 0.65, 900: 0.70, 1000: 0.72,
            1500: 0.75, 2000: 0.78, 3000: 0.80, 5000: 0.82,
        }
        for f_val in sorted(set([100, 300, 500, 900, 1000, 1500, 2000, 3000, 5000,
                                 int(self._config.machine.travel_feedrate),
                                 int(self._config.machine.z_feedrate)])):
            theo = round(f_val / 60, 1)
            ratio = _interp_ratio(f_val, self._dummy_ratios)
            measured = round(theo * ratio, 1)
            self._timing_tree.insert("", tk.END, values=(
                f"F{f_val}", f"{theo}", f"{measured}*", f"{ratio:.2f}x"))
            self._timing_data[f_val] = {"theoretical": theo, "measured": measured, "dummy": True}

        # Buttons
        btn_f = ttk.Frame(time_f)
        btn_f.pack(fill=tk.X, padx=4, pady=4)
        ttk.Button(btn_f, text="Auto-Measure", style="Accent.TButton",
                   command=self._on_auto_measure).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_f, text="Estimate Job Time",
                   command=self._on_estimate_time).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_f, text="Show 3D Plot",
                   command=self._on_show_3d).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_f, text="Animate 3D",
                   command=self._on_animate_3d).pack(side=tk.LEFT, padx=2)

        self._time_estimate_label = ttk.Label(time_f, text="", foreground=ACCENT)
        self._time_estimate_label.pack(padx=4, pady=2)

        return tab

    def _on_auto_measure(self):
        """Measure actual mm/s for each feedrate by timing real moves."""
        if not self._running:
            messagebox.showwarning("Not Connected", "Connect to the machine first.")
            return

        self._append_console("Starting auto-measure calibration...", "cmd")
        self._send_cmd("__AUTO_MEASURE__")

    def _on_estimate_time(self):
        """Estimate total job time from current placements + feedrate data."""
        from nanopnp.visualizer import parse_gcode, summarize
        import io

        # Generate G-code in dry-run to estimate
        from nanopnp.serial_comm import SerialConnection
        from nanopnp.motion import MotionController
        from nanopnp.paste_dispenser import PasteDispenser
        from nanopnp.pnp_engine import PnPEngine
        from nanopnp.feeder import FeederManager

        serial = SerialConnection(self._config.machine.serial_port,
                                  self._config.machine.baud_rate,
                                  dry_run=True, gcode_file=None)
        serial.connect(disable_endstops=self._config.machine.disable_software_endstops)
        motion = MotionController(serial, self._config)
        feeders = FeederManager(self._config)
        paste = PasteDispenser(motion, self._config)
        engine = PnPEngine(self._config, motion, feeders, vision_enabled=False)

        # Capture G-code
        gcode_lines = []
        orig_on_cmd = serial.on_command
        serial.on_command = lambda cmd, resp: gcode_lines.append(cmd)

        placements = self._config.enabled_placements()
        if placements:
            engine.run_job(placements, paste_dispenser=paste, paste_first=True)

        serial.on_command = orig_on_cmd
        serial.close()

        if not gcode_lines:
            self._time_estimate_label.config(text="No enabled placements to estimate")
            return

        gcode_text = "\n".join(gcode_lines)
        moves = parse_gcode(gcode_text)

        # Calculate time using measured or theoretical speeds
        total_time_s = 0.0
        for m in moves:
            dx = m.x1 - m.x0
            dy = m.y1 - m.y0
            dz = m.z1 - m.z0
            dist = (dx**2 + dy**2 + dz**2)**0.5

            # Use interpolated ratio from the dummy/measured table
            f_int = int(m.feedrate)
            td = self._timing_data.get(f_int)
            if td and td["measured"]:
                speed_mms = td["measured"]
            else:
                # Interpolate from the ratio table
                theo = m.feedrate / 60
                ratio = _interp_ratio(m.feedrate, self._dummy_ratios)
                speed_mms = theo * ratio

            if speed_mms > 0 and dist > 0.001:
                total_time_s += dist / speed_mms

        # Add dwell times
        n_picks = len(placements)
        dwell_s = (self._config.vacuum.pick_dwell_ms + self._config.vacuum.place_dwell_ms) / 1000
        total_time_s += n_picks * dwell_s

        stats = summarize(moves)
        mins = int(total_time_s // 60)
        secs = total_time_s % 60
        cph = (n_picks / total_time_s * 3600) if total_time_s > 0 else 0

        self._time_estimate_label.config(
            text=f"Estimated: {mins}m {secs:.0f}s for {n_picks} component(s) | "
                 f"{stats['travel_mm']}mm travel, {stats['paste_mm']}mm paste | "
                 f"~{cph:.0f} CPH"
        )
        self._append_console(f"Time estimate: {mins}m {secs:.0f}s, {cph:.0f} CPH", "cmd")

        # Store moves for 3D visualization
        self._last_gcode_moves = moves
        self._last_gcode_text = gcode_text

    def _on_show_3d(self):
        """Open 3D static plot in a separate matplotlib window."""
        moves = getattr(self, "_last_gcode_moves", None)
        if not moves:
            self._on_estimate_time()  # generate moves first
            moves = getattr(self, "_last_gcode_moves", None)
        if not moves:
            return

        from nanopnp.visualizer import plot_gcode_3d
        import matplotlib.pyplot as plt
        plot_gcode_3d(moves, config=self._config, title="NanoPnP 3D Toolpath")
        plt.tight_layout()
        plt.show()

    def _on_animate_3d(self):
        """Open animated 3D plot in a separate matplotlib window."""
        moves = getattr(self, "_last_gcode_moves", None)
        if not moves:
            self._on_estimate_time()
            moves = getattr(self, "_last_gcode_moves", None)
        if not moves:
            return

        from nanopnp.visualizer import animate_gcode_3d
        animate_gcode_3d(moves, config=self._config, title="NanoPnP 3D Toolpath")

    # ── Tab: Console ──────────────────────────────────────────

    def _build_console_tab(self) -> ttk.Frame:
        tab = ttk.Frame(self._nb)
        tab.rowconfigure(0, weight=1)
        tab.columnconfigure(0, weight=1)

        self._console_text = tk.Text(tab, font=self._mono, bg=BG_INPUT, fg=FG,
                                     insertbackground=FG, state=tk.DISABLED, wrap=tk.WORD)
        scroll = ttk.Scrollbar(tab, command=self._console_text.yview)
        self._console_text.configure(yscrollcommand=scroll.set)
        self._console_text.grid(row=0, column=0, sticky="nsew", padx=(4, 0), pady=4)
        scroll.grid(row=0, column=1, sticky="ns", padx=(0, 4), pady=4)
        self._console_text.tag_configure("cmd", foreground=ACCENT)
        self._console_text.tag_configure("resp", foreground=FG_DIM)
        self._console_text.tag_configure("err", foreground=RED)

        ef = ttk.Frame(tab)
        ef.grid(row=1, column=0, columnspan=2, sticky="ew", padx=4, pady=(0, 4))
        self._gcode_entry = ttk.Entry(ef, font=self._mono)
        self._gcode_entry.pack(side=tk.LEFT, expand=True, fill=tk.X)
        self._gcode_entry.bind("<Return>", lambda e: self._on_send_gcode())
        self._gcode_entry.bind("<Up>", self._history_prev)
        self._gcode_entry.bind("<Down>", self._history_next)
        ttk.Button(ef, text="Send", command=self._on_send_gcode, width=6).pack(side=tk.RIGHT, padx=(4, 0))
        ttk.Button(ef, text="Clear", command=self._clear_console, width=6).pack(side=tk.RIGHT, padx=(4, 0))

        return tab

    def _clear_console(self):
        self._console_text.config(state=tk.NORMAL)
        self._console_text.delete("1.0", tk.END)
        self._console_text.config(state=tk.DISABLED)

    # ── Keyboard shortcuts ────────────────────────────────────

    _NO_JOG_WIDGETS = (ttk.Entry, tk.Entry, tk.Text, ttk.Combobox, ttk.Spinbox)

    def _jog_key(self, axis, direction, event):
        """Only jog if focus is not on an editable widget."""
        if isinstance(event.widget, self._NO_JOG_WIDGETS):
            return  # let the widget handle the key
        self._jog(axis, direction)

    def _bind_keys(self):
        self._root.bind("<Left>", lambda e: self._jog_key("X", -1, e))
        self._root.bind("<Right>", lambda e: self._jog_key("X", 1, e))
        self._root.bind("<Up>", lambda e: self._jog_key("Y", 1, e))
        self._root.bind("<Down>", lambda e: self._jog_key("Y", -1, e))
        self._root.bind("<Prior>", lambda e: self._jog_key("Z", -1, e))
        self._root.bind("<Next>", lambda e: self._jog_key("Z", 1, e))
        self._root.bind("<Escape>", lambda e: self._on_estop())
        self._root.bind("<Home>", lambda e: self._on_home())

    # ── Connection ────────────────────────────────────────────

    def _refresh_ports(self):
        try:
            from serial.tools.list_ports import comports
            ports = [p.device for p in comports()]
        except ImportError:
            ports = []
        if not ports:
            ports = [self._config.machine.serial_port]
        self._port_combo["values"] = ports

    def _on_connect(self):
        port = self._port_var.get()
        baud = self._config.machine.baud_rate
        dry_run = self._dry_run_var.get()
        self._connect_btn.config(state=tk.DISABLED)
        self._status_dot.itemconfig("dot", fill=ORANGE)

        def _do_connect():
            try:
                self._serial = SerialConnection(port, baud, dry_run=dry_run)
                self._serial.on_command = self._on_serial_command
                self._serial.connect(disable_endstops=self._config.machine.disable_software_endstops)
                self._motion = MotionController(self._serial, self._config)
                self._feeders = FeederManager(self._config)
                self._paste = PasteDispenser(self._motion, self._config)
                self._engine = PnPEngine(self._config, self._motion, self._feeders,
                                            vision_enabled=self._vision_enabled_var.get())
                self._engine.vision_apply = self._vision_apply_var.get()
                # Persist last successful connection for auto-reconnect next launch
                if not dry_run:
                    self._save_last_connection(port, baud)
                self._result_queue.put(("connected", True))
            except Exception as e:
                self._result_queue.put(("connected", str(e)))

        threading.Thread(target=_do_connect, daemon=True).start()

    def _save_last_connection(self, port: str, baud: int):
        """Persist last successful connection details to .last_connection.json."""
        import json
        from pathlib import Path
        try:
            state_file = Path(self._config_path).parent / ".last_connection.json"
            state_file.write_text(json.dumps({"port": port, "baud": baud}))
        except Exception as e:
            log_msg = f"Failed to save connection state: {e}"
            self._result_queue.put(("console_err", log_msg))

    def _load_last_connection(self) -> dict | None:
        """Load last successful connection from .last_connection.json."""
        import json
        from pathlib import Path
        try:
            state_file = Path(self._config_path).parent / ".last_connection.json"
            if state_file.exists():
                return json.loads(state_file.read_text())
        except Exception:
            pass
        return None

    def _on_disconnect(self):
        # Update UI immediately — don't wait for serial close
        self._polling = False
        self._running = False
        self._status_dot.itemconfig("dot", fill=RED)
        self._connect_btn.config(state=tk.NORMAL)
        self._disconnect_btn.config(state=tk.DISABLED)

        # Close serial in background thread to avoid blocking the GUI
        serial_ref = self._serial
        self._serial = self._motion = self._feeders = self._paste = self._engine = None

        if serial_ref:
            def _do_close():
                try:
                    serial_ref.close()
                except Exception:
                    pass
            threading.Thread(target=_do_close, daemon=True).start()

    def _on_estop(self):
        if self._engine:
            self._engine.stop()
        if self._serial and self._serial.connected:
            try:
                self._serial.send("M410", timeout=2)
            except Exception:
                pass
        self._append_console("!! E-STOP !!", "err")

    # ── Worker ────────────────────────────────────────────────

    def _worker_loop(self):
        while self._running:
            try:
                item = self._cmd_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            if not self._motion:
                continue
            try:
                if item == "__POS__":
                    self._result_queue.put(("pos", self._motion.get_position()))
                elif item == "__HOME__":
                    self._motion.home()
                    self._result_queue.put(("pos", self._motion.get_position()))
                elif item == "__VAC_ON__":
                    self._motion.vacuum_on()
                    self._result_queue.put(("vac", True))
                elif item == "__VAC_OFF__":
                    self._motion.vacuum_off()
                    self._result_queue.put(("vac", False))
                elif item == "__T0__":
                    self._motion.tool_select(0)
                    self._result_queue.put(("tool", 0))
                elif item == "__T1__":
                    self._motion.tool_select(1)
                    self._result_queue.put(("tool", 1))
                elif item == "__SET_ORIGIN__":
                    self._serial.send("G92 X0 Y0 Z0 E0")
                    self._result_queue.put(("pos", self._motion.get_position()))
                elif isinstance(item, tuple) and item[0] == "__CORRECTION__":
                    _, ddx, ddy, dde = item
                    # Apply correction as a relative move: G91 → G1 → G90
                    self._serial.send("G91")  # switch to relative mode
                    self._serial.send(f"G1 X{ddx:.4f} Y{ddy:.4f} E{dde:.4f} F500")
                    self._serial.send("M400")
                    self._serial.send("G90")  # back to absolute mode
                    self._result_queue.put(("pos", self._motion.get_position()))
                elif item == "__AUTO_MEASURE__":
                    self._result_queue.put(("console_cmd", ("Auto-measure", "Starting...")))
                    import time as _time
                    results = {}
                    test_dist = 20.0  # mm to travel for each test
                    for f_val in sorted(self._timing_data.keys()):
                        # Move to a safe start position
                        self._motion.safe_move_to(10, 10)
                        self._motion.wait()
                        # Time a known-distance move
                        t0 = _time.monotonic()
                        self._motion.move_to(x=10 + test_dist, feedrate=float(f_val))
                        self._motion.wait()
                        t1 = _time.monotonic()
                        elapsed = t1 - t0
                        actual_mms = test_dist / elapsed if elapsed > 0.01 else 0
                        results[f_val] = round(actual_mms, 1)
                        self._result_queue.put(("console_cmd",
                            (f"F{f_val}", f"{test_dist}mm in {elapsed:.2f}s = {actual_mms:.1f} mm/s")))
                    # Return home
                    self._motion.safe_move_to(0, 0)
                    self._result_queue.put(("timing", results))
                    self._result_queue.put(("pos", self._motion.get_position()))
                elif isinstance(item, tuple) and item[0] in ("__RUN_PASTE_FILES__", "__RUN_PASTE_SELECTED__", "__RUN_PASTE_DRY__"):
                    self._polling = False
                    if item[0] == "__RUN_PASTE_DRY__":
                        _, edge_cuts, paste_gbr, refs = item
                        label = f"Solder dry {', '.join(refs)}..." if refs else "Solder dry run..."
                        self._result_queue.put(("job_status", label))
                        count = self._paste.dispense_paste(edge_cuts, paste_gbr, refs=refs, no_dispense=True)
                    elif item[0] == "__RUN_PASTE_SELECTED__":
                        _, edge_cuts, paste_gbr, refs = item
                        self._result_queue.put(("job_status", f"Soldering {', '.join(refs)}..."))
                        count = self._paste.dispense_paste(edge_cuts, paste_gbr, refs=refs)
                    else:
                        _, edge_cuts, paste_gbr = item
                        self._result_queue.put(("job_status", "Dispensing paste..."))
                        count = self._paste.dispense_paste(edge_cuts, paste_gbr)
                    self._result_queue.put(("job_status", f"Paste done: {count} components"))
                    self._result_queue.put(("pos", self._motion.get_position()))
                    self._polling = True
                    self._poll_position()
                elif item == "__RUN_PLACE__" or (isinstance(item, tuple) and item[0] == "__RUN_JOB_FILES__"):
                    self._polling = False  # pause position polling during job
                    is_full_job = isinstance(item, tuple)
                    self._engine.vision_enabled = self._vision_enabled_var.get()
                    self._engine.vision_apply = self._vision_apply_var.get()
                    placements = self._config.enabled_placements()
                    self._result_queue.put(("job_status", "Running..."))

                    def _prog(i, total, ref, status):
                        pct = (i / total * 100) if total > 0 else 0
                        self._result_queue.put(("job_progress", (pct, f"{ref}: {status}")))

                    if is_full_job:
                        _, edge_cuts, paste_gbr, _pos_path = item
                        # Paste phase uses the explicit edge_cuts + paste paths.
                        self._paste.dispense_paste(edge_cuts, paste_gbr)
                        result = self._engine.run_job(placements, progress_cb=_prog)
                    else:
                        result = self._engine.run_job(placements, progress_cb=_prog)

                    self._result_queue.put(("job_done", result))
                    self._result_queue.put(("pos", self._motion.get_position()))
                    if self._feeders:
                        self._result_queue.put(("feeders", self._feeders.status()))
                    self._polling = True  # resume polling
                    self._poll_position()
                elif isinstance(item, tuple) and item[0] == "__GOTO__":
                    _, x, y, z = item
                    self._motion.safe_move_to(x, y, z=z)
                    self._result_queue.put(("pos", self._motion.get_position()))
                elif isinstance(item, tuple) and item[0] == "__JOG__":
                    _, axis, target = item
                    self._motion.move_to(**{axis.lower(): target})
                    self._result_queue.put(("pos", self._motion.get_position()))
                else:
                    self._motion.send_raw(str(item))
            except Exception as e:
                msg = str(e)
                self._result_queue.put(("console_err", msg))
                # Auto-disconnect on serial errors (device unplugged, etc.)
                if "Device not configured" in msg or "write failed" in msg or "not connected" in msg.lower():
                    self._result_queue.put(("serial_dead", msg))
                    break  # exit worker loop

    def _monitor_loop(self):
        """Background thread: check every 1s if the serial port still exists."""
        import os
        import time as _time
        while self._running and self._serial is not None:
            _time.sleep(1.0)
            if not self._running:
                break
            if self._serial is None:
                break
            if self._serial.dry_run:
                continue  # no real port to check
            port = self._serial._port
            # Check if the port device file still exists
            if port and not os.path.exists(port):
                self._result_queue.put(("serial_lost", f"Port {port} no longer exists"))
                break

    def _on_serial_command(self, cmd, response):
        # Hide noisy periodic commands from the console
        cmd_stripped = cmd.strip().upper()
        if cmd_stripped.startswith("M114") or cmd_stripped == "M400":
            return
        first = response.split("\n")[0] if response else ""
        self._result_queue.put(("console_cmd", (cmd, first)))

    # ── Polling ───────────────────────────────────────────────

    def _poll_results(self):
        try:
            for _ in range(50):
                t, d = self._result_queue.get_nowait()
                if t == "connected":
                    if d is True:
                        self._running = True
                        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
                        self._worker.start()
                        self._monitor = threading.Thread(target=self._monitor_loop, daemon=True)
                        self._monitor.start()
                        self._polling = True
                        self._poll_position()
                        self._status_dot.itemconfig("dot", fill=GREEN)
                        self._disconnect_btn.config(state=tk.NORMAL)
                        self._populate_feeders()
                    else:
                        messagebox.showerror("Connection Error", str(d))
                        self._connect_btn.config(state=tk.NORMAL)
                        self._status_dot.itemconfig("dot", fill=RED)
                elif t == "pos":
                    self._update_pos(d)
                elif t == "console_cmd":
                    self._append_console(f">> {d[0]}", "cmd")
                    self._append_console(f"<< {d[1]}", "resp")
                elif t == "console_err":
                    self._append_console(f"ERROR: {d}", "err")
                elif t == "vac":
                    self._vac_label.config(text=f"VAC:{'ON' if d else 'OFF'}", foreground=GREEN if d else FG_DIM)
                elif t == "tool":
                    self._tool_label.config(text=f"T{d}", foreground=ACCENT)
                    self._tool_var.set(d)
                elif t in ("serial_dead", "serial_lost"):
                    # Serial connection lost — clean up like a normal disconnect
                    self._polling = False
                    self._running = False
                    if self._serial:
                        try:
                            self._serial.close()
                        except Exception:
                            pass
                    self._serial = self._motion = self._feeders = self._paste = self._engine = None
                    self._status_dot.itemconfig("dot", fill=RED)
                    self._connect_btn.config(state=tk.NORMAL)
                    self._disconnect_btn.config(state=tk.DISABLED)
                    self._append_console(f"!! Serial disconnected: {d}", "err")
                    messagebox.showwarning("Disconnected",
                        f"Serial connection lost:\n\n{d}\n\n"
                        "Check USB cable and board power, then click Connect to reconnect.")
                elif t == "feeders":
                    self._update_feeder_table(d)
                elif t == "timing":
                    # Update timing table with measured values
                    for item in self._timing_tree.get_children():
                        self._timing_tree.delete(item)
                    for f_val in sorted(self._timing_data.keys()):
                        theo = self._timing_data[f_val]["theoretical"]
                        measured = d.get(f_val)
                        if measured:
                            self._timing_data[f_val]["measured"] = measured
                            ratio = round(measured / theo, 2) if theo > 0 else 0
                            self._timing_tree.insert("", tk.END, values=(
                                f"F{f_val}", f"{theo}", f"{measured}", f"{ratio}x"))
                        else:
                            self._timing_tree.insert("", tk.END, values=(
                                f"F{f_val}", f"{theo}", "--", "--"))
                    self._append_console("Auto-measure complete", "cmd")
                elif t == "remote_status":
                    # Jetson poller tick — update toolbar label, no blocking.
                    self._update_target_status_label(d)
                elif t == "remote_send_done":
                    if d.ok:
                        self._append_console(f"[send] {d.summary}", "cmd")
                        self._progress_label.config(text="Sent → watcher will pick up", foreground=GREEN)
                    else:
                        self._append_console(f"[send] failed: {d.error}", "err")
                        self._progress_label.config(text=f"Send failed: {d.error[:60]}", foreground=RED)
                elif t == "job_status":
                    self._progress_label.config(text=str(d), foreground=ACCENT)
                elif t == "job_progress":
                    self._progress_var.set(d[0])
                    self._progress_label.config(text=d[1])
                elif t == "job_done":
                    r = d
                    self._progress_var.set(100)
                    self._progress_label.config(text=f"Done: {r.placed}/{r.total} in {r.duration_s}s ({r.cph:.0f} CPH)", foreground=GREEN)
                    for item in self._result_tree.get_children():
                        self._result_tree.delete(item)
                    for pr in r.placements:
                        tag = "ok" if pr.success else "fail"
                        self._result_tree.insert("", tk.END, values=(
                            pr.ref, pr.part_id,
                            f"({pr.nominal[0]:.1f},{pr.nominal[1]:.1f})",
                            f"({pr.corrected[0]:.1f},{pr.corrected[1]:.1f})",
                            f"{pr.corrections[0]:.3f}", f"{pr.corrections[1]:.3f}",
                            f"{pr.corrections[2]:.1f}",
                            "OK" if pr.success else pr.error), tags=(tag,))
        except queue.Empty:
            pass
        self._root.after(50, self._poll_results)

    def _poll_position(self):
        if self._polling and self._running:
            self._cmd_queue.put("__POS__")
        if self._polling:
            self._root.after(500, self._poll_position)

    def _update_pos(self, pos):
        self._pos = pos
        for a in ["X", "Y", "Z", "E"]:
            if a in pos:
                self._pos_labels[a].config(text=f"{pos[a]:7.3f}")

    def _append_console(self, text, tag=""):
        self._console_text.config(state=tk.NORMAL)
        self._console_text.insert(tk.END, text + "\n", tag if tag else ())
        self._console_text.see(tk.END)
        self._console_text.config(state=tk.DISABLED)

    # ── Handlers ──────────────────────────────────────────────

    def _send_cmd(self, cmd):
        if self._running:
            self._cmd_queue.put(cmd)

    def _jog(self, axis, direction):
        if not self._running:
            return
        target = self._pos.get(axis, 0.0) + self._step_size.get() * direction
        self._send_cmd(("__JOG__", axis, target))

    def _on_home(self):
        self._send_cmd("__HOME__")

    def _on_goto(self):
        r = simpledialog.askstring("GoTo", "X Y [Z]:", parent=self._root)
        if not r:
            return
        parts = r.split()
        try:
            x, y = float(parts[0]), float(parts[1])
            z = float(parts[2]) if len(parts) > 2 else None
        except (ValueError, IndexError):
            return
        self._send_cmd(("__GOTO__", x, y, z))

    def _on_send_gcode(self):
        cmd = self._gcode_entry.get().strip()
        if cmd:
            self._cmd_history.append(cmd)
            self._history_idx = len(self._cmd_history)
            self._send_cmd(cmd)
            self._gcode_entry.delete(0, tk.END)

    def _history_prev(self, event):
        if not self._cmd_history:
            return "break"
        self._history_idx = max(0, self._history_idx - 1)
        self._gcode_entry.delete(0, tk.END)
        self._gcode_entry.insert(0, self._cmd_history[self._history_idx])
        return "break"

    def _history_next(self, event):
        if not self._cmd_history:
            return "break"
        self._history_idx = min(len(self._cmd_history), self._history_idx + 1)
        self._gcode_entry.delete(0, tk.END)
        if self._history_idx < len(self._cmd_history):
            self._gcode_entry.insert(0, self._cmd_history[self._history_idx])
        return "break"

    def _on_run_paste(self):
        if not self._job_inputs.can_paste:
            return
        if self._target_mode.get() == "jetson":
            self._send_to_jetson("paste")
            return
        if not self._running:
            return
        self._send_cmd(("__RUN_PASTE_FILES__",
                        str(self._job_inputs.edge_cuts or ""),
                        str(self._job_inputs.paste)))

    def _on_solder_selected(self):
        """Dispense paste only for selected placement(s) in the table."""
        sel = self._place_tree.selection()
        if not sel:
            self._append_console("Select placement(s) first", "err")
            return
        if not self._job_inputs.can_paste:
            self._append_console("Load F_Paste file first", "err")
            return
        if not self._running:
            self._append_console("Connect to machine first", "err")
            return
        refs = []
        for item in sel:
            vals = self._place_tree.item(item, "values")
            refs.append(str(vals[0]).strip())
        self._send_cmd(("__RUN_PASTE_SELECTED__",
                        str(self._job_inputs.edge_cuts or ""),
                        str(self._job_inputs.paste),
                        refs))

    def _on_solder_dry(self):
        """Run solder path without dispensing — movement only."""
        if not self._job_inputs.can_paste:
            self._append_console("Load F_Paste file first", "err")
            return
        if not self._running:
            self._append_console("Connect to machine first", "err")
            return
        sel = self._place_tree.selection()
        refs = [str(self._place_tree.item(item, "values")[0]).strip() for item in sel] if sel else None
        self._send_cmd(("__RUN_PASTE_DRY__",
                        str(self._job_inputs.edge_cuts or ""),
                        str(self._job_inputs.paste),
                        refs))

    def _on_run_place(self):
        if not self._job_inputs.can_place:
            return
        if self._target_mode.get() == "jetson":
            self._send_to_jetson("place")
            return
        if self._running:
            self._send_cmd("__RUN_PLACE__")

    def _on_run_full_job(self):
        if self._job_inputs.mode != "full":
            return
        if self._target_mode.get() == "jetson":
            self._send_to_jetson("full")
            return
        if self._running:
            self._send_cmd(("__RUN_JOB_FILES__",
                            str(self._job_inputs.edge_cuts),
                            str(self._job_inputs.paste),
                            str(self._job_inputs.pos)))

    def _on_stop_job(self):
        if self._engine:
            self._engine.stop()
        self._on_estop()

    # ── Auto-save ─────────────────────────────────────────────

    def _auto_save(self):
        """Debounced save — waits 500ms after last edit before writing to disk."""
        if self._save_timer is not None:
            self._root.after_cancel(self._save_timer)
        self._save_timer = self._root.after(500, self._do_save)

    def _do_save(self):
        self._save_timer = None
        # Always write config.json (machine settings, feeders, placements).
        cfg = self._config
        path = self._config_path
        def _bg_save():
            try:
                save_config(cfg, path)
            except Exception as e:
                self._result_queue.put(("console_err", f"Save config failed: {e}"))
        threading.Thread(target=_bg_save, daemon=True).start()
        # If a .pos file is loaded, also write placements back to it.
        if self._pos_source is not None:
            self._save_pos_source()

    # ── Target selector (Local ↔ Jetson) ─────────────────────

    def _on_target_changed(self) -> None:
        """Radio button callback — update button labels and start/stop poller."""
        mode = self._target_mode.get()
        if mode == "jetson":
            if not self._config.remote.is_configured():
                messagebox.showinfo(
                    "Configure Jetson",
                    "Set host + user first via the ⚙ button.",
                    parent=self._root,
                )
                self._target_mode.set("local")
                return
            self._btn_paste.config(text="Send Paste →")
            self._btn_place.config(text="Send Place →")
            self._btn_full.config(text="Send Full Job →")
            self._start_remote_poll()
        else:
            self._btn_paste.config(text="Solder Only")
            self._btn_place.config(text="Pick & Place Only")
            self._btn_full.config(text="Full Job (Solder + PnP)")
            self._stop_remote_poll()
            self._target_status_label.config(text="", foreground=FG_DIM)

    def _open_remote_settings(self) -> None:
        """Tiny form dialog to edit config.remote (host/user/path/key)."""
        r = self._config.remote
        dlg = tk.Toplevel(self._root)
        dlg.title("Jetson Target")
        dlg.geometry("380x260")
        dlg.transient(self._root)
        dlg.grab_set()

        fields = [
            ("Host",          "host",          r.host),
            ("User",          "user",          r.user),
            ("Incoming path", "incoming_path", r.incoming_path),
            ("Status path",   "status_path",   r.status_path),
            ("SSH key",       "ssh_key",       r.ssh_key),
            ("Port",          "port",          str(r.port)),
        ]
        vars_: dict[str, tk.StringVar] = {}
        for i, (label, key, val) in enumerate(fields):
            ttk.Label(dlg, text=f"{label}:").grid(row=i, column=0, sticky="e", padx=(10, 4), pady=3)
            v = tk.StringVar(value=val)
            vars_[key] = v
            ttk.Entry(dlg, textvariable=v, width=32).grid(row=i, column=1, sticky="w", padx=(0, 10), pady=3)

        def _save():
            try:
                port = int(vars_["port"].get() or "22")
            except ValueError:
                messagebox.showerror("Error", "Port must be an integer", parent=dlg)
                return
            from nanopnp.config import RemoteConfig
            self._config.remote = RemoteConfig(
                host=vars_["host"].get().strip(),
                user=vars_["user"].get().strip(),
                incoming_path=vars_["incoming_path"].get().strip() or "~/nanopnp/incoming",
                status_path=vars_["status_path"].get().strip() or "~/nanopnp/status.json",
                ssh_key=vars_["ssh_key"].get().strip(),
                port=port,
            )
            self._auto_save()
            dlg.destroy()

        ttk.Button(dlg, text="Save", style="Accent.TButton", command=_save).grid(
            row=len(fields), column=0, columnspan=2, pady=10)

    def _get_remote_target(self):
        """Build a RemoteTarget from config.remote."""
        from nanopnp.remote import RemoteTarget
        from pathlib import Path
        r = self._config.remote
        return RemoteTarget(
            host=r.host, user=r.user,
            incoming_path=r.incoming_path,
            status_path=r.status_path,
            ssh_key=Path(r.ssh_key) if r.ssh_key else None,
            port=r.port,
        )

    def _start_remote_poll(self) -> None:
        """Kick off the 3s status poller if not already running."""
        if getattr(self, "_remote_poll_id", None) is not None:
            return
        self._remote_poll_tick()

    def _stop_remote_poll(self) -> None:
        if getattr(self, "_remote_poll_id", None) is not None:
            try:
                self._root.after_cancel(self._remote_poll_id)
            except Exception:
                pass
            self._remote_poll_id = None

    def _remote_poll_tick(self) -> None:
        """One poll: fetch status in a background thread, schedule next tick."""
        if self._target_mode.get() != "jetson":
            self._remote_poll_id = None
            return
        target = self._get_remote_target()

        def _bg():
            from nanopnp.remote import fetch_status
            st = fetch_status(target)
            self._result_queue.put(("remote_status", st))
        threading.Thread(target=_bg, daemon=True).start()

        # Schedule next tick — 3s base cadence; the tick reschedules itself.
        self._remote_poll_id = self._root.after(3000, self._remote_poll_tick)

    def _update_target_status_label(self, st) -> None:
        """Called from the GUI thread via the result queue."""
        if not st.reachable:
            self._target_status_label.config(
                text=f"● Jetson: unreachable",
                foreground=RED,
            )
            return
        state_colours = {
            "idle":    ("Jetson: idle",    GREEN),
            "running": ("Jetson: running", ORANGE),
            "failed":  ("Jetson: failed",  RED),
            "unknown": ("Jetson: unknown", FG_DIM),
        }
        txt, colour = state_colours.get(st.state, ("Jetson: " + st.state, FG_DIM))
        if st.current_message:
            txt = f"{txt} — {st.current_message[:40]}"
        elif st.state == "failed" and st.last_run_at:
            txt = f"{txt} (@{st.last_run_at[-8:]})"
        elif st.last_run_result and st.state == "idle":
            txt = f"{txt} · last {st.last_run_result} {st.last_run_at[-8:]}"
        self._target_status_label.config(text="● " + txt, foreground=colour)

    # ── Remote send (Jetson target) ───────────────────────────

    def _send_to_jetson(self, inputs_subset: str) -> None:
        """Ship the current JobInputs (or a subset) to the Jetson incoming folder.

        inputs_subset ∈ {"paste", "place", "full"} — decides which files
        from self._job_inputs get included in the send. For "paste" we
        strip the .pos so the watcher runs paste mode; for "place" we
        strip edge_cuts + paste; for "full" we send all three.
        """
        if self._target_mode.get() != "jetson":
            return
        ji = self._job_inputs
        from nanopnp.job_inputs import JobInputs
        if inputs_subset == "paste":
            payload = JobInputs(edge_cuts=ji.edge_cuts, paste=ji.paste)
        elif inputs_subset == "place":
            payload = JobInputs(pos=ji.pos)
        else:
            payload = ji

        target = self._get_remote_target()

        def _bg():
            from nanopnp.remote import send_job
            result = send_job(target, payload,
                              on_progress=lambda m: self._result_queue.put(("console_cmd", ("[send]", m))))
            self._result_queue.put(("remote_send_done", result))
        threading.Thread(target=_bg, daemon=True).start()
        self._append_console(f"[send] uploading {inputs_subset} set to {target.user_host()}", "cmd")

    def _save_pos_source(self) -> None:
        """Write current placements back to the loaded .pos file.

        Inverts the shift applied on load — the .pos file stays in its
        original board-relative coordinate frame. Merges val/side/package
        from `_pos_aux` for round-trip fidelity.
        """
        if self._pos_source is None:
            return
        pos_path = self._pos_source
        ox = self._config.board.origin.x
        oy = self._config.board.origin.y

        rows: list[dict] = []
        for p in self._config.placements:
            aux = self._pos_aux.get(p.ref, {})
            rows.append({
                "reference": p.ref,
                "value":     aux.get("value", ""),
                "package":   aux.get("package", p.part_id),
                "x":         round(p.x - ox, 6),
                "y":         round(p.y - oy, 6),
                "rotation":  p.rotation,
                "side":      aux.get("side", "top"),
            })

        def _bg_save():
            try:
                from nanopnp.board_parser import write_pos_file
                write_pos_file(pos_path, rows)
                self._result_queue.put(("console_cmd",
                    ("[save]", f"wrote {len(rows)} placements -> {pos_path.name}")))
            except Exception as e:
                self._result_queue.put(("console_err", f"Save .pos failed: {e}"))
        threading.Thread(target=_bg_save, daemon=True).start()

    # ── Placements ────────────────────────────────────────────

    def _populate_placements(self):
        for item in self._place_tree.get_children():
            self._place_tree.delete(item)
        for p in self._config.placements:
            self._place_tree.insert("", tk.END, values=(p.ref, p.part_id, f"{p.x:.2f}", f"{p.y:.2f}",
                                    f"{p.rotation:.0f}", "Yes" if p.enabled else "No"),
                                    tags=("en" if p.enabled else "dis",))
        self._place_tree.tag_configure("en", foreground=FG)
        self._place_tree.tag_configure("dis", foreground=FG_DIM)

    def _toggle_placement_enable(self):
        sel = self._place_tree.selection()
        if not sel:
            return
        idx = self._place_tree.index(sel[0])
        if idx < len(self._config.placements):
            p = self._config.placements[idx]
            p.enabled = not p.enabled
        self._populate_placements()
        self._auto_save()

    def _edit_selected_placement(self):
        sel = self._place_tree.selection()
        if not sel:
            return
        idx = self._place_tree.index(sel[0])
        if idx >= len(self._config.placements):
            return
        p = self._config.placements[idx]

        dlg = tk.Toplevel(self._root)
        dlg.title(f"Edit {p.ref}")
        dlg.geometry("280x200")
        dlg.transient(self._root)
        dlg.grab_set()

        fields = {"ref": p.ref, "part_id": p.part_id, "x": p.x, "y": p.y, "rotation": p.rotation}
        vars_ = {}
        for i, (label, val) in enumerate(fields.items()):
            ttk.Label(dlg, text=f"{label}:").grid(row=i, column=0, sticky="e", padx=(8, 4), pady=2)
            var = tk.StringVar(value=str(val))
            vars_[label] = var
            ttk.Entry(dlg, textvariable=var, width=18).grid(row=i, column=1, sticky="w", padx=(0, 8), pady=2)

        def _save():
            try:
                p.ref = vars_["ref"].get()
                p.part_id = vars_["part_id"].get()
                p.x = float(vars_["x"].get())
                p.y = float(vars_["y"].get())
                p.rotation = float(vars_["rotation"].get())
            except ValueError as e:
                messagebox.showerror("Error", str(e), parent=dlg)
                return
            self._populate_placements()
            self._auto_save()
            dlg.destroy()

        ttk.Button(dlg, text="Save", style="Accent.TButton", command=_save).grid(
            row=len(fields), column=0, columnspan=2, pady=8)

    def _set_all_placements(self, enabled: bool):
        for p in self._config.placements:
            p.enabled = enabled
        self._populate_placements()
        self._auto_save()

    def _goto_selected_placement(self):
        sel = self._place_tree.selection()
        if not sel:
            return
        vals = self._place_tree.item(sel[0], "values")
        self._send_cmd(("__GOTO__", float(vals[2]), float(vals[3]), None))

    # ── Inputs panel ──────────────────────────────────────────

    def _pick_input(self, kind: str, filetypes: list[tuple[str, str]]) -> None:
        """File-picker handler for the Inputs panel. kind ∈ {edge_cuts, paste, pos}."""
        from pathlib import Path
        titles = {
            "edge_cuts": "Select Edge_Cuts.gbr",
            "paste":     "Select F_Paste.gbr",
            "pos":       "Select top.pos",
        }
        path = filedialog.askopenfilename(
            title=titles[kind], filetypes=filetypes, parent=self._root,
        )
        if not path:
            return

        p = Path(path)
        if kind == "pos":
            try:
                self._load_pos_source(p)
            except Exception as e:
                messagebox.showerror("Error", f"Failed to load .pos: {e}", parent=self._root)
                return
        self._set_job_input(kind, p)
        self._input_path_vars[kind].set(str(p))
        self._update_inputs_state()
        self._append_console(f"[input] {kind} = {p.name}", "cmd")

    def _clear_input(self, kind: str) -> None:
        from pathlib import Path
        self._input_path_vars[kind].set("")
        self._set_job_input(kind, None)
        if kind == "pos":
            self._pos_source = None
            self._pos_aux = {}
            self._config.placements.clear()
            self._populate_placements()
        self._update_inputs_state()

    def _set_job_input(self, kind: str, path: "Path | None") -> None:
        from nanopnp.job_inputs import JobInputs
        self._job_inputs = JobInputs(
            edge_cuts=path if kind == "edge_cuts" else self._job_inputs.edge_cuts,
            paste=path if kind == "paste" else self._job_inputs.paste,
            pos=path if kind == "pos" else self._job_inputs.pos,
        )

    def _update_inputs_state(self) -> None:
        """Re-gate the Paste/Place/Full buttons and refresh the mode label."""
        ji = self._job_inputs
        mode_labels = {
            "full":  ("FULL JOB — paste + place",        GREEN),
            "paste": ("PASTE only (missing .pos)",        ORANGE),
            "place": ("PLACE only (missing gerbers)",     ACCENT),
            "none":  ("NONE — load inputs to enable runs", FG_DIM),
        }
        text, colour = mode_labels[ji.mode]
        self._mode_label.config(text=f"Mode: {text}", foreground=colour)

        if hasattr(self, "_btn_paste"):
            self._btn_paste.state(["!disabled"] if ji.can_paste else ["disabled"])
            self._btn_place.state(["!disabled"] if ji.can_place else ["disabled"])
            self._btn_full.state(["!disabled"] if ji.mode == "full" else ["disabled"])

    def _load_pos_source(self, pos_path: "Path") -> None:
        """Parse a .pos and make it the live placement source.

        Replaces `config.placements` entirely. Subsequent edits round-trip
        back to this file via `_do_save` instead of to config.json.

        Uses `load_placements_from_pos` so the same mapping/skip rules
        apply here and in the CLI. If Edge_Cuts + F_Paste are already
        loaded in the Inputs panel, passes them through to enable the
        auto-detect coordinate path (instead of the bottom-left fallback).
        """
        from nanopnp.board_parser import _parse_pos_raw, load_placements_from_pos

        rows = _parse_pos_raw(pos_path)
        if not rows:
            raise ValueError(f"No placements found in {pos_path.name}")

        # Aux lookup preserves val/side/package for .pos round-trip writes.
        self._pos_aux = {
            r["reference"]: {
                "value": r["value"],
                "package": r["package"],
                "side": r["side"],
            }
            for r in rows
        }
        self._pos_source = pos_path

        # Always use simple raw + origin (no auto-detect transform).
        # Passing Edge_Cuts/F_Paste triggers a different coordinate path
        # that produces wrong values and overwrites the .pos on save.
        placements = load_placements_from_pos(
            pos_path,
            origin_xy=(self._config.board.origin.x, self._config.board.origin.y),
            package_map=self._config.board.package_map,
            skip_refs_prefixes=self._config.board.skip_refs_prefixes,
        )

        self._config.placements.clear()
        self._config.placements.extend(placements)
        self._populate_placements()
        self._append_console(
            f"Loaded {len(self._config.placements)} placements from {pos_path.name}", "cmd"
        )

    # ── Feeders ───────────────────────────────────────────────

    def _populate_feeders(self):
        prev_sel = self._feed_tree.selection()
        for item in self._feed_tree.get_children():
            self._feed_tree.delete(item)
        for slot, f in self._config.feeders.items():
            remain = max(0, f.max_count - f.feed_count) if f.max_count > 0 else "--"
            self._feed_tree.insert("", tk.END, iid=slot, values=(
                slot, f.part_id, f"({f.ref_hole.x:.1f},{f.ref_hole.y:.1f},{f.ref_hole.z:.0f})",
                f"{f.pitch:.0f}", f.feed_count, remain, "Yes" if f.enabled else "No"),
                tags=("en" if f.enabled else "dis",))
        self._feed_tree.tag_configure("en", foreground=FG)
        self._feed_tree.tag_configure("dis", foreground=FG_DIM)
        # Restore selection
        for s in prev_sel:
            if self._feed_tree.exists(s):
                self._feed_tree.selection_set(s)

    def _update_feeder_table(self, status_list):
        for item in self._feed_tree.get_children():
            self._feed_tree.delete(item)
        for s in status_list:
            self._feed_tree.insert("", tk.END, values=(
                s["slot"], s["part_id"], s["next_pick"], "", s["feed_count"], s["remaining"],
                "Yes" if s["enabled"] else "No"),
                tags=("en" if s["enabled"] else "dis",))

    def _edit_selected_feeder(self):
        sel = self._feed_tree.selection()
        if not sel:
            return
        slot = str(sel[0])
        f = self._config.feeders.get(slot)
        if not f:
            return

        dlg = tk.Toplevel(self._root)
        dlg.title(f"Edit Feeder {slot}")
        dlg.geometry("320x340")
        dlg.transient(self._root)
        dlg.grab_set()

        fields = [
            ("Part ID", "part_id", f.part_id),
            ("Ref Hole X", "rh_x", f.ref_hole.x),
            ("Ref Hole Y", "rh_y", f.ref_hole.y),
            ("Ref Hole Z", "rh_z", f.ref_hole.z),
            ("Last Hole X", "lh_x", f.last_hole.x),
            ("Last Hole Y", "lh_y", f.last_hole.y),
            ("Last Hole Z", "lh_z", f.last_hole.z),
            ("Tape Width", "tape_w", f.tape_width),
            ("Rotation", "rot", f.rotation),
            ("Max Count", "max_count", f.max_count),
            ("Enabled", "enabled", f.enabled),
        ]
        vars_ = {}
        for i, (label, key, val) in enumerate(fields):
            ttk.Label(dlg, text=f"{label}:").grid(row=i, column=0, sticky="e", padx=(8, 4), pady=1)
            if isinstance(val, bool):
                var = tk.BooleanVar(value=val)
                ttk.Checkbutton(dlg, variable=var).grid(row=i, column=1, sticky="w", padx=(0, 8), pady=1)
            else:
                var = tk.StringVar(value=str(val))
                ttk.Entry(dlg, textvariable=var, width=16).grid(row=i, column=1, sticky="w", padx=(0, 8), pady=1)
            vars_[key] = var

        def _save():
            try:
                f.part_id = vars_["part_id"].get()
                f.ref_hole.x = float(vars_["rh_x"].get())
                f.ref_hole.y = float(vars_["rh_y"].get())
                f.ref_hole.z = float(vars_["rh_z"].get())
                f.last_hole.x = float(vars_["lh_x"].get())
                f.last_hole.y = float(vars_["lh_y"].get())
                f.last_hole.z = float(vars_["lh_z"].get())
                f.tape_width = float(vars_["tape_w"].get())
                f.rotation = float(vars_["rot"].get())
                f.max_count = int(vars_["max_count"].get())
                f.enabled = vars_["enabled"].get()
            except ValueError as e:
                messagebox.showerror("Error", str(e), parent=dlg)
                return
            self._populate_feeders()
            self._auto_save()
            dlg.destroy()

        ttk.Button(dlg, text="Save", style="Accent.TButton", command=_save).grid(
            row=len(fields), column=0, columnspan=2, pady=8)

    def _goto_selected_feeder(self):
        sel = self._feed_tree.selection()
        if not sel:
            return
        slot = str(sel[0])
        f = self._config.feeders.get(slot)
        if not f:
            return
        self._send_cmd(("__GOTO__", f.ref_hole.x, f.ref_hole.y, None))

    def _reset_feed_count(self):
        sel = self._feed_tree.selection()
        if not sel:
            # No feeder selected — reset ALL feeders
            for feeder in self._config.feeders.values():
                feeder.feed_count = 0
            if self._feeders:
                for feeder in self._config.feeders.values():
                    self._feeders.reset(feeder)
            self._populate_feeders()
            self._auto_save()
            self._append_console("Reset feed count for all feeders", "cmd")
            return
        slot = str(sel[0])
        feeder = self._config.feeders.get(slot)
        if feeder:
            feeder.feed_count = 0
            if self._feeders:
                self._feeders.reset(feeder)
            self._append_console(f"Reset feed count for {slot}", "cmd")
        else:
            self._append_console(f"Feeder '{slot}' not found in config", "err")
        self._populate_feeders()
        self._auto_save()

    # ── Save Config ───────────────────────────────────────────

    def _save_config(self):
        """Apply GUI edits to config object and save to JSON."""
        c = self._config
        try:
            c.machine.travel_feedrate = float(self._cfg_vars["travel_feedrate"].get())
            c.z_heights.safe = float(self._cfg_vars["safe_z"].get())
            c.z_heights.board_surface = float(self._cfg_vars["board_z"].get())
            c.z_heights.feeder_pick = float(self._cfg_vars["feeder_z"].get())
            c.z_heights.discard = float(self._cfg_vars["discard_z"].get())
            c.machine.axis_limits.x.max = float(self._cfg_vars["x_max"].get())
            c.machine.axis_limits.y.max = float(self._cfg_vars["y_max"].get())
            c.board.origin.x = float(self._cfg_vars["origin_x"].get())
            c.board.origin.y = float(self._cfg_vars["origin_y"].get())
            c.board.place_z = float(self._cfg_vars["place_z"].get())
            c.machine.disable_software_endstops = self._cfg_vars["disable_endstops"].get()
            c.vision.max_passes = int(self._cfg_vars["max_passes"].get())
            c.vision.max_linear_offset_mm = float(self._cfg_vars["max_linear"].get())
            c.vision.max_angular_offset_deg = float(self._cfg_vars["max_angular"].get())
            c.vision.converge_mm = float(self._cfg_vars["converge_mm"].get())
            c.camera.device_index = int(self._cfg_vars["cam_dev"].get())
            c.camera.settle_time_ms = int(self._cfg_vars["settle_ms"].get())
            c.camera.position.x = float(self._cfg_vars["cam_x"].get())
            c.camera.position.y = float(self._cfg_vars["cam_y"].get())
        except ValueError as e:
            messagebox.showerror("Invalid Value", str(e))
            return

        save_config(c, self._config_path)
        self._append_console(f"Config saved to {self._config_path}", "cmd")

    # ── Camera ────────────────────────────────────────────────

    def _toggle_camera(self):
        if self._camera_cap is not None:
            self._camera_cap.release()
            self._camera_cap = None
            self._cam_toggle_btn.config(text="Start Camera")
            self._camera_label.config(image="", text="Camera off")
            return
        if not HAS_CAMERA:
            return
        idx = self._cam_idx_var.get()
        self._camera_cap = cv2.VideoCapture(idx)
        if not self._camera_cap.isOpened():
            self._camera_cap = None
            messagebox.showerror("Camera", f"Cannot open device {idx}")
            return
        self._cam_toggle_btn.config(text="Stop Camera")
        self._update_camera()

    def _update_camera(self):
        if self._camera_cap is None:
            return
        # Flush stale buffered frames (OpenCV buffers 5+ frames on macOS)
        for _ in range(4):
            self._camera_cap.grab()
        ret, frame = self._camera_cap.read()
        if ret and frame is not None:
            # Optional: overlay detection bbox from vision pipeline
            if self._show_bbox_var.get():
                frame = self._overlay_detection(frame)

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w = rgb.shape[:2]
            # Fit to actual panel size (updated on window resize)
            panel_w = max(100, self._camera_label.winfo_width() - 8)
            panel_h = max(100, self._camera_label.winfo_height() - 8)
            scale = min(panel_w / w, panel_h / h)
            if scale != 1.0:
                rgb = cv2.resize(rgb, (int(w * scale), int(h * scale)))
            img = ImageTk.PhotoImage(image=Image.fromarray(rgb))
            self._camera_label.config(image=img, text="")
            self._camera_label._imgtk = img
        # Schedule next frame only if still capturing
        if self._camera_cap is not None:
            self._root.after(100, self._update_camera)

    def _overlay_detection(self, frame):
        """Run vision pipeline on the frame and draw bbox + dimensions."""
        import numpy as np
        from nanopnp.vision import VisionSystem
        try:
            vs = VisionSystem(self._config)
            vs._cap = self._camera_cap  # share existing camera
            # Monkey-patch capture to return this exact frame
            vs.capture = lambda settle=False: frame.copy()
            part_id = self._test_ic_var.get() if self._show_bbox_var.get() else ""
            result = vs.detect_part(part_id=part_id)
            vs._cap = None
            return result.frame if result.frame is not None else frame
        except Exception:
            return frame

    def _on_tune_vision(self):
        """Live vision tuning dialog with sliders and intermediate stage previews.

        Mirrors the current pipeline in vision.py:
        Original → Grayscale → Blur → Threshold → Morph Opening →
        Contours (filtered by area) → Pattern match → MinAreaRect → BBox
        """
        if self._camera_cap is None:
            messagebox.showwarning("Camera", "Start the camera first.")
            return

        import numpy as np

        dlg = tk.Toplevel(self._root)
        dlg.title("Vision Pipeline Tuner")
        dlg.geometry("1200x720")
        dlg.transient(self._root)

        left = ttk.Frame(dlg)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=8, pady=8)
        right = ttk.Frame(dlg)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=8, pady=8)

        # Sliders matching the new grayscale pipeline
        v = self._config.vision
        slider_vars = {}
        sliders = [
            ("Blur kernel",   "blur_kernel", 1, 31, v.blur_kernel),
            ("Threshold",     "threshold",   0, 255, v.threshold),
            ("Morph kernel",  "morph_k",     1, 15, 3),
            ("Min area (px)", "min_area",    0, 3000, 500),
            ("Max area (px)", "max_area",    100, 20000, 6000),
            ("Expected pads", "expected",    0, 20, 8),
        ]
        for label, key, lo, hi, init in sliders:
            row = ttk.Frame(left)
            row.pack(fill=tk.X, pady=2)
            ttk.Label(row, text=label, width=14, foreground=FG_DIM).pack(side=tk.LEFT)
            var = tk.IntVar(value=int(init))
            slider_vars[key] = var
            ttk.Scale(row, from_=lo, to=hi, variable=var,
                      orient=tk.HORIZONTAL, length=200).pack(side=tk.LEFT)
            ttk.Label(row, textvariable=var, width=5).pack(side=tk.LEFT, padx=4)

        # Part selector
        part_row = ttk.Frame(left)
        part_row.pack(fill=tk.X, pady=8)
        ttk.Label(part_row, text="Test IC:", foreground=FG_DIM).pack(side=tk.LEFT)
        part_var = tk.StringVar(value=list(self._config.parts.keys())[0])
        ttk.Combobox(part_row, textvariable=part_var,
                     values=list(self._config.parts.keys()),
                     width=12, state="readonly").pack(side=tk.LEFT, padx=4)

        # Info label
        info_lbl = ttk.Label(left, text="", foreground=ACCENT,
                             font=self._mono, wraplength=300, justify=tk.LEFT)
        info_lbl.pack(fill=tk.X, pady=6)

        def _apply_to_config():
            self._config.vision.blur_kernel = slider_vars["blur_kernel"].get()
            self._config.vision.threshold = slider_vars["threshold"].get()
            self._auto_save()
            self._append_console("Vision params applied to config", "cmd")

        ttk.Button(left, text="Apply to Config", style="Accent.TButton",
                   command=_apply_to_config).pack(pady=4)
        ttk.Button(left, text="Close", command=lambda: _on_close()).pack(pady=2)

        # Right side: 2x3 grid of stage previews
        preview_labels = {}
        stages = [
            ("1. Original",           0, 0),
            ("2. Grayscale",          0, 1),
            ("3. Blurred",            0, 2),
            ("4. Threshold (binary)", 1, 0),
            ("5. Morph Opening",      1, 1),
            ("6. Contours + BBox",    1, 2),
        ]
        for name, r, c in stages:
            f = ttk.LabelFrame(right, text=name)
            f.grid(row=r, column=c, sticky="nsew", padx=2, pady=2)
            lbl = tk.Label(f, bg="black")
            lbl.pack(expand=True, fill=tk.BOTH)
            preview_labels[name] = lbl
        for r in range(2):
            right.rowconfigure(r, weight=1)
        for c in range(3):
            right.columnconfigure(c, weight=1)

        def _convert(cv_img, max_w=360, max_h=220):
            if cv_img is None:
                return None
            if len(cv_img.shape) == 2:
                cv_img = cv2.cvtColor(cv_img, cv2.COLOR_GRAY2BGR)
            rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            h, w = rgb.shape[:2]
            scale = min(max_w / w, max_h / h, 1.0)
            if scale < 1.0:
                rgb = cv2.resize(rgb, (int(w * scale), int(h * scale)))
            return ImageTk.PhotoImage(image=Image.fromarray(rgb))

        def _match_pattern_local(centroids, expected):
            """Same logic as VisionSystem._match_pad_pattern."""
            if len(centroids) <= expected:
                return [c[2] for c in centroids]
            pts = np.array([(c[0], c[1]) for c in centroids])
            n = len(pts)
            dx_m = pts[:, 0:1] - pts[:, 0:1].T
            dy_m = pts[:, 1:2] - pts[:, 1:2].T
            dist_mat = np.sqrt(dx_m * dx_m + dy_m * dy_m)
            density_score = np.zeros(n)
            for i in range(n):
                sd = np.sort(dist_mat[i])
                k = min(expected - 1, n - 1)
                density_score[i] = np.sum(sd[1:1 + k])
            keep_idx = np.argsort(density_score)[:expected]
            return [centroids[i][2] for i in sorted(keep_idx)]

        dlg._running = True

        def _update():
            if not dlg._running or self._camera_cap is None:
                return
            for _ in range(2):
                self._camera_cap.grab()
            ret, frame = self._camera_cap.read()
            if not ret or frame is None:
                dlg.after(200, _update)
                return

            h, w = frame.shape[:2]
            img_center = (w // 2, h // 2)

            # 1. Grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # 2. Blur
            k = slider_vars["blur_kernel"].get()
            if k % 2 == 0:
                k += 1
            if k < 1:
                k = 1
            blurred = cv2.GaussianBlur(gray, (k, k), 0)

            # 3. Threshold
            thr = slider_vars["threshold"].get()
            _, binary = cv2.threshold(blurred, thr, 255, cv2.THRESH_BINARY)

            # 4. Morphological opening
            mk = max(1, slider_vars["morph_k"].get())
            kernel = np.ones((mk, mk), np.uint8)
            cleaned = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

            # 5. Find contours + filter by area
            contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            min_a = slider_vars["min_area"].get()
            max_a = slider_vars["max_area"].get()
            filtered = [c for c in contours if min_a <= cv2.contourArea(c) <= max_a]

            # Build result canvas
            result = frame.copy()
            cv2.circle(result, img_center, 6, (255, 0, 0), 2)

            # Draw all filtered contours in yellow
            cv2.drawContours(result, filtered, -1, (0, 255, 255), 1)

            # 6. Pattern match to expected count
            expected = slider_vars["expected"].get()
            if expected > 0 and len(filtered) > expected:
                centroids = []
                for c in filtered:
                    M = cv2.moments(c)
                    if M["m00"] > 0:
                        cx = M["m10"] / M["m00"]
                        cy = M["m01"] / M["m00"]
                        centroids.append((cx, cy, c))
                if len(centroids) >= expected:
                    matched = _match_pattern_local(centroids, expected)
                else:
                    matched = filtered
            else:
                matched = filtered

            # Highlight matched contours in green
            cv2.drawContours(result, matched, -1, (0, 255, 0), 2)

            # Fit minAreaRect if we have enough
            if len(matched) >= 4:
                all_points = np.vstack(matched)
                rect = cv2.minAreaRect(all_points)
                rc, rs, ra = rect
                box = cv2.boxPoints(rect)
                box = np.int32(box)
                cv2.drawContours(result, [box], 0, (0, 200, 255), 3)
                cv2.circle(result, (int(rc[0]), int(rc[1])), 6, (0, 0, 255), -1)

                upp_x = self._config.camera.units_per_pixel.x
                upp_y = self._config.camera.units_per_pixel.y
                rw_mm = rs[0] * upp_x
                rh_mm = rs[1] * upp_y

                # Machine-frame offsets
                img_dx = rc[0] - w / 2
                img_dy = rc[1] - h / 2
                machine_dx = img_dy * upp_x
                machine_dy = -img_dx * upp_y

                # Angle relative to vertical (pins L/R = 0°)
                rw_px, rh_px = rs
                if rw_px < rh_px:
                    long_axis = ra + 90
                else:
                    long_axis = ra
                rel = long_axis - 90
                while rel > 45: rel -= 90
                while rel < -45: rel += 90
                machine_drot = rel

                info_lbl.config(
                    text=f"Contours: {len(filtered)} → matched {len(matched)}\n"
                         f"Size: {rs[0]:.0f}×{rs[1]:.0f} px ({rw_mm:.2f}×{rh_mm:.2f}mm)\n"
                         f"Machine dX: {machine_dx:+.3f} mm\n"
                         f"Machine dY: {machine_dy:+.3f} mm\n"
                         f"Machine dC: {machine_drot:+.1f} °"
                )
            else:
                info_lbl.config(text=f"Contours: {len(filtered)} (need ≥4)")

            # Update all 6 previews
            stages_imgs = {
                "1. Original":           frame,
                "2. Grayscale":          gray,
                "3. Blurred":            blurred,
                "4. Threshold (binary)": binary,
                "5. Morph Opening":      cleaned,
                "6. Contours + BBox":    result,
            }
            for name, img in stages_imgs.items():
                photo = _convert(img)
                if photo:
                    preview_labels[name].config(image=photo)
                    preview_labels[name]._imgtk = photo

            dlg.after(150, _update)

        def _on_close():
            dlg._running = False
            dlg.destroy()

        dlg.protocol("WM_DELETE_WINDOW", _on_close)
        _update()

    def _on_test_vision(self):
        """Run full vision pipeline on current camera frame and show results."""
        if self._camera_cap is None:
            messagebox.showwarning("Camera", "Start the camera first.")
            return

        # Flush and grab fresh frame
        for _ in range(4):
            self._camera_cap.grab()
        ret, frame = self._camera_cap.read()
        if not ret or frame is None:
            messagebox.showerror("Camera", "Failed to capture frame")
            return

        from nanopnp.vision import VisionSystem
        part_id = self._test_ic_var.get()
        part = self._config.parts.get(part_id)
        body_w = part.body_width if part else 0
        body_h = part.body_length if part else 0

        vs = VisionSystem(self._config)
        vs._cap = self._camera_cap
        vs.capture = lambda settle=False: frame.copy()
        try:
            result = vs.detect_part(body_w, body_h, part_id=part_id)
        except Exception as e:
            messagebox.showerror("Vision Error", str(e))
            return
        finally:
            vs._cap = None

        # Cache the detection result so "Apply Correction" can use it
        self._last_detection = result

        # Show result in status label and update camera view with annotated frame
        self._detection_label.config(
            text=f"[{part_id}] dx={result.dx_mm:+.3f}mm  dy={result.dy_mm:+.3f}mm  "
                 f"drot={result.drot_deg:+.1f}°  conf={result.confidence:.2f}"
        )

        if result.frame is not None:
            annotated = result.frame
            rgb = cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB)
            h, w = rgb.shape[:2]
            scale = min(600 / w, 400 / h, 1.0)
            if scale < 1.0:
                rgb = cv2.resize(rgb, (int(w * scale), int(h * scale)))
            img = ImageTk.PhotoImage(image=Image.fromarray(rgb))
            self._camera_label.config(image=img, text="")
            self._camera_label._imgtk = img

        self._append_console(
            f"Test Vision [{part_id}]: dx={result.dx_mm:.3f} dy={result.dy_mm:.3f} "
            f"drot={result.drot_deg:.1f} conf={result.confidence:.2f}", "cmd")

    def _on_apply_correction(self):
        """Two-step correction: yaw first, then XY after re-detection.

        Step 1: Rotate by -drot (yaw only, no XY movement)
        Step 2: Wait for the machine to settle
        Step 3: Re-run vision to get fresh dx/dy (which changes after rotation)
        Step 4: Apply the new dx/dy as a relative XY move
        """
        result = getattr(self, "_last_detection", None)
        if result is None:
            messagebox.showwarning("No Detection",
                "Run 'Test Vision' first to get a detection result.")
            return
        if result.confidence == 0:
            messagebox.showwarning("No Detection",
                "Last detection failed (no contours). Run Test Vision again.")
            return
        if not self._running:
            messagebox.showwarning("Not Connected", "Connect to the machine first.")
            return

        dx = result.dx_mm
        dy = result.dy_mm
        drot = result.drot_deg

        # Confirm
        msg = (f"Apply correction in two steps?\n\n"
               f"Step 1 — Rotate:\n"
               f"  dE = {-drot:+.1f} °\n\n"
               f"Step 2 — (after re-detection):\n"
               f"  dX ≈ {-dx:+.3f} mm\n"
               f"  dY ≈ {-dy:+.3f} mm\n\n"
               f"(Step 2 values will be re-measured after rotation since\n"
               f"rotating the nozzle changes the apparent XY offset.)")
        if not messagebox.askyesno("Apply Correction", msg):
            return

        # Step 1: yaw only
        self._send_cmd(("__CORRECTION__", 0.0, 0.0, -drot))
        self._append_console(f"Correction step 1 (yaw): dE={-drot:+.1f}°", "cmd")

        # Schedule step 2 after the yaw completes. 800ms gives the move
        # plus mechanical settle time before re-detecting.
        self._root.after(800, self._correction_step2)

    def _correction_step2(self):
        """Re-run vision and apply XY correction."""
        if self._camera_cap is None or not self._running:
            return
        # Flush + grab fresh frame
        for _ in range(4):
            self._camera_cap.grab()
        ret, frame = self._camera_cap.read()
        if not ret or frame is None:
            self._append_console("Correction step 2: failed to capture", "err")
            return

        from nanopnp.vision import VisionSystem
        part_id = self._test_ic_var.get()
        part = self._config.parts.get(part_id)
        body_w = part.body_width if part else 0
        body_h = part.body_length if part else 0

        vs = VisionSystem(self._config)
        vs._cap = self._camera_cap
        vs.capture = lambda settle=False: frame.copy()
        try:
            result = vs.detect_part(body_w, body_h, part_id=part_id)
        except Exception as e:
            self._append_console(f"Correction step 2 vision error: {e}", "err")
            return
        finally:
            vs._cap = None

        if result.confidence == 0:
            self._append_console("Correction step 2: no detection, skipping XY", "err")
            return

        self._last_detection = result
        self._detection_label.config(
            text=f"[{part_id}] dx={result.dx_mm:+.3f}mm  dy={result.dy_mm:+.3f}mm  "
                 f"drot={result.drot_deg:+.1f}°  conf={result.confidence:.2f}"
        )

        # Update camera view with annotated frame
        if result.frame is not None:
            rgb = cv2.cvtColor(result.frame, cv2.COLOR_BGR2RGB)
            h, w = rgb.shape[:2]
            panel_w = max(100, self._camera_label.winfo_width() - 8)
            panel_h = max(100, self._camera_label.winfo_height() - 8)
            scale = min(panel_w / w, panel_h / h)
            if scale != 1.0:
                rgb = cv2.resize(rgb, (int(w * scale), int(h * scale)))
            img = ImageTk.PhotoImage(image=Image.fromarray(rgb))
            self._camera_label.config(image=img, text="")
            self._camera_label._imgtk = img

        # Apply XY correction (rotation is already done)
        self._send_cmd(("__CORRECTION__", -result.dx_mm, -result.dy_mm, 0.0))
        self._append_console(
            f"Correction step 2 (XY): dX={-result.dx_mm:+.3f} dY={-result.dy_mm:+.3f}", "cmd")

    def _capture_frame(self):
        if self._camera_cap is None:
            return
        ret, frame = self._camera_cap.read()
        if ret:
            ts = time.strftime("%Y%m%d_%H%M%S")
            path = f"capture_{ts}.png"
            cv2.imwrite(path, frame)
            self._append_console(f"Saved: {path}", "cmd")

    def _on_calibrate_upp(self):
        """Interactive UPP calibration — user draws a bbox on a captured frame.

        Steps:
        1. Capture a frame from the camera
        2. Show it in a dialog window
        3. User click-drags a bbox around the known-size object
        4. User enters the known dimensions
        5. Compute UPP = known_mm / measured_px, offer to apply + save
        """
        if self._camera_cap is None:
            messagebox.showwarning("Camera", "Start the camera first.")
            return

        # Capture a fresh frame
        for _ in range(4):
            self._camera_cap.grab()
        ret, frame = self._camera_cap.read()
        if not ret or frame is None:
            messagebox.showerror("Camera", "Failed to capture frame")
            return

        # Scale frame for display
        h, w = frame.shape[:2]
        max_w, max_h = 640, 480
        scale = min(max_w / w, max_h / h, 1.0)
        disp_w, disp_h = int(w * scale), int(h * scale)
        display_frame = cv2.resize(frame, (disp_w, disp_h)) if scale < 1.0 else frame.copy()

        dlg = tk.Toplevel(self._root)
        dlg.title("Calibrate UPP — Draw bbox around known object")
        dlg.transient(self._root)
        dlg.grab_set()

        # Canvas for image + drawing
        canvas = tk.Canvas(dlg, width=disp_w, height=disp_h, bg="black", cursor="crosshair")
        canvas.pack(padx=8, pady=8)

        rgb = cv2.cvtColor(display_frame, cv2.COLOR_BGR2RGB)
        photo = ImageTk.PhotoImage(image=Image.fromarray(rgb))
        canvas.create_image(0, 0, anchor=tk.NW, image=photo)
        canvas._photo_ref = photo  # prevent GC

        # Bbox drawing state
        bbox_state = {"x0": 0, "y0": 0, "rect": None, "text": None}

        def on_press(e):
            bbox_state["x0"] = e.x
            bbox_state["y0"] = e.y
            if bbox_state["rect"]:
                canvas.delete(bbox_state["rect"])
            if bbox_state["text"]:
                canvas.delete(bbox_state["text"])
            bbox_state["rect"] = canvas.create_rectangle(
                e.x, e.y, e.x, e.y, outline="#22c55e", width=2)

        def on_drag(e):
            if bbox_state["rect"]:
                canvas.coords(bbox_state["rect"], bbox_state["x0"], bbox_state["y0"], e.x, e.y)
                w_px = abs(e.x - bbox_state["x0"]) / scale
                h_px = abs(e.y - bbox_state["y0"]) / scale
                if bbox_state["text"]:
                    canvas.delete(bbox_state["text"])
                bbox_state["text"] = canvas.create_text(
                    min(e.x, bbox_state["x0"]) + 5, min(e.y, bbox_state["y0"]) - 10,
                    text=f"{w_px:.0f} x {h_px:.0f} px", anchor=tk.W,
                    fill="#22c55e", font=("Courier", 10, "bold"))

        canvas.bind("<Button-1>", on_press)
        canvas.bind("<B1-Motion>", on_drag)
        canvas.bind("<ButtonRelease-1>", on_drag)

        # Known dimensions input
        form = ttk.Frame(dlg)
        form.pack(padx=8, pady=4)
        ttk.Label(form, text="Known Width (mm):").grid(row=0, column=0, sticky="e", padx=4, pady=2)
        w_var = tk.StringVar(value="3.9")
        ttk.Entry(form, textvariable=w_var, width=10).grid(row=0, column=1, pady=2)
        ttk.Label(form, text="Known Height (mm):").grid(row=0, column=2, sticky="e", padx=(10, 4), pady=2)
        h_var = tk.StringVar(value="4.9")
        ttk.Entry(form, textvariable=h_var, width=10).grid(row=0, column=3, pady=2)

        ttk.Label(dlg, text="Click and drag to draw a bbox around the object",
                  foreground=FG_DIM).pack(pady=2)

        result_label = ttk.Label(dlg, text="", foreground=ACCENT, font=self._mono)
        result_label.pack(pady=4)

        def _apply():
            if not bbox_state["rect"]:
                messagebox.showerror("Error", "Draw a bbox first", parent=dlg)
                return
            try:
                kw = float(w_var.get())
                kh = float(h_var.get())
            except ValueError:
                messagebox.showerror("Error", "Invalid dimensions", parent=dlg)
                return

            x0, y0, x1, y1 = canvas.coords(bbox_state["rect"])
            # Convert canvas coords back to original frame coords
            w_px = abs(x1 - x0) / scale
            h_px = abs(y1 - y0) / scale

            if w_px < 10 or h_px < 10:
                messagebox.showerror("Error", "Bbox too small — draw a larger box", parent=dlg)
                return

            # Match longer known dimension to longer measured dimension
            known_long = max(kw, kh)
            known_short = min(kw, kh)
            measured_long = max(w_px, h_px)
            measured_short = min(w_px, h_px)

            new_upp_x = known_long / measured_long
            new_upp_y = known_short / measured_short

            old = (self._config.camera.units_per_pixel.x,
                   self._config.camera.units_per_pixel.y)

            result_label.config(
                text=f"Measured: {w_px:.0f} x {h_px:.0f} px\n"
                     f"Old UPP: ({old[0]:.6f}, {old[1]:.6f})\n"
                     f"New UPP: ({new_upp_x:.6f}, {new_upp_y:.6f})"
            )

            if messagebox.askyesno("Apply?",
                    f"Apply new UPP?\n\n"
                    f"X: {old[0]:.6f} → {new_upp_x:.6f}\n"
                    f"Y: {old[1]:.6f} → {new_upp_y:.6f}",
                    parent=dlg):
                self._config.camera.units_per_pixel.x = new_upp_x
                self._config.camera.units_per_pixel.y = new_upp_y
                self._upp_label.config(
                    text=f"UPP: X={new_upp_x:.8f}  Y={new_upp_y:.8f} mm/px")
                self._auto_save()
                self._append_console(
                    f"UPP calibrated: ({new_upp_x:.6f}, {new_upp_y:.6f})", "cmd")
                dlg.destroy()

        btns = ttk.Frame(dlg)
        btns.pack(pady=8)
        ttk.Button(btns, text="Apply", style="Accent.TButton", command=_apply).pack(side=tk.LEFT, padx=4)
        ttk.Button(btns, text="Cancel", command=dlg.destroy).pack(side=tk.LEFT, padx=4)

    # ── Lifecycle ─────────────────────────────────────────────

    def run(self):
        import signal as _signal
        _signal.signal(_signal.SIGTERM, lambda sig, frame: self._root.after(0, self._on_close))
        self._root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._root.mainloop()

    def _on_close(self):
        self._polling = False
        self._running = False
        if self._camera_cap:
            self._camera_cap.release()
        if self._serial:
            self._serial.close()
        self._root.destroy()

    def _on_reload(self):
        """Re-exec the Python process to pick up code changes.

        Sets an env var with the current connection state so the new process
        can optionally auto-reconnect to the same port at the same baud.
        The Marlin board doesn't reset on reconnect (it resets only when the
        FTDI chip re-enumerates), so the machine state is preserved.
        """
        import os, sys
        # Save connection state so the next process can resume
        if self._running and self._serial and not self._serial.dry_run:
            os.environ["NANOPNP_AUTO_CONNECT"] = "1"
            os.environ["NANOPNP_PORT"] = self._serial._port
            os.environ["NANOPNP_BAUD"] = str(self._serial._baud)

        self._polling = False
        self._running = False
        if self._camera_cap:
            try:
                self._camera_cap.release()
            except Exception:
                pass
        # Close serial cleanly — the new process will reopen it
        if self._serial:
            try:
                self._serial.close()
            except Exception:
                pass
        try:
            self._root.destroy()
        except Exception:
            pass
        os.execv(sys.executable, [sys.executable] + sys.argv)


def launch_gui(config: NanoPnPConfig, config_path: str = "config.json") -> None:
    NanoPnPGUI(config, config_path).run()
