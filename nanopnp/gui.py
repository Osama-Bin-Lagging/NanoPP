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
        self._running = False
        self._polling = False
        self._save_timer: str | None = None
        self._cmd_history: list[str] = []
        self._history_idx: int = -1
        self._camera_cap = None
        self._pos = {"X": 0.0, "Y": 0.0, "Z": 0.0, "E": 0.0}

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

        self._build_layout()
        self._bind_keys()
        self._poll_results()

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
        ttk.Button(qp, text="Set Origin", width=14, command=lambda: self._send_cmd("__SET_ORIGIN__")).pack(pady=1, padx=4)

        return sb

    # ── Tabs ──────────────────────────────────────────────────

    def _build_tabs(self, parent) -> ttk.Notebook:
        self._nb = ttk.Notebook(parent)
        self._nb.add(self._build_job_tab(), text="  Job  ")
        self._nb.add(self._build_setup_tab(), text="  Setup  ")
        self._nb.add(self._build_camera_tab(), text="  Camera  ")
        self._nb.add(self._build_console_tab(), text="  Console  ")
        return self._nb

    # ── Tab: Job ──────────────────────────────────────────────

    def _build_job_tab(self) -> ttk.Frame:
        tab = ttk.Frame(self._nb)
        tab.rowconfigure(0, weight=2)
        tab.rowconfigure(1, weight=0, minsize=40)
        tab.rowconfigure(2, weight=1)
        tab.columnconfigure(0, weight=1)

        # Placements table
        pl_f = ttk.LabelFrame(tab, text="Placements")
        pl_f.grid(row=0, column=0, sticky="nsew", padx=4, pady=(4, 2))
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
        ttk.Button(pb, text="Load Gerber...", style="Accent.TButton", command=self._load_gerber_dir).pack(side=tk.LEFT, padx=2)
        ttk.Button(pb, text="Edit", command=self._edit_selected_placement).pack(side=tk.LEFT, padx=2)
        ttk.Button(pb, text="Go To", command=self._goto_selected_placement).pack(side=tk.LEFT, padx=2)
        ttk.Button(pb, text="Toggle Enable", command=self._toggle_placement_enable).pack(side=tk.LEFT, padx=2)
        ttk.Button(pb, text="Enable All", command=lambda: self._set_all_placements(True)).pack(side=tk.LEFT, padx=2)
        ttk.Button(pb, text="Disable All", command=lambda: self._set_all_placements(False)).pack(side=tk.LEFT, padx=2)

        # Run controls
        ctrl = ttk.LabelFrame(tab, text="Run")
        ctrl.grid(row=1, column=0, sticky="ew", padx=4, pady=6)
        cr = ttk.Frame(ctrl)
        cr.pack(fill=tk.X, padx=4, pady=6)
        ttk.Button(cr, text="Solder Only", style="Accent.TButton", command=self._on_run_paste).pack(side=tk.LEFT, padx=3)
        ttk.Button(cr, text="Pick & Place Only", style="Green.TButton", command=self._on_run_place).pack(side=tk.LEFT, padx=3)
        ttk.Button(cr, text="Full Job (Solder + PnP)", style="Accent.TButton", command=self._on_run_full_job).pack(side=tk.LEFT, padx=3)
        ttk.Separator(cr, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=6)
        ttk.Button(cr, text="STOP", style="Red.TButton", command=self._on_stop_job).pack(side=tk.LEFT, padx=3)
        ttk.Checkbutton(cr, text="Vision", variable=self._vision_enabled_var).pack(side=tk.LEFT, padx=6)
        self._progress_var = tk.DoubleVar(value=0)
        ttk.Progressbar(cr, variable=self._progress_var, maximum=100, length=200).pack(side=tk.LEFT, padx=6)
        self._progress_label = ttk.Label(cr, text="Idle", foreground=FG_DIM)
        self._progress_label.pack(side=tk.LEFT, padx=4)

        # Results table
        res_f = ttk.LabelFrame(tab, text="Results")
        res_f.grid(row=2, column=0, sticky="nsew", padx=4, pady=(2, 4))
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

        # UPP display
        upp = self._config.camera.units_per_pixel
        self._upp_label = ttk.Label(tab, text=f"UPP: X={upp.x:.8f}  Y={upp.y:.8f} mm/px", foreground=FG_DIM)
        self._upp_label.grid(row=2, column=0, sticky="w", padx=8, pady=2)

        return tab

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

        return tab

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
                self._result_queue.put(("connected", True))
            except Exception as e:
                self._result_queue.put(("connected", str(e)))

        threading.Thread(target=_do_connect, daemon=True).start()

    def _on_disconnect(self):
        self._polling = False
        self._running = False
        if self._serial:
            self._serial.close()
        self._serial = self._motion = self._feeders = self._paste = self._engine = None
        self._status_dot.itemconfig("dot", fill=RED)
        self._connect_btn.config(state=tk.NORMAL)
        self._disconnect_btn.config(state=tk.DISABLED)

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
                elif item == "__RUN_PASTE__":
                    self._polling = False
                    self._result_queue.put(("job_status", "Dispensing paste..."))
                    placements = self._config.enabled_placements()
                    count = self._paste.dispense_board(placements)
                    self._result_queue.put(("job_status", f"Paste done: {count}/{len(placements)}"))
                    self._result_queue.put(("pos", self._motion.get_position()))
                    self._polling = True
                    self._poll_position()
                elif item in ("__RUN_PLACE__", "__RUN_JOB__"):
                    self._polling = False  # pause position polling during job
                    paste_first = item == "__RUN_JOB__"
                    self._engine.vision_enabled = self._vision_enabled_var.get()
                    placements = self._config.enabled_placements()
                    self._result_queue.put(("job_status", "Running..."))

                    def _prog(i, total, ref, status):
                        pct = (i / total * 100) if total > 0 else 0
                        self._result_queue.put(("job_progress", (pct, f"{ref}: {status}")))

                    result = self._engine.run_job(
                        placements, paste_dispenser=self._paste if paste_first else None,
                        paste_first=paste_first, progress_cb=_prog)
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
                self._result_queue.put(("console_err", str(e)))

    def _on_serial_command(self, cmd, response):
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
                elif t == "feeders":
                    self._update_feeder_table(d)
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
        if self._running:
            self._send_cmd("__RUN_PASTE__")

    def _on_run_place(self):
        if self._running:
            self._send_cmd("__RUN_PLACE__")

    def _on_run_full_job(self):
        if self._running:
            self._send_cmd("__RUN_JOB__")

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
        save_config(self._config, self._config_path)

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

    def _load_gerber_dir(self):
        d = filedialog.askdirectory(title="Select Gerber Directory", parent=self._root)
        if not d:
            return
        try:
            from nanopnp.board_parser import load_board
            origin = self._config.board.origin
            components, info = load_board(d, origin.x, origin.y)
        except Exception as e:
            messagebox.showerror("Error", str(e))
            return
        self._config.placements.clear()
        from nanopnp.config import Placement
        for c in components:
            self._config.placements.append(Placement(ref=c.reference, part_id=c.package,
                                                     x=c.x, y=c.y, rotation=c.rotation, enabled=True))
        self._populate_placements()
        self._auto_save()
        self._append_console(f"Loaded {len(components)} components from {d}", "cmd")

    # ── Feeders ───────────────────────────────────────────────

    def _populate_feeders(self):
        for item in self._feed_tree.get_children():
            self._feed_tree.delete(item)
        for slot, f in self._config.feeders.items():
            remain = max(0, f.max_count - f.feed_count) if f.max_count > 0 else "--"
            self._feed_tree.insert("", tk.END, values=(
                slot, f.part_id, f"({f.ref_hole.x:.1f},{f.ref_hole.y:.1f},{f.ref_hole.z:.0f})",
                f"{f.pitch:.0f}", f.feed_count, remain, "Yes" if f.enabled else "No"),
                tags=("en" if f.enabled else "dis",))
        self._feed_tree.tag_configure("en", foreground=FG)
        self._feed_tree.tag_configure("dis", foreground=FG_DIM)

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
        slot = self._feed_tree.item(sel[0], "values")[0]
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
            ("Pitch", "pitch", f.pitch),
            ("Tape Width", "tape_w", f.tape_width),
            ("Rotation", "rot", f.rotation),
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
                f.pitch = float(vars_["pitch"].get())
                f.tape_width = float(vars_["tape_w"].get())
                f.rotation = float(vars_["rot"].get())
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
        vals = self._feed_tree.item(sel[0], "values")
        pos = vals[2].strip("()").split(",")
        self._send_cmd(("__GOTO__", float(pos[0]), float(pos[1]), None))

    def _reset_feed_count(self):
        sel = self._feed_tree.selection()
        if not sel:
            return
        slot = self._feed_tree.item(sel[0], "values")[0]
        feeder = self._config.feeders.get(slot)
        if feeder:
            feeder.feed_count = 0
            if self._feeders:
                self._feeders.reset(feeder)
        self._populate_feeders()
        self._auto_save()
        self._append_console(f"Reset feed count for {slot}", "cmd")

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
        ret, frame = self._camera_cap.read()
        if ret and frame is not None:
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w = rgb.shape[:2]
            scale = min(600 / w, 400 / h, 1.0)
            if scale < 1.0:
                rgb = cv2.resize(rgb, (int(w * scale), int(h * scale)))
            img = ImageTk.PhotoImage(image=Image.fromarray(rgb))
            self._camera_label.config(image=img, text="")
            self._camera_label._imgtk = img
        self._root.after(200, self._update_camera)

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
        """Calibrate units-per-pixel using a known-size object on the camera.

        Steps:
        1. User places a known component (e.g. SOIC8 body 3.9x4.9mm) on the camera
        2. Dialog asks for the known dimensions
        3. Pipeline detects the object and measures pixel size
        4. Computes new UPP and offers to apply + save
        """
        if self._camera_cap is None:
            messagebox.showwarning("Camera", "Start the camera first.")
            return

        # Ask for known dimensions
        dlg = tk.Toplevel(self._root)
        dlg.title("Calibrate Units-Per-Pixel")
        dlg.geometry("340x220")
        dlg.transient(self._root)
        dlg.grab_set()

        ttk.Label(dlg, text="Place a known-size object on the camera,\n"
                  "then enter its dimensions below.", foreground=FG_DIM).pack(padx=12, pady=8)

        f = ttk.Frame(dlg)
        f.pack(padx=12, pady=4)
        ttk.Label(f, text="Width (mm):").grid(row=0, column=0, sticky="e", padx=4, pady=2)
        w_var = tk.StringVar(value="3.9")
        ttk.Entry(f, textvariable=w_var, width=10).grid(row=0, column=1, pady=2)
        ttk.Label(f, text="Height (mm):").grid(row=1, column=0, sticky="e", padx=4, pady=2)
        h_var = tk.StringVar(value="4.9")
        ttk.Entry(f, textvariable=h_var, width=10).grid(row=1, column=1, pady=2)

        ttk.Label(dlg, text="Default: SOIC8 body (3.9 x 4.9 mm)", foreground=FG_DIM).pack(pady=2)

        result_label = ttk.Label(dlg, text="", foreground=ACCENT)
        result_label.pack(pady=4)

        def _run():
            try:
                kw = float(w_var.get())
                kh = float(h_var.get())
            except ValueError:
                messagebox.showerror("Error", "Enter valid numbers", parent=dlg)
                return

            if not HAS_CAMERA:
                return

            # Build a temporary VisionSystem using the live camera
            from nanopnp.vision import VisionSystem, VisionError
            vs = VisionSystem(self._config)
            # Share the existing camera capture
            vs._cap = self._camera_cap

            try:
                cal = vs.calibrate_upp(kw, kh)
            except VisionError as e:
                messagebox.showerror("Calibration Failed", str(e), parent=dlg)
                return
            finally:
                vs._cap = None  # don't let it close our camera

            old = cal["old_upp"]
            new = cal["new_upp"]
            px = cal["measured_px"]
            result_label.config(
                text=f"Measured: {px[0]:.0f} x {px[1]:.0f} px\n"
                     f"Old UPP: ({old[0]:.6f}, {old[1]:.6f})\n"
                     f"New UPP: ({new[0]:.6f}, {new[1]:.6f})")

            # Save debug image
            if cal.get("frame") is not None:
                cv2.imwrite("calibration_debug.png", cal["frame"])

            if messagebox.askyesno("Apply?",
                    f"Apply new UPP?\n\nX: {old[0]:.6f} → {new[0]:.6f}\nY: {old[1]:.6f} → {new[1]:.6f}",
                    parent=dlg):
                self._config.camera.units_per_pixel.x = new[0]
                self._config.camera.units_per_pixel.y = new[1]
                self._upp_label.config(text=f"UPP: X={new[0]:.8f}  Y={new[1]:.8f} mm/px")
                self._auto_save()
                self._append_console(f"UPP calibrated: ({new[0]:.6f}, {new[1]:.6f})", "cmd")
                dlg.destroy()

        ttk.Button(dlg, text="Detect & Calibrate", style="Accent.TButton", command=_run).pack(pady=8)

    # ── Lifecycle ─────────────────────────────────────────────

    def run(self):
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


def launch_gui(config: NanoPnPConfig, config_path: str = "config.json") -> None:
    NanoPnPGUI(config, config_path).run()
