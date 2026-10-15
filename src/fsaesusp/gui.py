"""Simple desktop GUI: ``fsae-susp-gui`` (or ``python -m fsaesusp.gui``).

Runs the command-line tool in a subprocess, streams its output and previews the plots it writes.
"""
from __future__ import annotations

import csv
import os
import re
import queue
import subprocess
import sys
import threading
import time
import tkinter as tk
import tkinter.font as tkfont
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

import yaml

from . import vsusp
from .paths import design_name, designs_dir, project_root

LOCK_LABELS = dict(lower_x="Lower pivot x", lower_y="Lower pivot y", upper_x="Upper pivot x", upper_y="Upper pivot y",
                   upper_bj_x="Upper ball joint x", lower_bj_x="Lower ball joint x", camber="Static camber",
                   tie_rod_in="Outer tie rod, inboard", tie_rod_down="Outer tie rod, below hub",
                   tie_rod_x="Outer tie rod, fore/aft", rack_x="Rack line, fore/aft")

try:
    from PIL import Image, ImageTk          # installed with matplotlib
except ImportError:                         # pragma: no cover
    Image = ImageTk = None

# Output folders by command, as in the README: <project>/<folder>/<design name>/
OUT_DIRS = {"analyze": "results", "check3d": "results3d", "report": "report", "sensitivity": "sensitivity"}
OUT_LABELS = {"analyze": "analysis", "check3d": "3D", "report": "report", "sensitivity": "tolerance"}

MUTED, STRIPE, DIFF = "#6b6a66", "#f6f6f4", "#fdf0d5"
LOG_COLOURS = {"cmd": "#1f5fbf", "ok": "#2e7d32", "warn": "#a35a00", "err": "#b42318", "dim": MUTED}


def out_dir(design, cmd: str) -> Path:
    design = Path(design)
    return project_root(design) / OUT_DIRS[cmd] / design.stem


def compare_dir(designs) -> Path:
    """results/compare/<a>_vs_<b>[...]"""
    stems = [Path(d).stem for d in designs]
    return project_root(Path(designs[0])) / "results" / "compare" / "_vs_".join(stems)


def project_name(path: Path) -> str:
    try:
        return str((yaml.safe_load(path.read_text(encoding="utf-8")) or {}).get("name", ""))
    except Exception:
        return "(unreadable)"


def log_tag(line: str):
    """Colour for a line of command output."""
    if line.startswith("$ "):
        return "cmd"
    if line.startswith(("error", "Traceback")) or "Error:" in line:
        return "err"
    if line.startswith("  - "):                 # an entry under "Flags:"
        return "warn"
    if line.startswith(("Wrote ", "Updated ", "Flags: none")):
        return "ok"
    return None


def open_path(p: Path, select=False):
    """Open a file or folder with the system's default app; ``select`` shows a file in its folder instead."""
    if sys.platform == "win32":
        if select:
            subprocess.Popen(["explorer", "/select,", str(p)])
        else:
            os.startfile(p)
    else:
        subprocess.Popen(["open" if sys.platform == "darwin" else "xdg-open", str(p.parent if select else p)])


def elide(text: str, font, width: int) -> str:
    """``text`` cut short with an ellipsis to fit ``width`` pixels."""
    if font.measure(text) <= width:
        return text
    while text and font.measure(text + "...") > width:
        text = text[:-1]
    return text.rstrip() + "..."


def wrap_to_width(label):
    """Rewrap a label's text whenever its width changes."""
    label.bind("<Configure>", lambda e: label.configure(wraplength=max(e.width - 4, 50)), add="+")


def dpi_aware():
    """Crisp text on scaled Windows displays; without this Windows stretches the window as a bitmap."""
    if sys.platform == "win32":
        try:
            import ctypes
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except Exception:
            pass


class Tooltip:
    def __init__(self, widget, text, delay=600):
        self.w, self.text, self.delay, self.tip, self.job = widget, text, delay, None, None
        widget.bind("<Enter>", self._schedule, add="+")
        widget.bind("<Leave>", self._hide, add="+")
        widget.bind("<ButtonPress>", self._hide, add="+")

    def _schedule(self, _e):
        self._hide(); self.job = self.w.after(self.delay, self._show)

    def _show(self):
        self.job = None
        x, y = self.w.winfo_rootx() + 8, self.w.winfo_rooty() + self.w.winfo_height() + 4
        self.tip = tk.Toplevel(self.w); self.tip.wm_overrideredirect(True); self.tip.wm_geometry(f"+{x}+{y}")
        tk.Label(self.tip, text=self.text, justify="left", background="#ffffe8", foreground="#202020",
                 relief="solid", borderwidth=1, padx=6, pady=3).pack()

    def _hide(self, _e=None):
        if self.job:
            self.w.after_cancel(self.job); self.job = None
        if self.tip:
            self.tip.destroy(); self.tip = None


class App(tk.Tk):
    def __init__(self):
        dpi_aware()
        super().__init__()
        self.title("fsae-susp")
        self.s = s = self.winfo_fpixels("1i") / 96           # display scale: 1.25 at 125 %
        sw, sh = self.winfo_screenwidth(), self.winfo_screenheight()
        w, h = min(int(1280 * s), int(sw * .92)), min(int(820 * s), int(sh * .85))
        self.geometry(f"{w}x{h}+{(sw - w) // 2}+{max(0, (sh - h) // 2 - int(20 * s))}")
        self.minsize(int(1060 * s), int(580 * s))

        self.proc = None
        self.lines = queue.Queue()
        self._imgs, self._pils = [], {}
        self._png_paths = []
        self._after_run = None
        self._resize_job = None
        self._t0, self._cmd, self._stopping = 0.0, "", False
        self._locks = {}            # (design, axle) -> (the file's locks, what was ticked) from the Optimise dialog

        self.design = tk.StringVar()
        self.outdir = tk.StringVar()
        self.axle = tk.StringVar(value="both")
        self.quick = tk.BooleanVar(value=True)
        self.no_steering = tk.BooleanVar(value=False)
        self.status = tk.StringVar()
        self.elapsed = tk.StringVar()
        self.png_count = tk.StringVar()
        self.plot_view = tk.StringVar(value="all")
        self.design.trace_add("write", lambda *_: self.title(
            f"fsae-susp - {Path(self.design.get()).stem}" if self.design.get().strip() else "fsae-susp"))
        self._style()
        self._build()
        self._keys()
        self.status.set("Double-click a design in Projects, or pick a design file, to start."
                        if self.projects.get_children() else "Pick a design file or import a VSusp link to start.")
        self.after(100, self._drain)
        self.protocol("WM_DELETE_WINDOW", self._close)

    # ----------------------------------------------------------- layout
    def _px(self, n):
        return int(n * self.s)

    def _style(self):
        st = ttk.Style(self)
        base = tkfont.nametofont("TkDefaultFont"); size = base.cget("size")
        self.bold = base.copy(); self.bold.configure(weight="bold")
        self.heading = base.copy(); self.heading.configure(weight="bold", size=size + (1 if size > 0 else -1))
        self.mono = tkfont.Font(family="Consolas", size=10) if sys.platform == "win32" else \
            tkfont.nametofont("TkFixedFont").copy()
        self.linespace = base.metrics("linespace")
        st.configure("Treeview", rowheight=int(self.linespace * 1.55))
        st.configure("Treeview.Heading", font=self.bold)
        st.configure("TNotebook.Tab", padding=(self._px(12), self._px(4)))
        st.configure("TLabelframe.Label", font=self.bold)
        st.configure("Muted.TLabel", foreground=MUTED)
        st.configure("Heading.TLabel", font=self.heading)

    def _build(self):
        px, gap = self._px(10), self._px(6)
        top = ttk.Frame(self, padding=(px, px, px, 0)); top.pack(fill="x")

        box = ttk.LabelFrame(top, text="Design", padding=(px, gap // 2, px, px)); box.pack(fill="x")
        box.columnconfigure(1, weight=1)
        ttk.Label(box, text="File").grid(row=0, column=0, sticky="w", padx=(0, gap))
        ttk.Entry(box, textvariable=self.design).grid(row=0, column=1, sticky="ew", pady=2)
        b = ttk.Button(box, text="Browse...", command=self._pick_design); b.grid(row=0, column=2, sticky="ew", padx=(gap, 0))
        Tooltip(b, "Pick a design file (Ctrl+O)")
        b = ttk.Button(box, text="Import VSusp link...", command=self._import_link)
        b.grid(row=0, column=3, sticky="ew", padx=(gap, 0))
        Tooltip(b, "Make a new design in designs/ from a VSusp link")
        ttk.Label(box, text="Output").grid(row=1, column=0, sticky="w", padx=(0, gap))
        ttk.Entry(box, textvariable=self.outdir, state="readonly").grid(row=1, column=1, sticky="ew", pady=2)
        b = ttk.Button(box, text="Open folder", command=self._open_outdir); b.grid(row=1, column=2, sticky="ew", padx=(gap, 0))
        Tooltip(b, "Open the folder the last command wrote to")

        run = ttk.LabelFrame(top, text="Run", padding=(px, gap // 2, px, px)); run.pack(fill="x", pady=(gap, 0))
        self.run_buttons = []

        def button(text, fn, tip):
            b = ttk.Button(run, text=text, command=fn); b.pack(side="left", padx=(0, gap))
            Tooltip(b, tip); self.run_buttons.append(b)

        def sep():
            ttk.Separator(run, orient="vertical").pack(side="left", fill="y", padx=(gap, gap * 2))

        button("Analyze", self._analyze, "Front-view table, flags and plot.\nWrites results/<design>/")
        button("Full report", self._report, "Front view and 3D check, all plots and report.md.\nWrites report/<design>/")
        button("VSusp link", self._link, "Print the design's VSusp link")
        sep()
        button("3D check", self._check3d, "3D tables, flags and plots.\nWrites results3d/<design>/")
        c = ttk.Checkbutton(run, text="Skip steering", variable=self.no_steering); c.pack(side="left")
        Tooltip(c, "Leave out the slower steering sweep in the 3D check")
        sep()
        button("Springs...", self._springs, "Analyse the rod, rocker and damper (motion ratio, rates, ride frequency,\n"
               "roll and load transfer), or design new ones from spring_design\nand save them as a new design")
        button("Sensitivity...", self._sensitivity, "How far each hardpoint's build tolerance moves the kinematics.\n"
               "Writes sensitivity/<design>/")
        sep()
        button("Optimise...", self._optimize, "Pick what to lock, then optimise the inner pivots (and the ball\n"
               "joints, if the design file gives them bounds), place the front steering for any\n"
               "Ackermann or steering-ratio target, and save the result as a new design")
        ttk.Label(run, text="Axle").pack(side="left", padx=(0, gap // 2))
        ttk.Combobox(run, textvariable=self.axle, values=["both", "front", "rear"], width=6,
                     state="readonly").pack(side="left", padx=(0, gap * 2))
        c = ttk.Checkbutton(run, text="Quick", variable=self.quick); c.pack(side="left")
        Tooltip(c, "Smaller search, about 4x faster")
        self.stop_btn = ttk.Button(run, text="Stop", command=self._stop, state="disabled")
        self.stop_btn.pack(side="right")
        Tooltip(self.stop_btn, "Stop the running command (Esc)")

        bar = ttk.Frame(self, padding=(px, gap // 2)); bar.pack(fill="x", side="bottom")
        ttk.Separator(self).pack(fill="x", side="bottom")
        ttk.Label(bar, textvariable=self.status).pack(side="left")
        self.progress = ttk.Progressbar(bar, mode="indeterminate", length=self._px(120))
        self.elapsed_lbl = ttk.Label(bar, textvariable=self.elapsed, style="Muted.TLabel")

        panes = ttk.PanedWindow(self, orient="horizontal"); panes.pack(fill="both", expand=True, padx=px, pady=gap)
        panes.add(self._build_projects(panes), weight=1)
        nb = ttk.Notebook(panes); panes.add(nb, weight=3)
        self.nb = nb
        self._build_output(nb)
        self._build_plots(nb)
        self._build_compare(nb)

        self._refresh_projects()
        want = sum(self.projects.column(c, "width") for c in ("#0", *self.projects["columns"])) + self._px(24)
        self.after(150, lambda: panes.sashpos(0, min(want, int(self.winfo_width() * .42))))

    def _build_projects(self, parent):
        gap = self._px(6)
        f = ttk.Frame(parent, padding=(0, 0, gap, 0))
        head = ttk.Frame(f); head.pack(fill="x", pady=(0, gap))
        ttk.Label(head, text="Projects", style="Heading.TLabel").pack(side="left")
        ttk.Label(head, text="in designs/", style="Muted.TLabel").pack(side="left", padx=gap)
        cols = ("project", "modified", "outputs")
        tv = ttk.Treeview(f, columns=cols, selectmode="extended")
        tv.heading("#0", text="Design", anchor="w")
        for c in cols:
            tv.heading(c, text=c.capitalize(), anchor="w")
        tv.tag_configure("odd", background=STRIPE)
        ys = ttk.Scrollbar(f, orient="vertical", command=tv.yview); tv.configure(yscrollcommand=ys.set)

        bar = ttk.Frame(f); bar.pack(side="bottom", fill="x", pady=(gap, 0))
        hint = ttk.Label(f, text="Double-click to open. Ctrl+click 2 to 4 designs to compare. Right-click for more.",
                         style="Muted.TLabel")
        hint.pack(side="bottom", fill="x", pady=(gap // 2, 0)); wrap_to_width(hint)
        ys.pack(side="right", fill="y"); tv.pack(fill="both", expand=True)
        tv.bind("<Double-1>", lambda e: tv.identify_row(e.y) and self._open_project())
        tv.bind("<Return>", lambda e: self._open_project())
        tv.bind("<Button-3>", self._project_menu)
        self.projects = tv
        b = ttk.Button(bar, text="Open", command=self._open_project); b.pack(side="left")
        Tooltip(b, "Open the selected design and show its latest plots")
        b = ttk.Button(bar, text="Compare", command=self._compare); b.pack(side="left", padx=gap)
        Tooltip(b, "Front-view metrics of the selected designs side by side.\nWrites results/compare/<a>_vs_<b>/")
        self.run_buttons.append(b)
        b = ttk.Button(bar, text="Refresh", command=self._refresh_projects); b.pack(side="left")
        Tooltip(b, "Reload the designs folder (F5)")
        return f

    def _build_output(self, nb):
        f = ttk.Frame(nb); nb.add(f, text="Output")
        bar = ttk.Frame(f, padding=self._px(4)); bar.grid(row=0, column=0, columnspan=2, sticky="ew")
        ttk.Button(bar, text="Clear", command=self._clear_log).pack(side="right")
        ttk.Button(bar, text="Copy all", command=self._copy_log).pack(side="right", padx=self._px(6))
        ttk.Label(bar, text="Output of the commands you run", style="Muted.TLabel").pack(side="left")
        self.log = tk.Text(f, wrap="none", font=self.mono, state="disabled", borderwidth=0, highlightthickness=0,
                           padx=self._px(8), pady=self._px(6), background="white", foreground="#202020")
        for tag, colour in LOG_COLOURS.items():
            self.log.tag_configure(tag, foreground=colour)
        bold = tkfont.Font(font=self.mono); bold.configure(weight="bold"); self._mono_bold = bold
        self.log.tag_configure("cmd", font=bold, spacing1=self._px(8), spacing3=self._px(4))
        ys = ttk.Scrollbar(f, orient="vertical", command=self.log.yview)
        xs = ttk.Scrollbar(f, orient="horizontal", command=self.log.xview)
        self.log.configure(yscrollcommand=ys.set, xscrollcommand=xs.set)
        self.log.grid(row=1, column=0, sticky="nsew"); ys.grid(row=1, column=1, sticky="ns"); xs.grid(row=2, column=0, sticky="ew")
        f.rowconfigure(1, weight=1); f.columnconfigure(0, weight=1)
        self.log.bind("<Button-1>", lambda e: self.log.focus_set())     # so Ctrl+C copies the selection
        self._write("Command output appears here.\n", "dim"); self._log_hint = True

    def _build_plots(self, nb):
        gap = self._px(6)
        f = ttk.Frame(nb); nb.add(f, text="Plots")
        bar = ttk.Frame(f, padding=self._px(4)); bar.pack(fill="x")
        for text, view, tip in (("All", "all", "Show every plot at once; click one to enlarge it"),
                                ("One", "one", "Show one plot, large")):
            b = ttk.Radiobutton(bar, text=text, value=view, variable=self.plot_view, style="Toolbutton",
                                command=self._show_png, width=5)
            b.pack(side="left"); Tooltip(b, tip)
        ttk.Separator(bar, orient="vertical").pack(side="left", fill="y", padx=gap)
        b = ttk.Button(bar, text="<", width=3, command=lambda: self._step_png(-1)); b.pack(side="left")
        Tooltip(b, "Previous plot (Left arrow)")
        b = ttk.Button(bar, text=">", width=3, command=lambda: self._step_png(1)); b.pack(side="left", padx=(gap // 2, gap))
        Tooltip(b, "Next plot (Right arrow)")
        # the right-hand items are packed before the plot list, so on a narrow window the list shrinks first
        ttk.Button(bar, text="Refresh", command=self._load_pngs).pack(side="right")
        b = ttk.Button(bar, text="Open image", command=self._open_png); b.pack(side="right", padx=gap)
        Tooltip(b, "Open the plot in your image viewer (or double-click it)")
        ttk.Label(bar, textvariable=self.png_count, style="Muted.TLabel").pack(side="right", padx=gap)
        self.png_box = ttk.Combobox(bar, state="readonly", width=12)
        self.png_box.pack(side="left", fill="x", expand=True)
        self.png_box.bind("<<ComboboxSelected>>", lambda e: self._show_png(view="one"))
        self.canvas = tk.Canvas(f, background="white", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda e: self._schedule_render())
        self.canvas.bind("<Double-1>", lambda e: self.plot_view.get() == "one" and self._open_png())

    def _build_compare(self, nb):
        f = ttk.Frame(nb); nb.add(f, text="Compare")
        self.cmp_note = ttk.Label(f, text="Select 2 to 4 designs in Projects (Ctrl+click) and click Compare.",
                                  style="Muted.TLabel", padding=self._px(6))
        self.cmp_note.pack(fill="x"); wrap_to_width(self.cmp_note)
        self.cmp = ttk.Treeview(f, show="headings")
        self.cmp.tag_configure("odd", background=STRIPE)
        self.cmp.tag_configure("diff", background=DIFF)
        ys = ttk.Scrollbar(f, orient="vertical", command=self.cmp.yview); self.cmp.configure(yscrollcommand=ys.set)
        ys.pack(side="right", fill="y"); self.cmp.pack(fill="both", expand=True)

    def _keys(self):
        self.bind("<Control-o>", lambda e: self._pick_design())
        self.bind("<F5>", lambda e: (self._refresh_projects(), self._load_pngs()))
        self.bind("<Escape>", lambda e: self._stop())
        for key, step in (("<Left>", -1), ("<Right>", 1)):
            self.bind(key, lambda e, d=step: self._step_png(d) if self._plots_have_keys(e) else None)

    def _plots_have_keys(self, e):
        return self.nb.index("current") == 1 and e.widget.winfo_class() not in ("Entry", "TEntry", "Text", "TCombobox")

    def _center(self, win):
        win.update_idletasks()
        x = self.winfo_rootx() + (self.winfo_width() - win.winfo_reqwidth()) // 2
        y = self.winfo_rooty() + (self.winfo_height() - win.winfo_reqheight()) // 3
        win.geometry(f"+{max(0, x)}+{max(0, y)}")

    # ----------------------------------------------------------- projects
    def _refresh_projects(self):
        tv = self.projects; keep = set(tv.selection())
        tv.delete(*tv.get_children())
        d = designs_dir()
        files = sorted(d.glob("*.y*ml"), key=lambda q: q.stat().st_mtime, reverse=True) if d.is_dir() else []
        font = tkfont.nametofont("TkDefaultFont"); pad = self._px(18)
        widths = [font.measure("Design") + pad, font.measure("Project" * 2) + pad,
                  font.measure("2026-10-10 10:00") + pad, font.measure("Outputs") + pad]
        for i, q in enumerate(files):
            outs = ", ".join(OUT_LABELS[c] for c in OUT_DIRS if out_dir(q, c).is_dir()) or "-"
            when = time.strftime("%Y-%m-%d %H:%M", time.localtime(q.stat().st_mtime))
            name = elide(project_name(q), font, self._px(220))
            tv.insert("", "end", iid=str(q), text=q.stem, values=(name, when, outs), tags=("odd",) if i % 2 else ())
            widths[0] = max(widths[0], font.measure(q.stem) + pad + self._px(14))      # + the tree indent
            widths[1] = max(widths[1], font.measure(name) + pad)
            widths[3] = max(widths[3], font.measure(outs) + pad)
        for c, w in zip(("#0", "project", "modified", "outputs"), widths):
            tv.column(c, width=w, minwidth=self._px(40), stretch=c == "project")
        tv.selection_set([i for i in keep if tv.exists(i)])

    def _open_project(self):
        sel = self.projects.selection()
        if not sel:
            return
        d = Path(sel[0]); self.design.set(str(d))
        found = [p for p in (out_dir(d, c) for c in OUT_DIRS) if p.is_dir()]
        if found:
            self.outdir.set(str(max(found, key=lambda p: p.stat().st_mtime)))     # the newest results
            self._load_pngs(); self.nb.select(1)
            self.status.set(f"Opened {d.stem}. Showing its plots from {', '.join(q.parent.name for q in found)}.")
        else:
            self.outdir.set(""); self._load_pngs()
            self.status.set(f"Opened {d.stem}. No results yet: click Analyze or Full report.")

    def _project_menu(self, e):
        tv = self.projects; row = tv.identify_row(e.y)
        if not row:
            return
        if row not in tv.selection():
            tv.selection_set(row)
        tv.focus(row)
        sel = tv.selection(); busy = "disabled" if self.proc else "normal"
        m = tk.Menu(self, tearoff=False)
        m.add_command(label="Open", command=self._open_project, font=self.bold)
        m.add_separator()
        for label, fn in (("Analyze", self._analyze), ("3D check", self._check3d), ("Springs...", self._springs),
                          ("Sensitivity...", self._sensitivity), ("Full report", self._report)):
            m.add_command(label=label, state=busy, command=lambda fn=fn: (self._open_project(), fn()))
        m.add_command(label=f"Compare {len(sel)} designs", command=self._compare,
                      state=busy if 2 <= len(sel) <= 4 else "disabled")
        m.add_separator()
        m.add_command(label="Show in folder", command=lambda: open_path(Path(row), select=True))
        m.tk_popup(e.x_root, e.y_root)

    def _compare(self):
        sel = sorted(self.projects.selection(), key=lambda q: Path(q).stat().st_mtime)   # oldest = baseline
        if not 2 <= len(sel) <= 4:
            messagebox.showinfo("fsae-susp", "Select 2 to 4 designs (Ctrl+click) to compare.", parent=self); return
        out = compare_dir(sel)
        self.outdir.set(str(out))
        self._after_run = lambda: (self._load_compare(out / "comparison.csv"), self.nb.select(2))
        self._run(["compare", *sel, "-o", str(out)])

    def _load_compare(self, path):
        try:
            with open(path, newline="", encoding="utf-8") as f:
                header, *rows = list(csv.reader(f))
        except (OSError, ValueError) as e:
            self.cmp_note.configure(text=f"Could not read {path}: {e}"); return
        tv = self.cmp; font = tkfont.nametofont("TkDefaultFont"); pad = self._px(20)
        tv.delete(*tv.get_children())
        tv.configure(columns=list(range(len(header))))
        for i, h in enumerate(header):
            w = max(font.measure(x[i]) for x in [header, *rows] if i < len(x)) + pad
            tv.heading(i, text=h, anchor="w")
            tv.column(i, width=w, minwidth=self._px(60), stretch=i == 0, anchor="w")
        n = len(header) - 1 - (header[-1] == "Change")
        for k, r in enumerate(rows):
            tag = "diff" if len(set(r[1:1 + n])) > 1 else "odd" if k % 2 else ""
            tv.insert("", "end", values=r, tags=(tag,) if tag else ())
        self.cmp_note.configure(text=f"Front-view metrics; rows that differ are shaded. The overlay plot is in the "
                                     f"Plots tab, and CSV and Markdown copies are in {path.parent}.")

    # ----------------------------------------------------------- pickers
    def _pick_design(self):
        d = designs_dir()
        p = filedialog.askopenfilename(title="Design file", initialdir=d if d.is_dir() else None, parent=self,
                                       filetypes=[("Design files", "*.yaml *.yml *.json"), ("All", "*.*")])
        if p:
            self.design.set(p)

    def _open_outdir(self):
        p = Path(self.outdir.get())
        if not self.outdir.get() or not p.is_dir():
            messagebox.showinfo("fsae-susp", "Nothing written yet. Run a command first." if not self.outdir.get()
                                else f"{p} does not exist yet. Run a command first.", parent=self); return
        open_path(p)

    def _import_link(self):
        px, gap = self._px(10), self._px(6)
        win = tk.Toplevel(self); win.title("Import VSusp link"); win.transient(self); win.withdraw()
        f = ttk.Frame(win, padding=px); f.pack(fill="both", expand=True)
        f.columnconfigure(1, weight=1); f.rowconfigure(1, weight=1)
        ttk.Label(f, text="Paste a VSusp link, or the path of a text file holding one:").grid(
            row=0, column=0, columnspan=3, sticky="w")
        txt = tk.Text(f, width=80, height=7, wrap="char", font=self.mono, undo=True, relief="solid", borderwidth=1,
                      padx=4, pady=4)
        txt.grid(row=1, column=0, columnspan=3, sticky="nsew", pady=(gap // 2, gap))
        name = tk.StringVar(); touched = [False]
        ttk.Label(f, text="Save as").grid(row=2, column=0, sticky="w", padx=(0, gap))
        ent = ttk.Entry(f, textvariable=name); ent.grid(row=2, column=1, sticky="ew")
        ent.bind("<Key>", lambda e: touched.__setitem__(0, True))
        target = ttk.Label(f, style="Muted.TLabel"); target.grid(row=3, column=1, columnspan=2, sticky="w", pady=(2, 0))
        btns = ttk.Frame(f); btns.grid(row=4, column=0, columnspan=3, sticky="e", pady=(px, 0))

        def clean():
            return re.sub(r"[^\w.-]+", "_", name.get().strip()).removesuffix(".yaml")

        def link_text():
            link = txt.get("1.0", "end").strip()
            try:
                return Path(link).read_text(encoding="utf-8").strip() if Path(link).is_file() else link
            except OSError:
                return link

        def on_text(_e=None):
            txt.edit_modified(False)
            if not touched[0]:
                try:
                    name.set(design_name(vsusp.parse_link(link_text())["project_name"]))
                except Exception:               # the import itself reports a bad link
                    name.set("design" if txt.get("1.0", "end").strip() else "")

        def on_name(*_):
            n = clean()
            target.configure(text=f"designs/{n}.yaml" + ("  (exists - will ask before overwriting)"
                                                          if n and (designs_dir() / f"{n}.yaml").exists() else "")
                             if n else "")

        def go():
            link = txt.get("1.0", "end").strip(); n = clean()
            if not link or not n:
                (txt if not link else ent).focus_set(); return
            out = designs_dir() / f"{n}.yaml"
            if out.exists() and not messagebox.askyesno("fsae-susp", f"{out} exists. Overwrite it?", parent=win):
                return
            out.parent.mkdir(parents=True, exist_ok=True)
            win.destroy()
            self.design.set(str(out))
            self._after_run = self._refresh_projects
            self._run(["import-vsusp", link, "-o", str(out)])

        name.trace_add("write", on_name)
        txt.bind("<<Modified>>", on_text)
        ttk.Button(btns, text="Cancel", command=win.destroy).pack(side="right")
        ok = ttk.Button(btns, text="Import", command=go, default="active"); ok.pack(side="right", padx=gap)
        if self.proc:
            ok.configure(state="disabled"); target.configure(text="Wait for the running command to finish.")
        win.bind("<Escape>", lambda e: win.destroy())
        txt.bind("<Control-Return>", lambda e: (go(), "break")[1])
        ent.bind("<Return>", lambda e: go())
        try:
            clip = self.clipboard_get().strip()
        except tk.TclError:
            clip = ""
        if "vsusp" in clip.lower() and len(clip) < 20000:   # a link on the clipboard: fill it in
            txt.insert("1.0", clip); txt.tag_add("sel", "1.0", "end")
        self._center(win); win.deiconify(); win.grab_set(); txt.focus_set()

    # ----------------------------------------------------------- commands
    def _need_design(self):
        d = self.design.get().strip()
        if not d or not Path(d).is_file():
            messagebox.showwarning("fsae-susp", "Pick an existing design file first: double-click one in Projects, "
                                   "or click Browse.", parent=self); return None
        return d

    def _analyze(self):
        d = self._need_design()
        if d:
            self._run(["analyze", d, "-o", self._out(d, "analyze")])

    def _optimize(self):
        d = self._need_design()
        if not d:
            return
        self.configure(cursor="watch"); self.update_idletasks()
        try:
            from . import api, config as cfgmod        # numpy and scipy: loaded the first time only
            from .optimize import LOCKABLE, BALL_JOINTS
            from .steeropt import STEERING_VARS, wants_steering
            cfg = cfgmod.load(d)
            axles = api.AXLES if self.axle.get() == "both" else (self.axle.get(),)
            setup = {ax: api.optimise_setup(cfg, ax) for ax in axles}
        except Exception as e:
            messagebox.showerror("fsae-susp", f"Could not set up the optimiser for {Path(d).name}:\n{e}", parent=self)
            return
        finally:
            self.configure(cursor="")
        target = cfg["optimise"]["camber"]["outside_target"]
        px, gap = self._px(10), self._px(6)
        win = tk.Toplevel(self); win.title(f"Optimise {Path(d).stem}"); win.transient(self); win.withdraw()
        f = ttk.Frame(win, padding=px); f.pack(fill="both", expand=True)
        ncol = 1 + 2 * len(axles)
        ticks = {}
        for j, ax in enumerate(axles):
            st = setup[ax]; c = 1 + 2 * j
            ttk.Label(f, text=ax.capitalize(), font=self.bold).grid(row=1, column=c, columnspan=2, sticky="w",
                                                                     padx=(px * 2, 0))
            # what was ticked here last time, unless the design file's locks have changed since
            file_locks, chosen = self._locks.get((d, ax), (st["locked"], set(st["locked"])))
            if file_locks != st["locked"]:
                chosen = set(st["locked"])
            for i, k in enumerate(LOCKABLE, start=2):
                if k in STEERING_VARS and ax != "front":
                    ticks[ax, k] = tk.BooleanVar(value=False)
                    ttk.Label(f, text="front only", style="Muted.TLabel").grid(row=i, column=c, sticky="w",
                                                                              padx=(px * 2, gap))
                    continue
                v = st["values"][k]; fixed = False
                if k == "camber":
                    text, note = f"{v:.2f} deg", f"else chosen for {target:g} deg outside"
                elif k in BALL_JOINTS:          # mm inboard of the hub face; fixed unless the design file gives bounds
                    lo, hi = map(float, st["bounds"][k]); fixed = lo == hi and k not in st["locked"]
                    text = f"{v:.2f} mm"
                    note = ("fixed (give it bounds to free it)" if lo == hi == v else f"fixed at {lo:g} by bounds"
                            if lo == hi else f"{lo:g} to {hi:g}")
                elif k in STEERING_VARS:        # mm; tie_rod_x and rack_x ahead of the axle (behind = negative)
                    lo, hi = map(float, st["bounds"][k]); fixed = lo == hi and k not in st["locked"]
                    text = f"{v:.1f} mm"
                    note = (("fixed (give it bounds to free it)" if lo == hi == v else f"fixed at {lo:g} by bounds")
                            if lo == hi else f"{lo:g} to {hi:g}")
                    if not wants_steering(st["targets"]):
                        note += " (with a steering target)"
                else:
                    lo, hi = map(float, st["bounds"][k]); fixed = lo == hi and k not in st["locked"]
                    text, note = f"{v:.1f} mm", f"fixed at {lo:g} by bounds" if lo == hi else f"{lo:g} to {hi:g}"
                var = tk.BooleanVar(value=k in chosen); ticks[ax, k] = var
                cb = ttk.Checkbutton(f, text=text, variable=var, state="disabled" if fixed else "normal")
                cb.grid(row=i, column=c, sticky="w", padx=(px * 2, gap), pady=1)
                ttk.Label(f, text=note, style="Muted.TLabel").grid(row=i, column=c + 1, sticky="w")
                Tooltip(cb, f"Lock the {ax} {LOCK_LABELS[k].lower()} at {text}")
        for i, k in enumerate(LOCKABLE, start=2):
            ttk.Label(f, text=LOCK_LABELS[k]).grid(row=i, column=0, sticky="w")
        f.update_idletasks()                # wrap the text above and below to the table's width
        wrap = max(f.grid_bbox(0, 1, ncol - 1, len(LOCKABLE) + 1)[2], self._px(380))
        ttk.Label(f, wraplength=wrap, text="Tick what keeps its current value. Everything else is searched "
                  "within its range.").grid(row=0, column=0, columnspan=ncol, sticky="w", pady=(0, gap))
        kpis = [f"{ax} {float(setup[ax]['targets']['kpi']):g} deg" for ax in axles
                if setup[ax]["targets"].get("kpi") is not None]
        ft = setup["front"]["targets"] if "front" in setup else {}
        steer_t = [f"{lab} {ft[k]:g}{unit}" for k, lab, unit in (("ackermann", "Ackermann", "%"),
                   ("rack_per_deg", "rack", " mm/deg"), ("steering_ratio", "steering ratio", ":1"))
                   if ft.get(k) is not None]
        ttk.Label(f, style="Muted.TLabel", wraplength=wrap, text="Locks in the design file (optimise: <axle>: lock:) "
                  "start ticked. The new design keeps the locks used, so it opens with them ticked."
                  + (f" KPI target ({', '.join(kpis)}): the upper ball joint, or the lower one if the upper one is "
                     "fixed, is placed to hold it exactly, and the static camber is searched with the pivots."
                     if kpis else "")
                  + (f" Steering targets (front: {', '.join(steer_t)}): after the pivots, the outer tie rod and "
                     "rack line are placed to hit them, keeping the tie rod's rim clearance."
                     if steer_t else "")).grid(
            row=len(LOCKABLE) + 2, column=0, columnspan=ncol, sticky="w", pady=(gap, 0))
        steer_only = tk.BooleanVar(value=False)
        if steer_t:
            so = ttk.Checkbutton(f, text="Steering only (keep the pivots, ball joints and camber)", variable=steer_only)
            so.grid(row=len(LOCKABLE) + 3, column=0, columnspan=ncol, sticky="w", pady=(gap, 0))
            Tooltip(so, "Only place the front outer tie rod and rack line for the steering targets")
        btns = ttk.Frame(f); btns.grid(row=len(LOCKABLE) + 4, column=0, columnspan=ncol, sticky="e", pady=(px, 0))

        def go():
            lock, unlock = [], []           # passed as changes to the design file's locks
            for ax in axles:
                st = setup[ax]
                chosen = {k for k in LOCKABLE if ticks[ax, k].get()}
                self._locks[d, ax] = (st["locked"], chosen)
                lock += [f"{ax}.{k}" for k in LOCKABLE if k in chosen and k not in st["locked"]]
                unlock += [f"{ax}.{k}" for k in LOCKABLE if k not in chosen and k in st["locked"]]
            win.destroy()
            p = Path(d)
            out = filedialog.asksaveasfilename(title="Save optimised design as", defaultextension=".yaml", parent=self,
                                               initialdir=p.parent, initialfile=p.stem + "_opt.yaml",
                                               filetypes=[("YAML", "*.yaml")])
            if not out:
                return
            args = ["optimize", d, "-o", out, "--axle", self.axle.get(), "--workers", "-1", "--plot"]
            if self.quick.get():
                args.append("--quick")
            if steer_only.get():
                args.append("--steering-only")
            args += (["--lock", ",".join(lock)] if lock else []) + (["--unlock", ",".join(unlock)] if unlock else [])
            self._after_run = lambda: (self.design.set(out), self.outdir.set(str(Path(out).with_suffix("")) + "_plots"))
            self._run(args)

        ttk.Button(btns, text="Cancel", command=win.destroy).pack(side="right")
        ok = ttk.Button(btns, text="Optimise...", command=go, default="active"); ok.pack(side="right", padx=gap)
        if self.proc:
            ok.configure(state="disabled")
        win.bind("<Escape>", lambda e: win.destroy())
        win.bind("<Return>", lambda e: ok.invoke())
        self._center(win); win.deiconify(); win.grab_set(); ok.focus_set()

    def _check3d(self):
        d = self._need_design()
        if d:
            self._run(["check3d", d, "-o", self._out(d, "check3d")] + (["--no-steering"] if self.no_steering.get() else []))

    def _report(self):
        d = self._need_design()
        if d:
            self._run(["report", d, "-o", self._out(d, "report")])

    def _springs(self):
        d = self._need_design()
        if not d:
            return
        try:
            data = yaml.safe_load(Path(d).read_text(encoding="utf-8")) or {}
        except (OSError, yaml.YAMLError):
            data = {}
        has_springs = any((data.get(a) or {}).get("spring") for a in ("front", "rear"))
        has_design = bool(data.get("spring_design"))
        px, gap = self._px(10), self._px(6)
        win = tk.Toplevel(self); win.title(f"Springs {Path(d).stem}"); win.transient(self); win.withdraw()
        f = ttk.Frame(win, padding=px); f.pack(fill="both", expand=True)
        mode = tk.StringVar(value="analyse" if has_springs or not has_design else "design")
        r1 = ttk.Radiobutton(f, text="Analyse the design's springs", value="analyse", variable=mode,
                             state="normal" if has_springs else "disabled")
        r1.grid(row=0, column=0, columnspan=3, sticky="w")
        ttk.Label(f, text="Motion ratio, rates, ride frequency, roll and load transfer. Writes results3d/<design>/"
                  if has_springs else "No front.spring or rear.spring block in this design yet.",
                  style="Muted.TLabel").grid(row=1, column=0, columnspan=3, sticky="w", padx=(self._px(22), 0))
        r2 = ttk.Radiobutton(f, text="Design new springs...", value="design", variable=mode,
                             state="normal" if has_design else "disabled")
        r2.grid(row=2, column=0, columnspan=3, sticky="w", pady=(gap, 0))
        ttk.Label(f, text="Generate and optimise the rod, rocker and damper within spring_design's bounds, pick the "
                  "spring rates\nand size the anti-roll bar, then save it as a new design." if has_design else
                  "Add a spring_design block first (see examples/fsae_aero_v2.yaml).",
                  style="Muted.TLabel").grid(row=3, column=0, columnspan=3, sticky="w", padx=(self._px(22), 0))
        opts = ttk.Frame(f); opts.grid(row=4, column=0, columnspan=3, sticky="w", padx=(self._px(22), 0), pady=(gap, 0))
        axle = tk.StringVar(value=self.axle.get()); quick = tk.BooleanVar(value=self.quick.get())
        ttk.Label(opts, text="Axle").pack(side="left")
        ax_box = ttk.Combobox(opts, textvariable=axle, values=["both", "front", "rear"], width=6, state="readonly")
        ax_box.pack(side="left", padx=(gap // 2, gap * 2))
        q = ttk.Checkbutton(opts, text="Quick (about 10 s per axle, else about 1 min)", variable=quick); q.pack(side="left")

        def sync(*_):
            st = "normal" if mode.get() == "design" else "disabled"
            q.configure(state=st); ax_box.configure(state="readonly" if st == "normal" else "disabled")
        mode.trace_add("write", sync); sync()
        btns = ttk.Frame(f); btns.grid(row=5, column=0, columnspan=3, sticky="e", pady=(px, 0))

        def go():
            win.destroy()
            if mode.get() == "analyse":
                self._run(["springs", d, "-o", self._out(d, "check3d")]); return
            p = Path(d)
            out = filedialog.asksaveasfilename(title="Save the design with new springs as", defaultextension=".yaml",
                                               parent=self, initialdir=p.parent, initialfile=p.stem + "_springs.yaml",
                                               filetypes=[("YAML", "*.yaml")])
            if not out:
                return
            args = ["design-springs", d, "-o", out, "--axle", axle.get(), "--workers", "-1", "--plot"]
            if quick.get():
                args.append("--quick")
            self._after_run = lambda: (self.design.set(out), self.outdir.set(str(Path(out).with_suffix("")) + "_plots"))
            self._run(args)

        ttk.Button(btns, text="Cancel", command=win.destroy).pack(side="right")
        ok = ttk.Button(btns, text="Run", command=go, default="active"); ok.pack(side="right", padx=gap)
        if self.proc or not (has_springs or has_design):
            ok.configure(state="disabled")
        win.bind("<Escape>", lambda e: win.destroy())
        win.bind("<Return>", lambda e: ok.invoke())
        self._center(win); win.deiconify(); win.grab_set(); ok.focus_set()

    def _sensitivity(self):
        d = self._need_design()
        if not d:
            return
        try:
            sc = (yaml.safe_load(Path(d).read_text(encoding="utf-8")) or {}).get("sensitivity") or {}
        except (OSError, yaml.YAMLError):
            sc = {}
        px, gap = self._px(10), self._px(6)
        win = tk.Toplevel(self); win.title(f"Sensitivity {Path(d).stem}"); win.transient(self); win.withdraw()
        f = ttk.Frame(win, padding=px); f.pack(fill="both", expand=True)
        tol = tk.StringVar(value=f"{float(sc.get('tol', 1.0)):g}")
        tol_u = tk.StringVar(value=f"{float(sc.get('tol_upright', sc.get('tol', 1.0))):g}")
        ttk.Label(f, text="Each hardpoint coordinate is moved by its tolerance, one at a time.").grid(
            row=0, column=0, columnspan=3, sticky="w", pady=(0, gap))
        for i, (lab, var, note) in enumerate((("Chassis points", tol, "arm pivots, inner tie rod / toe link"),
                                              ("Upright points", tol_u, "ball joints, outer tie rod / toe link")), 1):
            ttk.Label(f, text=lab).grid(row=i, column=0, sticky="w")
            ttk.Spinbox(f, textvariable=var, from_=0.05, to=10, increment=0.25, width=7).grid(
                row=i, column=1, sticky="w", padx=gap, pady=1)
            ttk.Label(f, text=f"mm  ({note})", style="Muted.TLabel").grid(row=i, column=2, sticky="w")
        axle = tk.StringVar(value=self.axle.get())
        ttk.Label(f, text="Axle").grid(row=3, column=0, sticky="w", pady=(gap, 0))
        ttk.Combobox(f, textvariable=axle, values=["both", "front", "rear"], width=6, state="readonly").grid(
            row=3, column=1, sticky="w", padx=gap, pady=(gap, 0))
        ttk.Label(f, text="About 20 s per axle.", style="Muted.TLabel").grid(row=4, column=0, columnspan=3,
                                                                           sticky="w", pady=(gap, 0))
        btns = ttk.Frame(f); btns.grid(row=5, column=0, columnspan=3, sticky="e", pady=(px, 0))

        def go():
            try:
                t, tu = float(tol.get()), float(tol_u.get())
                if t <= 0 or tu <= 0:
                    raise ValueError
            except ValueError:
                messagebox.showwarning("fsae-susp", "Tolerances are numbers above 0 mm.", parent=win); return
            win.destroy()
            self._run(["sensitivity", d, "-o", self._out(d, "sensitivity"), "--tol", f"{t:g}",
                       "--tol-upright", f"{tu:g}", "--axle", axle.get()])

        ttk.Button(btns, text="Cancel", command=win.destroy).pack(side="right")
        ok = ttk.Button(btns, text="Run", command=go, default="active"); ok.pack(side="right", padx=gap)
        if self.proc:
            ok.configure(state="disabled")
        win.bind("<Escape>", lambda e: win.destroy())
        win.bind("<Return>", lambda e: ok.invoke())
        self._center(win); win.deiconify(); win.grab_set(); ok.focus_set()

    def _out(self, design, cmd):
        p = out_dir(design, cmd)
        self.outdir.set(str(p))
        return str(p)

    def _link(self):
        d = self._need_design()
        if d:
            self._run(["link", d])

    # ----------------------------------------------------------- process
    def _run(self, args):
        if self.proc:
            return
        if self._log_hint:
            self._clear_log(); self._log_hint = False
        self._write(f"$ fsae-susp {' '.join(args)}\n")
        self.nb.select(0)
        env = dict(os.environ, PYTHONUNBUFFERED="1", PYTHONIOENCODING="utf-8")
        flags = subprocess.CREATE_NO_WINDOW if sys.platform == "win32" else 0
        self.proc = subprocess.Popen([sys.executable, "-m", "fsaesusp", *args], stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT, text=True, encoding="utf-8", errors="replace",
                                     env=env, creationflags=flags)
        self._t0, self._cmd, self._stopping = time.monotonic(), args[0], False
        self._set_busy(True)
        self.status.set(f"Running {args[0]}...")
        threading.Thread(target=self._reader, args=(self.proc,), daemon=True).start()

    def _reader(self, proc):
        for line in proc.stdout:
            self.lines.put(line)
        self.lines.put((proc.wait(),))

    def _drain(self):
        try:
            while True:
                item = self.lines.get_nowait()
                if isinstance(item, tuple):
                    self._finished(item[0])
                else:
                    self._write(item)
        except queue.Empty:
            pass
        if self.proc:
            self.elapsed.set(f"{self._cmd}  {time.monotonic() - self._t0:.0f} s")
        self.after(100, self._drain)

    def _finished(self, code):
        self.proc = None
        took = f"{time.monotonic() - self._t0:.1f} s"
        self._set_busy(False)
        after, self._after_run = self._after_run, None
        self._refresh_projects()
        if self._stopping:
            self._write(f"[stopped after {took}]\n", "warn")
            self.status.set("Stopped.")
        elif code == 0:
            self._write(f"[done in {took}]\n", "dim")
            if after:
                after()
            self.status.set(f"{self._cmd} done in {took}.")
            if self._load_pngs() and self.nb.index("current") != 1:
                self.status.set(f"{self._cmd} done in {took}. The plots are in the Plots tab.")
        else:
            self._write(f"[failed with exit code {code} after {took}]\n", "err")
            self.status.set(f"{self._cmd} failed (exit code {code}). See the Output tab.")

    def _stop(self):
        if self.proc:
            self._stopping = True
            self.proc.terminate()

    def _set_busy(self, busy):
        for b in self.run_buttons:
            b.configure(state="disabled" if busy else "normal")
        self.stop_btn.configure(state="normal" if busy else "disabled")
        if busy:
            self.progress.pack(side="right"); self.elapsed_lbl.pack(side="right", padx=self._px(8))
            self.progress.start(15)
        else:
            self.progress.stop(); self.progress.pack_forget(); self.elapsed_lbl.pack_forget(); self.elapsed.set("")

    def _write(self, s, tag=None):
        log = self.log
        at_end = log.yview()[1] >= 0.999        # follow the output unless the user scrolled up
        log.configure(state="normal")
        log.insert("end", s, tag or log_tag(s) or ())
        log.configure(state="disabled")
        if at_end:
            log.see("end")

    def _clear_log(self):
        self.log.configure(state="normal"); self.log.delete("1.0", "end"); self.log.configure(state="disabled")

    def _copy_log(self):
        self.clipboard_clear(); self.clipboard_append(self.log.get("1.0", "end-1c"))
        self.status.set("Output copied to the clipboard.")

    def _close(self):
        if self.proc and not messagebox.askyesno("fsae-susp", f"{self._cmd} is still running. Stop it and quit?",
                                                 parent=self):
            return
        self._stop(); self.destroy()

    # ----------------------------------------------------------- plots
    def _plot_dirs(self):
        """Folders to show plots from. For a design's own results that is every command's folder, so the
        front-view plot from Analyze shows beside the 3D plots from 3D check."""
        out, d = self.outdir.get(), self.design.get().strip()
        own = [out_dir(d, c) for c in OUT_DIRS] if d else []
        if not out or Path(out) in own:
            return [q for q in own if q.is_dir()]
        return [Path(out)] if Path(out).is_dir() else []

    def _load_pngs(self):
        shown = self._current_png()
        newest = {}                                 # one of each plot: the latest run's
        dirs = self._plot_dirs()
        for folder in dirs:
            for q in folder.glob("*.png"):
                if q.name not in newest or q.stat().st_mtime > newest[q.name].stat().st_mtime:
                    newest[q.name] = q
        self._png_paths = [newest[k] for k in sorted(newest)]
        self._pils = {}
        for q in self._png_paths:
            if Image is not None:
                try:
                    with Image.open(q) as im:
                        self._pils[q] = im.copy()   # kept in memory so resizing does not reread the files
                except OSError:
                    pass
        self.png_box.configure(values=[self._png_label(q, len(dirs) > 1) for q in self._png_paths])
        names = [q.name for q in self._png_paths]
        if names:
            self.png_box.current(names.index(shown.name) if shown and shown.name in names else 0)
        else:
            self.png_box.set("")
        self._show_png()
        return bool(names)

    @staticmethod
    def _png_label(q, where):
        return f"{q.name}   ({q.parent.parent.name})" if where else q.name

    def _step_png(self, d):
        if self._png_paths:
            if self.plot_view.get() == "one":
                self.png_box.current((self.png_box.current() + d) % len(self._png_paths))
            self._show_png(view="one")

    def _current_png(self):
        i = self.png_box.current()
        return self._png_paths[i] if 0 <= i < len(self._png_paths) else None

    def _open_png(self):
        p = self._current_png()
        if p and p.is_file():
            open_path(p)

    def _show_png(self, view=None, index=None):
        if view:
            self.plot_view.set(view)
        if index is not None:
            self.png_box.current(index)
        n = len(self._png_paths)
        if not n:
            self.png_count.set("")
        elif self.plot_view.get() == "all":
            self.png_count.set(f"{n} plot{'s' if n > 1 else ''}")
        else:
            self.png_count.set(f"{self.png_box.current() + 1} of {n}")
        self._render()

    def _schedule_render(self):
        if self._resize_job:
            self.after_cancel(self._resize_job)
        self._resize_job = self.after(60, self._render)

    def _fit(self, q, w, h):
        """Photo of plot ``q`` scaled to fit w x h pixels."""
        w, h = max(w, 10), max(h, 10)
        if q in self._pils:
            im = self._pils[q].copy(); im.thumbnail((w, h), Image.LANCZOS)
            return ImageTk.PhotoImage(im)
        try:
            im = tk.PhotoImage(file=str(q))
        except tk.TclError:
            return None
        f = max(1, -(-im.width() // w), -(-im.height() // h))
        return im.subsample(f, f)

    def _grid(self, w, h, cap):
        """Columns for tiling the plots in w x h: the layout that shows the most image area."""
        sizes = [self._pils[q].size if q in self._pils else (4, 3) for q in self._png_paths]; n = len(sizes)

        def area(cols):
            rows = -(-n // cols); cw, ch = w / cols, h / rows - cap
            return sum(min(cw / a, ch / b) ** 2 * a * b for a, b in sizes) if ch > 0 else 0
        return max(range(1, n + 1), key=area)

    def _render(self):
        self._resize_job = None
        c = self.canvas; c.delete("all"); c.configure(cursor="")
        w, h = max(c.winfo_width(), 50), max(c.winfo_height(), 50)
        m = self._px(12)
        self._imgs = []
        if not self._png_paths:
            msg = ("No plots in the output folder yet." if self._plot_dirs()
                   else "No plots yet. Open a design with results, or run Analyze, 3D check or Full report.")
            c.create_text(w // 2, h // 2, text=msg, fill=MUTED, font=self.heading, width=w - 2 * m, justify="center")
            return
        if self.plot_view.get() == "one":
            img = self._fit(self._current_png(), w - 2 * m, h - 2 * m)
            if img:
                self._imgs.append(img); c.create_image(w // 2, h // 2, image=img, anchor="center")
            return
        cap = self.linespace + self._px(6)                  # caption under each plot
        n = len(self._png_paths); cols = self._grid(w - m, h - m, cap); rows = -(-n // cols)
        cw, ch = (w - m) / cols, (h - m) / rows
        many = len(self._plot_dirs()) > 1
        for k, q in enumerate(self._png_paths):
            r, col = divmod(k, cols)
            x0 = m / 2 + col * cw + (cw * (rows * cols - n) / 2 if r == rows - 1 else 0)   # centre a short last row
            y0 = m / 2 + r * ch
            img = self._fit(q, int(cw - m), int(ch - m - cap))
            if not img:
                continue
            self._imgs.append(img); tag = f"plot{k}"
            iy = y0 + (ch - cap - img.height()) / 2
            c.create_image(x0 + cw / 2, iy, image=img, anchor="n", tags=tag)
            c.create_text(x0 + cw / 2, iy + img.height() + self._px(4), text=self._png_label(q, many),
                          anchor="n", fill=MUTED, tags=tag)
            c.tag_bind(tag, "<Button-1>", lambda e, k=k: self._show_png(view="one", index=k))
            c.tag_bind(tag, "<Enter>", lambda e: c.configure(cursor="hand2"))
            c.tag_bind(tag, "<Leave>", lambda e: c.configure(cursor=""))

def main():
    App().mainloop()


if __name__ == "__main__":
    main()
