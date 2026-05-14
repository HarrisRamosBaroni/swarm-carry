"""
Recording reviewer — visually inspect, trim, and rename JSONL recordings
produced by real_robot/laptop/record.py.

Usage:
  python3 -m real_robot.laptop.review                       # browse default dir
  python3 -m real_robot.laptop.review path/to/file.jsonl    # open one file
  python3 -m real_robot.laptop.review path/to/dir/          # browse a dir

Layout (one matplotlib window per recording, swap with Prev/Next):
  Left  : XY trajectory per robot + payload, colored by time.
  Right : stacked time-series — pose theta, |v| from diffed pose,
          load-cell force per robot, commanded |v|. Vertical markers for
          goal updates (green), estop (red), ctrl_stop (orange).
  Bottom: two trim sliders (t_in / t_out) — drawn across every time-series
          subplot. "Trim & Save" writes a new <stamp>_<name>_trimmed.jsonl
          plus updated .meta.json. Name / Notes text boxes + "Save Meta"
          renames the active file and rewrites the meta sidecar.
  Sidebar: per-topic row counts, effective Hz, max gap, robots seen,
           goal/estop/ctrl_stop counts. Red entries flag suspicious data.
"""
import argparse
import json
import shutil
import sys
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, TextBox


PAYLOAD_ID = -1
ROBOT_COLORS = ["tab:blue", "tab:orange", "tab:green", "tab:red",
                "tab:purple", "tab:brown", "tab:pink", "tab:gray"]
PAYLOAD_COLOR = "black"
DEFAULT_DIR = Path("real_robot/recordings")


def _color_for(rid: int) -> str:
    if rid == PAYLOAD_ID:
        return PAYLOAD_COLOR
    return ROBOT_COLORS[rid % len(ROBOT_COLORS)]


def _label_for(rid: int) -> str:
    return "payload" if rid == PAYLOAD_ID else f"robot {rid}"


@dataclass
class Recording:
    path: Path
    meta_path: Path
    meta: dict
    rows: list = field(default_factory=list)         # (t, topic, data)
    poses: dict = field(default_factory=dict)        # rid -> (t[], x[], y[], theta[])
    forces: dict = field(default_factory=dict)       # rid -> (t[], total[], readings_dict)
    cmds: dict = field(default_factory=dict)         # rid -> (t[], vmag[])
    goals: list = field(default_factory=list)        # (t, x, y, theta)
    estops: list = field(default_factory=list)
    ctrl_stops: list = field(default_factory=list)
    t0: float = 0.0
    t_end: float = 0.0

    @property
    def duration(self) -> float:
        return max(0.0, self.t_end - self.t0)


def _load(path: Path) -> Recording:
    meta_path = path.with_suffix(".meta.json")
    meta = {}
    if meta_path.exists():
        with open(meta_path) as f:
            meta = json.load(f)
    rec = Recording(path=path, meta_path=meta_path, meta=meta)

    poses = defaultdict(lambda: ([], [], [], []))
    forces = defaultdict(lambda: ([], [], defaultdict(list)))  # t, total, label->vals
    cmds = defaultdict(lambda: ([], []))

    t_min = float("inf")
    t_max = 0.0

    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                continue
            t = row.get("t")
            topic = row.get("topic")
            data = row.get("data") or {}
            if t is None or topic is None:
                continue
            t_min = min(t_min, t)
            t_max = max(t_max, t)
            rec.rows.append((t, topic, data))

            if topic == "pose":
                rid = data.get("id")
                if rid is None:
                    continue
                ts, xs, ys, ths = poses[rid]
                ts.append(t)
                xs.append(data.get("x", np.nan))
                ys.append(data.get("y", np.nan))
                ths.append(data.get("theta", np.nan))
            elif topic == "force":
                rid = data.get("id")
                if rid is None:
                    continue
                ts, totals, by_label = forces[rid]
                ts.append(t)
                readings = data.get("readings") or []
                total = 0.0
                for r in readings:
                    v = r.get("value", 0.0)
                    by_label[r.get("label", "?")].append((t, v))
                    total += float(v)
                totals.append(total)
            elif topic == "cmd":
                rid = data.get("id")
                if rid is None:
                    continue
                ts, vmags = cmds[rid]
                vmags.append(float(np.hypot(data.get("vx", 0.0), data.get("vy", 0.0))))
                ts.append(t)
            elif topic == "goal":
                rec.goals.append((t, data.get("x"), data.get("y"), data.get("theta")))
            elif topic == "estop":
                rec.estops.append(t)
            elif topic == "ctrl_stop":
                rec.ctrl_stops.append(t)

    rec.t0 = t_min if t_min != float("inf") else 0.0
    rec.t_end = t_max
    rec.poses = {rid: (np.asarray(ts), np.asarray(xs), np.asarray(ys), np.asarray(ths))
                 for rid, (ts, xs, ys, ths) in poses.items()}
    rec.forces = {rid: (np.asarray(ts), np.asarray(totals), {k: np.asarray(v) for k, v in by_label.items()})
                  for rid, (ts, totals, by_label) in forces.items()}
    rec.cmds = {rid: (np.asarray(ts), np.asarray(vmags))
                for rid, (ts, vmags) in cmds.items()}
    return rec


def _diff_speed(ts: np.ndarray, xs: np.ndarray, ys: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    if len(ts) < 2:
        return ts, np.zeros_like(ts)
    dt = np.diff(ts)
    dt[dt <= 0] = np.nan
    vx = np.diff(xs) / dt
    vy = np.diff(ys) / dt
    speed = np.hypot(vx, vy)
    return ts[1:], speed


def _health(rec: Recording) -> list[tuple[str, str, bool]]:
    """Return list of (label, value, is_bad)."""
    out: list[tuple[str, str, bool]] = []
    dur = rec.duration if rec.duration > 0 else 1e-9
    out.append(("duration", f"{rec.duration:.1f}s", rec.duration < 0.5))
    out.append(("rows", str(len(rec.rows)), False))

    n_robots = sum(1 for rid in rec.poses if rid != PAYLOAD_ID)
    out.append(("robots seen", str(n_robots), n_robots == 0))
    out.append(("payload pose", "yes" if PAYLOAD_ID in rec.poses else "no", False))

    for rid in sorted(rec.poses):
        ts, xs, ys, _ = rec.poses[rid]
        hz = len(ts) / dur
        gaps = np.diff(ts) if len(ts) >= 2 else np.array([0.0])
        max_gap = float(gaps.max()) if len(gaps) else 0.0
        nan_frac = float(np.mean(np.isnan(xs) | np.isnan(ys))) if len(xs) else 0.0
        bad = hz < 5.0 or max_gap > 1.0 or nan_frac > 0.01
        out.append((f"  pose {_label_for(rid)}", f"{hz:.1f}Hz gap≤{max_gap:.2f}s", bad))

    for rid in sorted(rec.forces):
        ts, totals, _ = rec.forces[rid]
        hz = len(ts) / dur
        out.append((f"  force {_label_for(rid)}",
                    f"{hz:.1f}Hz max={float(np.max(np.abs(totals))):.1f}" if len(totals) else "no data",
                    hz < 1.0))

    for rid in sorted(rec.cmds):
        ts, _ = rec.cmds[rid]
        hz = len(ts) / dur
        out.append((f"  cmd {_label_for(rid)}", f"{hz:.1f}Hz", hz < 1.0))

    out.append(("goals", str(len(rec.goals)), False))
    out.append(("estop", str(len(rec.estops)), len(rec.estops) > 0))
    out.append(("ctrl_stop", str(len(rec.ctrl_stops)), False))
    return out


class ReviewerApp:
    def __init__(self, paths: list[Path], start_idx: int = 0):
        self.paths = paths
        self.idx = start_idx
        self.rec: Optional[Recording] = None
        self.fig = plt.figure(figsize=(16, 9))
        self.fig.canvas.manager.set_window_title("recording reviewer")
        self._build_axes()
        self._connect_widgets()
        self._load_current()

    # ---- layout ----
    def _build_axes(self):
        gs = self.fig.add_gridspec(
            nrows=6, ncols=4,
            left=0.05, right=0.98, top=0.95, bottom=0.20,
            hspace=0.45, wspace=0.35,
            width_ratios=[1.2, 1.0, 1.0, 0.7],
        )
        self.ax_xy = self.fig.add_subplot(gs[:, 0])
        self.ax_xy.set_aspect("equal")
        self.ax_xy.set_xlabel("x (m)")
        self.ax_xy.set_ylabel("y (m)")
        self.ax_xy.grid(alpha=0.3)

        self.ax_theta = self.fig.add_subplot(gs[0, 1:3])
        self.ax_speed = self.fig.add_subplot(gs[1, 1:3], sharex=self.ax_theta)
        self.ax_force = self.fig.add_subplot(gs[2, 1:3], sharex=self.ax_theta)
        self.ax_cmd = self.fig.add_subplot(gs[3, 1:3], sharex=self.ax_theta)
        for ax, lab in [(self.ax_theta, "θ (rad)"),
                        (self.ax_speed, "|v| pose (m/s)"),
                        (self.ax_force, "force"),
                        (self.ax_cmd, "|v| cmd (m/s)")]:
            ax.set_ylabel(lab)
            ax.grid(alpha=0.3)
        self.ax_cmd.set_xlabel("t (s, since start)")
        self.ts_axes = [self.ax_theta, self.ax_speed, self.ax_force, self.ax_cmd]

        # health sidebar
        self.ax_health = self.fig.add_subplot(gs[:, 3])
        self.ax_health.axis("off")

        # bottom widgets
        self.ax_slider_in = self.fig.add_axes([0.08, 0.155, 0.55, 0.020])
        self.ax_slider_out = self.fig.add_axes([0.08, 0.125, 0.55, 0.020])
        self.ax_slider_tol = self.fig.add_axes([0.08, 0.095, 0.55, 0.020])

        self.ax_name = self.fig.add_axes([0.10, 0.04, 0.20, 0.035])
        self.ax_notes = self.fig.add_axes([0.36, 0.04, 0.28, 0.035])

        self.ax_btn_prev = self.fig.add_axes([0.70, 0.13, 0.06, 0.04])
        self.ax_btn_next = self.fig.add_axes([0.77, 0.13, 0.06, 0.04])
        self.ax_btn_reload = self.fig.add_axes([0.84, 0.13, 0.06, 0.04])
        self.ax_btn_save_meta = self.fig.add_axes([0.70, 0.07, 0.13, 0.04])
        self.ax_btn_trim = self.fig.add_axes([0.84, 0.07, 0.13, 0.04])

    def _connect_widgets(self):
        self.slider_in = Slider(self.ax_slider_in, "trim in", 0.0, 1.0, valinit=0.0)
        self.slider_out = Slider(self.ax_slider_out, "trim out", 0.0, 1.0, valinit=1.0)
        self.slider_tol = Slider(self.ax_slider_tol, "goal tol (m)", 0.01, 1.0, valinit=0.2)
        self.slider_in.on_changed(self._on_slider)
        self.slider_out.on_changed(self._on_slider)
        self.slider_tol.on_changed(lambda _v: self._redraw_xy())

        self.tb_name = TextBox(self.ax_name, "name ", initial="")
        self.tb_notes = TextBox(self.ax_notes, "notes ", initial="")

        self.btn_prev = Button(self.ax_btn_prev, "◀ prev")
        self.btn_next = Button(self.ax_btn_next, "next ▶")
        self.btn_reload = Button(self.ax_btn_reload, "reload")
        self.btn_save_meta = Button(self.ax_btn_save_meta, "save name/notes")
        self.btn_trim = Button(self.ax_btn_trim, "trim & save")

        self.btn_prev.on_clicked(lambda _e: self._step(-1))
        self.btn_next.on_clicked(lambda _e: self._step(+1))
        self.btn_reload.on_clicked(lambda _e: self._load_current())
        self.btn_save_meta.on_clicked(lambda _e: self._save_meta())
        self.btn_trim.on_clicked(lambda _e: self._trim_and_save())

        # trim-marker lines per timeseries axis (created on load)
        self.in_lines: list = []
        self.out_lines: list = []

    # ---- navigation ----
    def _step(self, delta: int):
        if not self.paths:
            return
        self.idx = (self.idx + delta) % len(self.paths)
        self._load_current()

    def _load_current(self):
        if not self.paths:
            self.fig.suptitle("(no recordings)")
            return
        path = self.paths[self.idx]
        try:
            self.rec = _load(path)
        except Exception as e:
            self.fig.suptitle(f"failed to load {path.name}: {e}")
            self.fig.canvas.draw_idle()
            return
        # reset name/notes
        self.tb_name.set_val(self.rec.meta.get("name", ""))
        self.tb_notes.set_val(self.rec.meta.get("notes", ""))

        dur = self.rec.duration
        # reset sliders to full range
        for s in (self.slider_in, self.slider_out):
            s.valmin = 0.0
            s.valmax = max(dur, 1e-3)
            s.ax.set_xlim(s.valmin, s.valmax)
        self.slider_in.set_val(0.0)
        self.slider_out.set_val(max(dur, 1e-3))
        # seed tol slider from the last goal row that carried a tol field
        tol_seed = 0.2
        for _, topic, data in self.rec.rows:
            if topic == "goal" and isinstance(data, dict) and "tol" in data:
                tol_seed = float(data["tol"])
        self.slider_tol.set_val(tol_seed)

        self._redraw()

    # ---- drawing ----
    def _redraw_xy(self):
        """Redraw only the XY map — respects current trim window and tol slider."""
        rec = self.rec
        if rec is None:
            return
        self.ax_xy.clear()
        self.ax_xy.grid(alpha=0.3)
        self.ax_xy.set_aspect("equal")
        self.ax_xy.set_xlabel("x (m)")
        self.ax_xy.set_ylabel("y (m)")

        t0 = rec.t0
        t_in_abs = t0 + self.slider_in.val
        t_out_abs = t0 + self.slider_out.val
        tol = float(self.slider_tol.val)

        from matplotlib.lines import Line2D
        legend_handles: list = []
        for rid in sorted(rec.poses):
            ts, xs, ys, _ = rec.poses[rid]
            if len(ts) == 0:
                continue
            mask = (ts >= t_in_abs) & (ts <= t_out_abs)
            if not mask.any():
                continue
            xs_m, ys_m, ts_m = xs[mask], ys[mask], ts[mask]
            color = _color_for(rid)
            self.ax_xy.plot(xs_m, ys_m, "-", color=color, alpha=0.5, linewidth=1.2)
            self.ax_xy.scatter(xs_m, ys_m, c=ts_m - t0, cmap="viridis", s=4)
            self.ax_xy.plot(xs_m[0], ys_m[0], "o", color=color, ms=8, mec="black")
            self.ax_xy.plot(xs_m[-1], ys_m[-1], "s", color=color, ms=8, mec="black")
            legend_handles.append(Line2D([0], [0], color=color, marker="o",
                                         linewidth=2, label=_label_for(rid)))

        # goals within trim window — latest solid, earlier faded
        goals_in = [(t, x, y, th) for (t, x, y, th) in rec.goals
                    if t_in_abs <= t <= t_out_abs and x is not None and y is not None]
        for i, (t, x, y, _th) in enumerate(goals_in):
            is_last = (i == len(goals_in) - 1)
            alpha_pt = 0.95 if is_last else 0.35
            alpha_ring = 0.7 if is_last else 0.25
            self.ax_xy.plot(x, y, marker="*", color="green", ms=14,
                            mec="black", alpha=alpha_pt, zorder=5)
            ring = plt.Circle((x, y), tol, fill=False, linestyle="--",
                              edgecolor="green", linewidth=1.2, alpha=alpha_ring)
            self.ax_xy.add_patch(ring)
        if goals_in:
            legend_handles.append(Line2D([0], [0], marker="*", color="green",
                                         mec="black", linewidth=0, markersize=10,
                                         label=f"goal (tol={tol:.2f}m)"))

        if legend_handles:
            self.ax_xy.legend(handles=legend_handles, loc="best", fontsize=8)
        self.fig.canvas.draw_idle()

    def _redraw(self):
        for ax in self.ts_axes:
            ax.clear()
            ax.grid(alpha=0.3)
        for ax, lab in [(self.ax_theta, "θ (rad)"),
                        (self.ax_speed, "|v| pose (m/s)"),
                        (self.ax_force, "force"),
                        (self.ax_cmd, "|v| cmd (m/s)")]:
            ax.set_ylabel(lab)
        self.ax_cmd.set_xlabel("t (s, since start)")

        rec = self.rec
        if rec is None:
            return
        t0 = rec.t0

        self._redraw_xy()

        # theta
        for rid in sorted(rec.poses):
            ts, _, _, ths = rec.poses[rid]
            if len(ts) == 0:
                continue
            self.ax_theta.plot(ts - t0, ths, color=_color_for(rid),
                               label=_label_for(rid), linewidth=0.9)
        self.ax_theta.legend(loc="upper right", fontsize=7, ncol=4)

        # speed (diffed pose)
        for rid in sorted(rec.poses):
            ts, xs, ys, _ = rec.poses[rid]
            tt, sp = _diff_speed(ts, xs, ys)
            if len(tt) == 0:
                continue
            self.ax_speed.plot(tt - t0, sp, color=_color_for(rid), linewidth=0.9)

        # force: one line per (robot, axis). horizontal = solid, vertical = dashed.
        _LINESTYLE = {"horizontal": "-", "vertical": "--"}
        for rid in sorted(rec.forces):
            _ts, _totals, by_label = rec.forces[rid]
            for lab, arr in by_label.items():
                if len(arr) == 0:
                    continue
                self.ax_force.plot(arr[:, 0] - t0, arr[:, 1],
                                   color=_color_for(rid),
                                   linestyle=_LINESTYLE.get(lab, ":"),
                                   linewidth=0.9,
                                   label=f"{_label_for(rid)} {lab}")
        if any(len(rec.forces[r][0]) for r in rec.forces):
            self.ax_force.legend(loc="upper right", fontsize=7, ncol=4)

        # cmd
        for rid in sorted(rec.cmds):
            ts, vmags = rec.cmds[rid]
            if len(ts) == 0:
                continue
            self.ax_cmd.plot(ts - t0, vmags, color=_color_for(rid),
                             linewidth=0.9, label=_label_for(rid))

        # event vlines
        for ax in self.ts_axes:
            for t, *_ in rec.goals:
                ax.axvline(t - t0, color="green", linewidth=0.6, alpha=0.4)
            for t in rec.estops:
                ax.axvline(t - t0, color="red", linewidth=1.0, alpha=0.7)
            for t in rec.ctrl_stops:
                ax.axvline(t - t0, color="orange", linewidth=0.8, alpha=0.6)

        # trim marker lines
        self.in_lines = [ax.axvline(self.slider_in.val, color="black",
                                    linestyle="--", linewidth=1.0) for ax in self.ts_axes]
        self.out_lines = [ax.axvline(self.slider_out.val, color="black",
                                     linestyle="--", linewidth=1.0) for ax in self.ts_axes]

        # title
        name = rec.meta.get("name", "")
        mode = rec.meta.get("mode", "?")
        ctrl = rec.meta.get("controller", "?")
        self.fig.suptitle(
            f"[{self.idx + 1}/{len(self.paths)}] {rec.path.name}  "
            f"— {mode}/{ctrl}  {rec.duration:.1f}s  {len(rec.rows)} rows",
            fontsize=10,
        )

        # health sidebar
        self.ax_health.clear()
        self.ax_health.axis("off")
        lines = _health(rec)
        y = 1.0
        self.ax_health.text(0.0, y, "health", weight="bold", fontsize=10,
                            transform=self.ax_health.transAxes)
        y -= 0.04
        for label, val, bad in lines:
            color = "red" if bad else "black"
            self.ax_health.text(0.0, y, f"{label:<18}{val}", color=color,
                                family="monospace", fontsize=8,
                                transform=self.ax_health.transAxes)
            y -= 0.028
            if y < 0.0:
                break

        self.fig.canvas.draw_idle()

    def _on_slider(self, _val):
        if self.slider_out.val < self.slider_in.val:
            # swap visually by clamping out >= in
            self.slider_out.set_val(self.slider_in.val)
            return
        for ln in self.in_lines:
            ln.set_xdata([self.slider_in.val, self.slider_in.val])
        for ln in self.out_lines:
            ln.set_xdata([self.slider_out.val, self.slider_out.val])
        self._redraw_xy()

    # ---- actions ----
    def _save_meta(self):
        """Rename file pair to current name; persist notes."""
        if self.rec is None:
            return
        new_name = self.tb_name.text.strip()
        notes = self.tb_notes.text
        stamp = self.rec.meta.get("stamp_utc") or self.rec.path.stem.split("_")[0]

        safe = _safe(new_name)
        new_stem = f"{stamp}_{safe}" if safe else stamp
        new_jsonl = self.rec.path.with_name(new_stem + ".jsonl")
        new_meta = self.rec.path.with_name(new_stem + ".meta.json")

        if new_jsonl != self.rec.path and new_jsonl.exists():
            print(f"[review] refuse to overwrite {new_jsonl}")
            return

        if new_jsonl != self.rec.path:
            self.rec.path.rename(new_jsonl)
            if self.rec.meta_path.exists():
                self.rec.meta_path.rename(new_meta)
            # update paths list
            self.paths[self.idx] = new_jsonl
            self.rec.path = new_jsonl
            self.rec.meta_path = new_meta

        self.rec.meta["name"] = new_name
        self.rec.meta["notes"] = notes
        with open(self.rec.meta_path, "w") as f:
            json.dump(self.rec.meta, f, indent=2)
        print(f"[review] saved meta → {self.rec.path.name}")
        self._redraw()

    def _trim_and_save(self):
        if self.rec is None:
            return
        t_in = self.slider_in.val
        t_out = self.slider_out.val
        if t_out <= t_in:
            print("[review] trim out <= trim in, ignored")
            return
        t0 = self.rec.t0
        abs_in = t0 + t_in
        abs_out = t0 + t_out

        new_name = self.tb_name.text.strip()
        notes = self.tb_notes.text
        stamp = self.rec.meta.get("stamp_utc") or self.rec.path.stem.split("_")[0]
        safe = _safe(new_name)
        suffix = "_trimmed"
        stem = f"{stamp}_{safe}{suffix}" if safe else f"{stamp}{suffix}"
        out_jsonl = self.rec.path.with_name(stem + ".jsonl")
        out_meta = self.rec.path.with_name(stem + ".meta.json")
        # uniquify if exists
        n = 2
        while out_jsonl.exists():
            stem2 = f"{stem}_{n}"
            out_jsonl = self.rec.path.with_name(stem2 + ".jsonl")
            out_meta = self.rec.path.with_name(stem2 + ".meta.json")
            n += 1

        kept = 0
        with open(self.rec.path) as fin, open(out_jsonl, "w") as fout:
            for line in fin:
                line_s = line.strip()
                if not line_s:
                    continue
                try:
                    row = json.loads(line_s)
                except json.JSONDecodeError:
                    continue
                t = row.get("t")
                if t is None or t < abs_in or t > abs_out:
                    continue
                fout.write(line)
                kept += 1

        meta = dict(self.rec.meta)
        meta.update({
            "name": new_name or meta.get("name", ""),
            "notes": notes,
            "started_at": abs_in,
            "stopped_at": abs_out,
            "duration_s": abs_out - abs_in,
            "row_count": kept,
            "trimmed_from": self.rec.path.name,
            "trim_in_s": t_in,
            "trim_out_s": t_out,
        })
        with open(out_meta, "w") as f:
            json.dump(meta, f, indent=2)
        print(f"[review] trimmed → {out_jsonl.name} ({kept} rows, {meta['duration_s']:.1f}s)")
        # refresh path list to include new file
        self.paths = _collect_paths(self.paths[0].parent)
        try:
            self.idx = self.paths.index(out_jsonl)
        except ValueError:
            pass
        self._load_current()


def _safe(name: str) -> str:
    keep = "-_."
    return "".join(c if c.isalnum() or c in keep else "_" for c in name).strip("_")


def _collect_paths(root: Path) -> list[Path]:
    if root.is_file():
        return [root]
    return sorted(p for p in root.glob("*.jsonl") if not p.name.startswith("_active_"))


def _parse_filter_file(filter_path: Path, recordings_dir: Path) -> list[Path]:
    """Extract .jsonl filenames from a highlight list and resolve them against recordings_dir."""
    paths = []
    with open(filter_path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            # find the first token ending in .jsonl (handles any column layout)
            for token in line.split():
                if token.endswith(".jsonl"):
                    candidate = recordings_dir / token
                    if candidate.exists():
                        paths.append(candidate)
                    else:
                        print(f"[review] filter: not found: {candidate}")
                    break
    return paths


def main():
    p = argparse.ArgumentParser()
    p.add_argument("target", nargs="?", default=str(DEFAULT_DIR),
                   help="file or directory (default: real_robot/recordings/)")
    p.add_argument("--filter", metavar="FILE",
                   help="text file listing .jsonl names to navigate (one per line)")
    args = p.parse_args()

    target = Path(args.target)
    if not target.exists():
        print(f"[review] not found: {target}")
        sys.exit(1)

    if target.is_file():
        all_paths = _collect_paths(target.parent)
        recordings_dir = target.parent
        start = all_paths.index(target) if target in all_paths else 0
    else:
        all_paths = _collect_paths(target)
        recordings_dir = target
        start = 0

    if args.filter:
        filter_path = Path(args.filter)
        if not filter_path.exists():
            print(f"[review] filter file not found: {filter_path}")
            sys.exit(1)
        paths = _parse_filter_file(filter_path, recordings_dir)
        if not paths:
            print(f"[review] no matching recordings found from filter {filter_path}")
            sys.exit(1)
        print(f"[review] filter: {len(paths)} recordings selected")
        start = 0
    else:
        paths = all_paths

    if not paths:
        print(f"[review] no recordings in {target}")
        sys.exit(1)

    app = ReviewerApp(paths, start_idx=start)
    plt.show()


if __name__ == "__main__":
    main()
