"""Steering-geometry optimiser for the front axle (3D).

Places the outer tie rod and the rack line so Ackermann and the steering ratio hit their
targets, while the tie rod keeps its rim clearance at full lock, the steering arm and tie
rod stay away from a straight line, and bump steer stays small. Among the layouts that do
all that, it picks the one closest to the current design.

Design variables (VSusp / design-file conventions):

* ``tie_rod_in``   outer tie rod, mm inboard of the hub face (``knuckle.tie_rod[0]``)
* ``tie_rod_down`` outer tie rod, mm below the hub centre (``knuckle.tie_rod[1]``)
* ``tie_rod_x``    outer tie rod, mm ahead of the axle (``three_d.tie_rod.x``; behind = negative)
* ``rack_x``       rack line, mm ahead of the axle (``tie_rod.x + tie_rod.inner_x_offset``)

The rack height and rack-end position still come from ``inner: auto`` (or ``{y: ...}``), so
bump steer stays at its minimum for each layout.
"""
from __future__ import annotations

import copy
import time

import numpy as np
from scipy.optimize import least_squares

from . import config as cfgmod
from .analysis2d import static_summary
from .clearance import ClearanceModel
from .geometry3d import SolveError3D
from .layout3d import build_corner
from .steering import ackermann_percent, arm_tie_rod_angle, rack_for_left_steer

STEERING_VARS = ("tie_rod_in", "tie_rod_down", "tie_rod_x", "rack_x")

DEFAULT_STEERING_TARGETS = dict(
    ackermann=None,              # %, at three_d.steering.ackermann_at (inside wheel). None = off
    ackermann_tol=0.5,           # %, normalisation for the Ackermann miss
    rack_per_deg=None,           # mm of rack travel per degree of wheel steer at centre. None = off
    rack_per_deg_tol=0.005,      # mm/deg, normalisation for the rack_per_deg miss
    steering_ratio=None,         # steering-wheel deg per wheel deg at centre; needs three_d.steering.c_factor
    tie_rod_clearance_min=None,  # mm, tie rod to rim at full lock (+/-15 mm heave). None = the current value
    arm_tie_rod_angle_min=30.0,  # deg, steering arm to tie rod at lock and centre (and 180 minus this)
    bump_steer_max=0.05,         # deg, max toe change over +/-30 mm
)

_HEAVES = (-15.0, 0.0, 15.0)       # clearance at lock is checked at these heaves (as the 3D check)
_STEP = 0.25                       # mm, finite-difference step
_REG = 0.1                         # weight of "stay close to the current design" (per 10 mm moved)


def wants_steering(targets: dict | None) -> bool:
    t = targets or {}
    return any(t.get(k) is not None for k in ("ackermann", "rack_per_deg", "steering_ratio"))


def steering_values(cfg: dict) -> dict:
    """Current values of the steering variables for the front axle."""
    f = cfg["front"]; tr = f["three_d"].get("tie_rod", {}) or {}
    otr = f["knuckle"]["tie_rod"]
    x = float(tr.get("x", -65.0))
    inner = tr.get("inner", "auto")
    if isinstance(inner, dict) and all(k in inner for k in ("x", "y", "z")):
        rack_x = float(inner["x"])
    else:
        rack_x = x + float(tr.get("inner_x_offset", 0.0))
    return dict(tie_rod_in=float(otr[0]), tie_rod_down=float(otr[1]), tie_rod_x=x, rack_x=rack_x)


def default_bounds(values: dict) -> dict:
    """Outer tie rod +/-15 mm across and fore/aft; its height and the rack line stay put."""
    v = values
    return dict(tie_rod_in=[v["tie_rod_in"] - 15, v["tie_rod_in"] + 15], tie_rod_down=[v["tie_rod_down"]] * 2,
                tie_rod_x=[v["tie_rod_x"] - 15, v["tie_rod_x"] + 15], rack_x=[v["rack_x"]] * 2)


def apply_values(cfg: dict, values: dict) -> dict:
    """Design copy with the steering variables written into knuckle.tie_rod and three_d.tie_rod."""
    c = copy.deepcopy(cfg)
    f = c["front"]
    f["knuckle"]["tie_rod"] = [round(float(values["tie_rod_in"]), 2), round(float(values["tie_rod_down"]), 2)]
    tr = dict(f["three_d"].get("tie_rod", {}) or {})
    tr["x"] = round(float(values["tie_rod_x"]), 2)
    inner = tr.get("inner", "auto")
    if not (isinstance(inner, dict) and all(k in inner for k in ("x", "y", "z"))):
        off = round(float(values["rack_x"]) - tr["x"], 2)
        if abs(off) > 1e-9 or "inner_x_offset" in tr:
            tr["inner_x_offset"] = off
    f["three_d"]["tie_rod"] = tr
    return c


def resolve_targets(cfg: dict, targets: dict | None) -> dict:
    """Steering targets with defaults; ``steering_ratio`` becomes ``rack_per_deg`` via the c-factor."""
    T = dict(DEFAULT_STEERING_TARGETS)
    T.update({k: v for k, v in (targets or {}).items() if k in DEFAULT_STEERING_TARGETS})
    st = cfg["front"]["three_d"].get("steering", {}) or {}
    if T["steering_ratio"] is not None:
        if T["rack_per_deg"] is not None:
            raise ValueError("give either a rack_per_deg or a steering_ratio target, not both")
        cf = st.get("c_factor")
        if not cf:
            raise ValueError("a steering_ratio target needs front.three_d.steering.c_factor "
                             "(mm of rack travel per steering-wheel turn)")
        T["rack_per_deg"] = float(T["steering_ratio"]) * float(cf) / 360.0
    if (cfg["front"]["three_d"].get("tie_rod", {}) or {}).get("design"):
        raise ValueError("steering targets move knuckle.tie_rod and three_d.tie_rod.x: use those instead of "
                         "three_d.tie_rod.design (check3d --sync writes them)")
    return T


def check_free(bounds: dict):
    if not any(float(bounds[k][1]) > float(bounds[k][0]) for k in STEERING_VARS):
        raise ValueError("steering targets need a steering variable that can move: give optimise.front.bounds "
                         f"{', '.join(STEERING_VARS)} a range (lo < hi), and don't lock them all")


class _Evaluator:
    """Steering numbers for one layout, as few solves as the targets need."""

    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.P = cfgmod.axle_fv(cfg, "front")
        self.track = static_summary(self.P)["track"]
        st = cfg["front"]["three_d"].get("steering", {}) or {}
        self.lock = float(st.get("lock_inside", 27.0)); self.ack_at = float(st.get("ackermann_at", 25.0))
        self.c_factor = st.get("c_factor")
        tyre, self.wheel = cfgmod.axle_tyre_wheel(cfg, "front")
        self.tire_r = tyre["diameter"] / 2
        self.calls = 0

    def __call__(self, values: dict, clearance=True) -> dict:
        self.calls += 1
        c = apply_values(self.cfg, values)
        P = dict(self.P); P["otr_x"], P["otr_y"] = c["front"]["knuckle"]["tie_rod"]
        corner, info = build_corner(P, c["front"]["three_d"], c["vehicle"], front=True)
        wb = c["vehicle"]["wheelbase"]
        m = dict(itr=info["itr"], otr=info["otr"], tie_rod_len=info["link_length"])
        r_ack = rack_for_left_steer(corner, self.ack_at)
        di = corner.solve_path(0, r_ack)["steer"]; do = -corner.solve_path(0, -r_ack)["steer"]
        m["ackermann"] = float(ackermann_percent(di, do, self.track, wb)[0])
        corner.reset(); m["rack_per_deg"] = float(1.0 / abs(corner.solve(0, 1.0)["steer"])); corner.reset()
        if self.c_factor:
            m["steering_ratio"] = m["rack_per_deg"] * 360.0 / float(self.c_factor)
        r_lock = rack_for_left_steer(corner, self.lock)
        sl, sr = corner.solve_path(0, r_lock), corner.solve_path(0, -r_lock)
        m["outside_lock"], m["rack_for_lock"] = float(-sr["steer"]), float(abs(r_lock))
        m["angles"] = [arm_tie_rod_angle(sl), arm_tie_rod_angle(corner.solve_path(0, 0.0)), arm_tie_rod_angle(sr)]
        corner.reset()
        m["bump_steer"] = float(max(abs(corner.solve(z)["toe_in"]) for z in (-30, -20, -10, 10, 20, 30)))
        corner.reset()
        if clearance:
            # tightest near full lock: check 70 % and 100 % of the lock travel each way, at each heave,
            # walking out from the centre once per direction (continuation, as solve_path)
            clr = ClearanceModel(corner, self.wheel, self.tire_r)
            worst = {}
            for z in _HEAVES:
                for sign in (1.0, -1.0):
                    corner.solve(z, 0.0)
                    stops = {round(0.7 * r_lock, 9), round(r_lock, 9)}
                    rs = sorted(set(np.arange(4.0, abs(r_lock), 4.0).round(9)) | {abs(s) for s in stops})
                    for r in rs:
                        s = corner.solve(z, sign * r)
                        if any(abs(r - abs(t)) < 1e-6 for t in stops):
                            for k, v in clr.evaluate(s).items():
                                worst[k] = min(v, worst.get(k, np.inf))
                corner.reset()
            m["clearance"] = {k: float(v) for k, v in worst.items()}
        return m


def optimise_steering(cfg: dict, bounds: dict, targets: dict | None = None, quick: bool = False,
                      progress=print) -> dict:
    """Place the outer tie rod and the rack line of the front axle for the steering targets.

    ``bounds``: [lo, hi] per steering variable (see ``STEERING_VARS``), lo == hi fixes it.
    ``targets``: see ``DEFAULT_STEERING_TARGETS``. Returns the new ``values``, the new design
    (``cfg``), the steering numbers ``before`` and ``after``, and the target misses.
    """
    T = resolve_targets(cfg, targets)
    check_free(bounds)
    v0 = steering_values(cfg)
    ev = _Evaluator(cfg)
    before = ev(v0)
    if T["tie_rod_clearance_min"] is None:
        T["tie_rod_clearance_min"] = before["clearance"]["tie rod"]
    names = [k for k in STEERING_VARS if float(bounds[k][1]) > float(bounds[k][0])]
    lo = np.array([float(bounds[k][0]) for k in names]); hi = np.array([float(bounds[k][1]) for k in names])
    x0 = np.clip([v0[k] for k in names], lo, hi)
    amin = float(T["arm_tie_rod_angle_min"])

    def values_of(x):
        v = dict(v0); v.update(zip(names, map(float, x)))
        return v

    def residuals(x):
        try:
            m = ev(values_of(x))
        except (SolveError3D, ValueError, np.linalg.LinAlgError):
            return np.full(5 + len(names), 30.0)
        r = [((m["ackermann"] - T["ackermann"]) / T["ackermann_tol"]) if T["ackermann"] is not None else 0.0,
             ((m["rack_per_deg"] - T["rack_per_deg"]) / T["rack_per_deg_tol"]) if T["rack_per_deg"] is not None else 0.0,
             max(0.0, T["tie_rod_clearance_min"] - m["clearance"]["tie rod"]) / 0.5,
             (max(0.0, amin - min(m["angles"])) + max(0.0, max(m["angles"]) - (180.0 - amin))) / 2.0,
             max(0.0, m["bump_steer"] - T["bump_steer_max"]) / 0.01]
        r += list(_REG * (np.asarray(x) - x0) / 10.0)
        return np.array(r)

    def jac(x):
        f0 = residuals(x); J = np.empty((f0.size, x.size))
        for j in range(x.size):
            h = _STEP if x[j] + _STEP <= hi[j] else -_STEP
            xh = np.array(x, float); xh[j] += h
            J[:, j] = (residuals(xh) - f0) / h
        return J

    t0 = time.time()
    if np.max(np.abs(residuals(x0)[:5])) < 0.5:       # already within half a tolerance of everything
        values, after = dict(v0), before
        progress("  steering already meets its targets and limits: nothing moved")
    else:
        sol = least_squares(residuals, x0, jac=jac, bounds=(lo, hi), method="trf", x_scale=1.0,
                            max_nfev=8 if quick else 25, xtol=1e-4, ftol=1e-6)
        values = values_of(np.clip(np.round(sol.x, 1), lo, hi))     # 0.1 mm
        after = ev(values)
    misses = {}
    if T["ackermann"] is not None:
        misses["ackermann"] = after["ackermann"] - T["ackermann"]
    if T["rack_per_deg"] is not None:
        misses["rack_per_deg"] = after["rack_per_deg"] - T["rack_per_deg"]
    misses["tie_rod_clearance"] = min(0.0, after["clearance"]["tie rod"] - T["tie_rod_clearance_min"])
    if after is not before:
        progress(f"  steering placed in {time.time() - t0:.0f} s ({ev.calls} layouts)")
    notes = []
    if T["ackermann"] is not None and abs(misses["ackermann"]) > 2 * T["ackermann_tol"]:
        notes.append(f"Ackermann {after['ackermann']:.1f}% misses its {T['ackermann']:g}% target")
    if T["rack_per_deg"] is not None and abs(misses["rack_per_deg"]) > 2 * T["rack_per_deg_tol"]:
        notes.append(f"rack travel {after['rack_per_deg']:.3f} mm/deg misses its {T['rack_per_deg']:.3f} target")
    if misses["tie_rod_clearance"] < -0.25:
        notes.append(f"tie-rod clearance {after['clearance']['tie rod']:.1f} mm is under its "
                     f"{T['tie_rod_clearance_min']:.1f} mm minimum")
    for n in notes:
        progress(f"  note: {n}: free more of {', '.join(k for k in STEERING_VARS if k not in names) or 'the bounds'}"
                 " or relax the limits")
    return dict(values=values, start=v0, cfg=apply_values(cfg, values), before=before, after=after,
                targets=T, misses=misses, notes=notes, free=names)
