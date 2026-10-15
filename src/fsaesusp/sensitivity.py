"""Hardpoint tolerance sensitivity in 3D.

Each hardpoint coordinate (x, y, z of the arm pivots, the inner tie rod / toe link and the
upright points) is moved by +/- its tolerance, and the corner's kinematics are measured again.
Two effects are separated:

* **as built**: the arms and tie rod keep their nominal lengths, so the static camber and toe move.
  This is how far the alignment has to be adjusted (shims, tie-rod turns) after the build.
* **after alignment**: the links are made to fit, so the static camber and toe are back at nominal.
  What remains is the change in the kinematics (roll centre, camber gain, bump steer, anti, ...).

Results are linear sensitivities (central differences over +/- the tolerance), with a worst-case
and a root-sum-square (RSS) stack per metric for one corner.
"""
from __future__ import annotations

import numpy as np

from .geometry3d import Corner3D, SolveError3D, anti_values
from .steering import ackermann_percent, rack_for_left_steer

CHASSIS_PTS = ("UF", "UR", "LF", "LR", "ITR")
UPRIGHT_PTS = ("UBJ", "LBJ", "OTR")
POINTS = CHASSIS_PTS + UPRIGHT_PTS
POINT_LABELS = dict(UF="Upper arm front pivot", UR="Upper arm rear pivot", LF="Lower arm front pivot",
                    LR="Lower arm rear pivot", ITR="Inner tie rod / toe link", UBJ="Upper ball joint",
                    LBJ="Lower ball joint", OTR="Outer tie rod / toe link")

# key, label, format, front-only
METRICS = (
    ("camber_static", "Static camber as built (deg)", "{:+.3f}", False),
    ("toe_static", "Static toe-in as built (deg)", "{:+.3f}", False),
    ("rc_height", "Roll-centre height (mm)", "{:.1f}", False),
    ("rc_heave_slope", "RC change per mm heave", "{:+.3f}", False),
    ("camber_gain", "Camber gain (deg/mm)", "{:+.4f}", False),
    ("toe_bump", "Toe-in at +25 mm bump (deg)", "{:+.3f}", False),
    ("toe_droop", "Toe-in at -25 mm droop (deg)", "{:+.3f}", False),
    ("roll_steer", "Roll steer (deg toe / deg roll)", "{:+.4f}", False),
    ("anti", "Anti-dive / anti-squat (%)", "{:.1f}", False),
    ("caster", "Caster (deg)", "{:.2f}", False),
    ("trail", "Mechanical trail (mm)", "{:.1f}", False),
    ("kpi", "KPI (deg)", "{:.2f}", False),
    ("scrub", "Scrub radius (mm)", "{:+.1f}", False),
    ("ackermann", "Ackermann (%)", "{:.1f}", True),
)
# The static camber and toe are set at alignment, so their spread is the adjustment range needed, not a defect.
AS_BUILT = ("camber_static", "toe_static")
# How much each kinematic metric may move before it matters (``sensitivity.allow`` in the design file overrides).
DEFAULT_ALLOW = dict(rc_height=5.0, rc_heave_slope=0.1, camber_gain=0.005, toe_bump=0.05, toe_droop=0.05,
                     roll_steer=0.02, anti=5.0, caster=0.25, trail=2.0, kpi=0.5, scrub=2.0, ackermann=10.0)


def _rc(c, z, h):
    """Roll-centre height above the (raised) ground with both wheels at ``z``: the contact patch moves
    perpendicular to the line from it to the instant centre, so RC = half-track * dy/dz."""
    sp, sm = c.solve(z + h), c.solve(z - h)
    y = 0.5 * (sp["CP"][1] + sm["CP"][1])
    return float(y * (sp["CP"][1] - sm["CP"][1]) / (2 * h))


def kinematic_metrics(c: Corner3D, front: bool, vehicle: dict, rack_ack=None, track=None, h=0.25) -> dict:
    """The sensitivity metrics of one corner with its static alignment as built into ``c``."""
    s0 = c.reset()
    m = {k: float(s0[k]) for k in ("caster", "trail", "kpi", "scrub")}
    m["camber_static"], m["toe_static"] = float(s0["camber"]), float(s0["toe_in"])
    sp, sm = c.solve(h), c.solve(-h)
    m["camber_gain"] = float((sp["camber"] - sm["camber"]) / (2 * h))
    m["rc_height"] = _rc(c, 0.0, h)
    m["rc_heave_slope"] = (_rc(c, 2.0, h) - _rc(c, -2.0, h)) / 4.0
    c.reset(); m["toe_bump"] = float(c.solve(25.0)["toe_in"])
    c.reset(); m["toe_droop"] = float(c.solve(-25.0)["toe_in"])
    c.reset()
    m["roll_steer"] = float((c.solve(0, roll=h)["toe_in"] - c.solve(0, roll=-h)["toe_in"]) / (2 * h))
    c.reset()
    a = anti_values(c, front, vehicle["wheelbase"], vehicle["cg_height"], vehicle["front_brake_bias"],
                    vehicle.get("inboard_drive", True))
    m["anti"] = float(a["anti_dive" if front else "anti_squat"])
    if front and rack_ack is not None:
        di = c.solve_path(0, rack_ack)["steer"]; do = -c.solve_path(0, -rack_ack)["steer"]
        m["ackermann"] = float(ackermann_percent(di, do, track, vehicle["wheelbase"])[0])
    c.reset()
    return m


def _perturbed(c0: Corner3D, key: str, axis: int, d: float, keep_lengths: bool) -> Corner3D:
    hp = {k: v.copy() for k, v in c0.hp.items()}
    hp[key][axis] += d
    c = Corner3D(hp, c0.R, roll_z=c0.roll_z)
    if keep_lengths:
        c.L = dict(c0.L)
    return c


def corner_sensitivity(c0: Corner3D, front: bool, vehicle: dict, tol: dict, track=None, ackermann_at=25.0,
                       allow=None, points=None, progress=None) -> dict:
    """Sensitivity of one corner. ``tol``: tolerance (mm) per hardpoint key; ``allow``: overrides of
    ``DEFAULT_ALLOW``; ``points``: the hardpoints to move (default ``POINTS``).

    Returns ``nominal`` metrics, ``rows`` (one per point and axis: ``sens`` per mm and ``effect`` at the
    tolerance, both signed; ``share`` of each metric's worst-case stack; ``used``, the fraction of each
    kinematic metric's allowance) and ``stack`` (worst case, RSS and allowance per metric)."""
    al = dict(DEFAULT_ALLOW); al.update(allow or {})
    rack_ack = None
    if front:
        rack_ack = rack_for_left_steer(c0, ackermann_at)
    nominal = kinematic_metrics(c0, front, vehicle, rack_ack, track)
    keys = [k for k, *_ in METRICS if k in nominal]
    rows = []
    for key in points or POINTS:
        t = float(tol[key])
        for axis, ax in enumerate("xyz"):
            if progress:
                progress(f"  {key} {ax}")
            res = []
            for sgn in (1, -1):
                try:
                    built = _perturbed(c0, key, axis, sgn * t, keep_lengths=True).reset()
                    m = kinematic_metrics(_perturbed(c0, key, axis, sgn * t, keep_lengths=False), front, vehicle,
                                          rack_ack, track)
                    m["camber_static"], m["toe_static"] = float(built["camber"]), float(built["toe_in"])
                except SolveError3D:
                    m = {k: np.nan for k in keys}
                res.append(m)
            sens = {k: (res[0][k] - res[1][k]) / (2 * t) if t > 0 else 0.0 for k in keys}
            rows.append(dict(point=key, axis=ax, tol=t, sens=sens, effect={k: sens[k] * t for k in keys}))
    stack = {}
    for k in keys:
        e = np.array([abs(r["effect"][k]) for r in rows])
        e = e[np.isfinite(e)]
        stack[k] = dict(worst=float(e.sum()), rss=float(np.sqrt((e ** 2).sum())), allow=al.get(k))
        stack[k]["over"] = k not in AS_BUILT and stack[k]["rss"] > float(al[k])
    for r in rows:
        r["share"] = {k: (abs(r["effect"][k]) / stack[k]["worst"] if stack[k]["worst"] > 0 else 0.0) for k in keys}
        r["used"] = {k: abs(r["effect"][k]) / float(al[k]) for k in keys if k not in AS_BUILT}
    return dict(nominal=nominal, rows=rows, stack=stack, metrics=keys)


def tolerances(tol: float, tol_upright: float | None = None) -> dict:
    tu = tol if tol_upright is None else tol_upright
    return {k: (tu if k in UPRIGHT_PTS else tol) for k in CHASSIS_PTS + UPRIGHT_PTS}


def top_contributors(axle_res: dict, metric: str, n=3) -> list:
    """The ``n`` coordinates with the largest share of a metric's worst-case stack: [(label, share)]."""
    rows = sorted(axle_res["rows"], key=lambda r: -np.nan_to_num(r["share"][metric]))
    return [(f"{r['point']} {r['axis']}", r["share"][metric]) for r in rows[:n] if r["share"][metric] > 0.005]


def influence(axle_res: dict) -> list:
    """Coordinates ranked by the largest fraction of any kinematic metric's allowance they use on their own:
    [(row, fraction, metric)]."""
    out = []
    for r in axle_res["rows"]:
        k = max(r["used"], key=lambda m: np.nan_to_num(r["used"][m]))
        out.append((r, r["used"][k], k))
    return sorted(out, key=lambda x: -np.nan_to_num(x[1]))
