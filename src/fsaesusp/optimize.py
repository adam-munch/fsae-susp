"""Chassis-pivot optimiser for one axle (front view).

Design variables are the inner pivot positions (x from centre, height above
ground) of the lower and upper arm. Arm lengths are derived so the wheel keeps
its track and a chosen static camber, so the knuckle/upright is untouched.
The cost is a sum of squared, normalised target misses plus packaging penalties.
"""
from __future__ import annotations

import time
import numpy as np
from scipy.optimize import differential_evolution, minimize

from .geometry2d import Corner2D, axle_state, build_from_pose, SolveError
from .analysis2d import metrics, pick_static_camber

DEFAULT_TARGETS = dict(
    rc_height=30.0,              # mm, static roll-centre height
    rc_tol=5.0,                  # mm, normalisation for the RC height miss
    rc_heave_slope_max=0.5,      # mm/mm, allowed |dRC/dheave| before penalty
    rc_lateral_tol=15.0,         # mm/deg, normalisation for RC lateral migration
    fvsa_range=[900.0, 1300.0],  # mm, front-view swing-arm length band
    lower_arm_min=250.0,         # mm
    upper_arm_min=240.0,         # mm
    lca_incl_range=[-3.0, 4.0],  # deg, outer end up positive
    uca_incl_range=[-4.0, 20.0], # deg
    half_track_change_max=3.0,   # mm over +/-25 mm single-wheel travel
    rc_at_heave_min=5.0,         # mm, RC must stay above this at the aero heave case
    min_frame_depth=100.0,       # mm, upper pivot at least this far above the lower pivot
    optimise_camber=-1.8,        # deg, static camber used while optimising pivots
)

DEFAULT_CAMBER = dict(outside_target=-1.5, low_speed=[1.5, 0.0], high_speed=[1.8, 10.0])

# What `optimise.<axle>.lock` can hold at the design's current value: the four pivot coordinates
# (VSusp coordinates, as in the bounds) and the static camber.
LOCKABLE = ("lower_x", "lower_y", "upper_x", "upper_y", "camber")


def _knuckle(P):
    return {k: P[k] for k in ("hux", "huy", "hlx", "hly", "otr_x", "otr_y")}


def _cost(x, ctx, detail=False):
    xA, yA, xB, yB = x
    T = ctx["targets"]
    if yB - yA < T["min_frame_depth"]:
        return 1e3 + (T["min_frame_depth"] - (yB - yA))
    P = build_from_pose(ctx["knuckle"], (xA, yA), (xB, yB), ctx["half_track"], T["optimise_camber"],
                        ctx["ride"], ctx["tire_d"], ctx["compression"], ctx["offset"])
    pen = (max(0, T["lower_arm_min"] - P["lower_len"]) / 10) ** 2 + (max(0, T["upper_arm_min"] - P["upper_len"]) / 10) ** 2
    try:
        m = metrics(P, ctx["roll_op"], ctx["heave_op"])
    except (SolveError, ValueError, np.linalg.LinAlgError, ZeroDivisionError):
        return 1e3
    if not np.isfinite(m["rc"]):
        return 1e3
    lo, hi = T["lca_incl_range"]; pen += (max(0, m["lca_incl"] - hi) / 0.5) ** 2 + (max(0, lo - m["lca_incl"])) ** 2
    lo, hi = T["uca_incl_range"]; pen += (max(0, m["uca_incl"] - hi)) ** 2 + (max(0, lo - m["uca_incl"])) ** 2
    terms = dict(
        rc_height=((m["rc"] - T["rc_height"]) / T["rc_tol"]) ** 2,
        rc_heave=(max(0, abs(m["rc_heave_slope"]) - T["rc_heave_slope_max"]) / 0.25) ** 2,
        rc_lateral=(m["rc_lateral"] / T["rc_lateral_tol"]) ** 2,
        fvsa=(max(0, T["fvsa_range"][0] - m["fvsa"]) / 100) ** 2 + (max(0, m["fvsa"] - T["fvsa_range"][1]) / 100) ** 2,
        rc_at_heave=(max(0, T["rc_at_heave_min"] - m["rc_at_heave"]) / 5) ** 2,
        half_track=(max(0, abs(m["half_track_bump25"]) - T["half_track_change_max"]) / 1.5) ** 2
                   + (max(0, abs(m["half_track_droop25"]) - T["half_track_change_max"]) / 1.5) ** 2,
        packaging=pen,
    )
    J = float(sum(terms.values()))
    if detail:
        return J, terms, m, P
    return J


def optimise_axle(P0: dict, bounds: dict, targets: dict | None = None, camber: dict | None = None,
                  roll_op: float = 1.5, heave_op: float = 10.0, seed: int = 0, quick: bool = False,
                  workers: int = 1, static_camber: float | None = None, progress=print) -> dict:
    """Optimise the inner pivots of one axle.

    ``bounds``: {"lower_x": [lo, hi], "lower_y": [lo, hi], "upper_x": [...], "upper_y": [...]}
    with x from the frame centre and y **above the frame bottom** (VSusp convention).
    A range with lo == hi fixes that coordinate. ``static_camber`` (deg) keeps that
    static camber instead of choosing one for the outside-wheel target. Returns a dict
    with the new front-view parameters, metrics before/after and the cost breakdown.
    """
    T = dict(DEFAULT_TARGETS); T.update(targets or {})
    if static_camber is not None:
        T["optimise_camber"] = float(static_camber)
    C = dict(DEFAULT_CAMBER); C.update(camber or {})
    ride = float(P0["bottom_y"])
    s0 = axle_state(Corner2D(P0))
    ctx = dict(targets=T, knuckle=_knuckle(P0), half_track=float(s0["r"]["T"][0]), ride=ride,
               tire_d=float(P0["tire_d"]), compression=float(P0.get("compression", 0.0)),
               offset=float(P0["offset"]), roll_op=roll_op, heave_op=heave_op)
    order = ("lower_x", "lower_y", "upper_x", "upper_y")
    bnds = []
    for k in order:
        lo, hi = map(float, bounds[k])
        if k.endswith("_y"):
            lo, hi = lo + ride, hi + ride           # to height above ground
        bnds.append((lo, hi))
    free = [i for i, (lo, hi) in enumerate(bnds) if hi > lo]
    fixed = {i: bnds[i][0] for i in range(4) if i not in free}

    def full(xf):
        x = np.empty(4)
        for j, i in enumerate(free):
            x[i] = xf[j]
        for i, v in fixed.items():
            x[i] = v
        return x

    t0 = time.time()
    if free:
        fb = [bnds[i] for i in free]
        res = differential_evolution(_cost_free, fb, args=(ctx, free, fixed), seed=seed,
                                     popsize=10 if quick else 18, maxiter=60 if quick else 160,
                                     tol=1e-7, polish=False, workers=workers,
                                     updating="deferred" if workers != 1 else "immediate")
        loc = minimize(_cost_free, res.x, args=(ctx, free, fixed), method="Nelder-Mead", bounds=fb,
                       options=dict(xatol=0.02, fatol=1e-6, maxiter=400))
        xf = loc.x if loc.fun <= res.fun else res.x
        xf = np.clip(xf, [b[0] for b in fb], [b[1] for b in fb])
        xbest = full(xf)
    else:
        xbest = full([])
    # round pivots to 0.5 mm (staying inside the bounds)
    xbest = np.clip(np.round(xbest * 2) / 2, [b[0] for b in bnds], [b[1] for b in bnds])
    xA, yA, xB, yB = xbest
    J, terms, m_opt, _ = _cost(xbest, ctx, detail=True)
    if static_camber is not None:
        cam = float(static_camber)
    else:
        cam = pick_static_camber(ctx["knuckle"], (xA, yA), (xB, yB), ctx["half_track"], ride, ctx["tire_d"],
                                 ctx["compression"], ctx["offset"], C["outside_target"], tuple(C["low_speed"]),
                                 tuple(C["high_speed"]))
    P = build_from_pose(ctx["knuckle"], (xA, yA), (xB, yB), ctx["half_track"], cam, ride, ctx["tire_d"],
                        ctx["compression"], ctx["offset"], P0.get("tire_w", 200.0), P0.get("wheel_d", 254.0))
    nd = 1 if static_camber is None else 2          # 0.01 mm (VSusp's step) keeps a locked camber to ~0.005 deg
    P["upper_len"] = round(P["upper_len"], nd); P["lower_len"] = round(P["lower_len"], nd)
    progress(f"  optimised in {time.time() - t0:.0f} s, cost {J:.2f}")
    return dict(params=P, cost=J, terms=terms, static_camber=cam,
                before=metrics(P0, roll_op, heave_op), after=metrics(P, roll_op, heave_op),
                pivots=dict(lower_x=xA, lower_y=yA - ride, upper_x=xB, upper_y=yB - ride))


def _cost_free(xf, ctx, free, fixed):
    x = np.empty(4)
    for j, i in enumerate(free):
        x[i] = xf[j]
    for i, v in fixed.items():
        x[i] = v
    return _cost(x, ctx)
