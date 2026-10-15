"""Chassis-pivot optimiser for one axle (front view).

Design variables are the inner pivot positions (x from centre, height above
ground) of the lower and upper arm and, when their bounds let them move, the
ball joints' inboard positions on the upright (``upper_bj_x``, ``lower_bj_x``).
Arm lengths are derived so the wheel keeps its track and a chosen static camber.

With a ``kpi`` target, one ball joint is placed so the static KPI is exactly the
target: KPI = knuckle angle - camber, so the joint follows the static camber,
and the static camber is searched together with the pivots.
The cost is a sum of squared, normalised target misses plus packaging penalties.
"""
from __future__ import annotations

import time
import numpy as np
from scipy.optimize import brentq, differential_evolution, minimize

from .geometry2d import Corner2D, axle_state, build_from_pose, SolveError
from .analysis2d import metrics, pick_static_camber, outside_camber

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
    optimise_camber=-1.8,        # deg, static camber used while optimising pivots (without a kpi target)
    kpi=None,                    # deg, static KPI to hold exactly; needs upper_bj_x or lower_bj_x bounds. None = off
    scrub_target=0.0,            # mm, scrub radius aimed for when a ball joint can move
    scrub_tol=15.0,              # mm, normalisation for the scrub miss (only when a ball joint can move)
    outside_camber_tol=0.05,     # deg, normalisation for the outside-wheel camber miss when camber is searched
    camber_range=[-4.0, 0.5],    # deg, static camber search range (with a kpi target)
)

DEFAULT_CAMBER = dict(outside_target=-1.5, low_speed=[1.5, 0.0], high_speed=[1.8, 10.0])

# What `optimise.<axle>.lock` can hold at the design's current value: the four pivot coordinates
# (VSusp coordinates, as in the bounds), the ball joints' inboard positions, the static camber and, for the
# front steering stage (steeropt.py), the outer tie rod and the rack line.
LOCKABLE = ("lower_x", "lower_y", "upper_x", "upper_y", "upper_bj_x", "lower_bj_x", "camber",
            "tie_rod_in", "tie_rod_down", "tie_rod_x", "rack_x")
PIVOTS = ("lower_x", "lower_y", "upper_x", "upper_y")
BALL_JOINTS = ("upper_bj_x", "lower_bj_x")      # inboard of the hub face, as in the design file's knuckle
BJ_KEYS = dict(upper_bj_x="hux", lower_bj_x="hlx")


def _knuckle(P):
    return {k: P[k] for k in ("hux", "huy", "hlx", "hly", "otr_x", "otr_y")}


def knuckle_roles(bj_bounds: dict, kpi) -> tuple:
    """Which ball joints move and how: ``(derived, searched)``.

    ``bj_bounds`` maps ``upper_bj_x``/``lower_bj_x`` to ``(lo, hi)``; lo == hi fixes that joint.
    With a ``kpi`` target, ``derived`` is the joint placed to hold it (the upper one if it can move),
    and ``searched`` lists the free joints the search varies directly."""
    free = [n for n in BALL_JOINTS if bj_bounds[n][1] > bj_bounds[n][0]]
    if kpi is None:
        return None, free
    if not free:
        raise ValueError("a kpi target needs a ball joint that can move: give optimise.<axle>.bounds "
                         "upper_bj_x or lower_bj_x a range (lo < hi), and don't lock both")
    derived = "upper_bj_x" if "upper_bj_x" in free else "lower_bj_x"
    return derived, [n for n in free if n != derived]


def _knuckle_for(ctx, searched: dict, cam: float, derived=None) -> dict:
    """Knuckle with the searched ball joints set and the derived one (if any) placed for the KPI target.
    KPI = atan((hux - hlx) / (huy + hly)) - camber, exactly, at the static pose."""
    k = dict(ctx["knuckle"])
    for n, v in searched.items():
        k[BJ_KEYS[n]] = float(v)
    derived = ctx["derived"] if derived is None else derived
    if derived:
        a = ctx["kpi_h"] * np.tan(np.radians(ctx["targets"]["kpi"] + cam))     # hux - hlx
        if derived == "upper_bj_x":
            k["hux"] = k["hlx"] + a
        else:
            k["hlx"] = k["hux"] - a
    return k


def _build(ctx, k, lower_pivot, upper_pivot, cam):
    return build_from_pose(k, lower_pivot, upper_pivot, ctx["half_track"], cam, ctx["ride"], ctx["tire_d"],
                           ctx["compression"], ctx["offset"], ctx["tire_w"], ctx["wheel_d"])


def _outside_avg(P, C):
    """Outside-wheel camber averaged over the low- and high-speed cases (as pick_static_camber)."""
    c = Corner2D(P); th0 = c.solve()["theta"]
    lo = axle_state(c, C["low_speed"][0], C["low_speed"][1], C["low_speed"][1], theta0=th0)["cam_l"]
    hi = axle_state(c, C["high_speed"][0], C["high_speed"][1], C["high_speed"][1], theta0=th0)["cam_l"]
    return 0.5 * (lo + hi)


def _cost(x, ctx, detail=False):
    v = dict(zip(ctx["names"], x))
    xA, yA, xB, yB = (v[n] for n in PIVOTS)
    T = ctx["targets"]
    if yB - yA < T["min_frame_depth"]:
        return 1e3 + (T["min_frame_depth"] - (yB - yA))
    cam = v.get("camber", T["optimise_camber"])
    k = _knuckle_for(ctx, {n: v[n] for n in BALL_JOINTS if n in v}, cam)
    P = _build(ctx, k, (xA, yA), (xB, yB), cam)
    pen = (max(0, T["lower_arm_min"] - P["lower_len"]) / 10) ** 2 + (max(0, T["upper_arm_min"] - P["upper_len"]) / 10) ** 2
    if ctx["derived"]:                      # the joint placed for the KPI target must stay inside its bounds
        lo, hi = ctx["bj_bounds"][ctx["derived"]]; bj = k[BJ_KEYS[ctx["derived"]]]
        pen += (max(0, lo - bj) / 0.5) ** 2 + (max(0, bj - hi) / 0.5) ** 2
    try:
        m = metrics(P, ctx["roll_op"], ctx["heave_op"])
        out_avg = _outside_avg(P, ctx["camber"]) if "camber" in v else None
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
    )
    if ctx["bj_free"]:
        terms["scrub"] = ((m["scrub"] - T["scrub_target"]) / T["scrub_tol"]) ** 2
    if out_avg is not None:
        terms["outside_camber"] = ((out_avg - ctx["camber"]["outside_target"]) / T["outside_camber_tol"]) ** 2
    terms["packaging"] = pen
    J = float(sum(terms.values()))
    if detail:
        return J, terms, m, P
    return J


def _pick_camber_tied(ctx, lower_pivot, upper_pivot, searched: dict, derived=None):
    """Static camber for the outside-wheel target with the derived ball joint following it (KPI held)."""
    C = ctx["camber"]

    def knuckle(cam):
        return _knuckle_for(ctx, searched, cam, derived)

    def f(cam):
        P = _build(ctx, knuckle(cam), lower_pivot, upper_pivot, cam)
        return 0.5 * (outside_camber(P, *C["low_speed"]) + outside_camber(P, *C["high_speed"])) - C["outside_target"]
    cam = brentq(f, -5.0, 1.0, xtol=1e-6)
    return cam, knuckle(cam)


def optimise_axle(P0: dict, bounds: dict, targets: dict | None = None, camber: dict | None = None,
                  roll_op: float = 1.5, heave_op: float = 10.0, seed: int = 0, quick: bool = False,
                  workers: int = 1, static_camber: float | None = None, progress=print) -> dict:
    """Optimise the inner pivots (and, if their bounds allow, the ball joints) of one axle.

    ``bounds``: {"lower_x": [lo, hi], "lower_y": [lo, hi], "upper_x": [...], "upper_y": [...]}
    with x from the frame centre and y **above the frame bottom** (VSusp convention), and
    optionally "upper_bj_x"/"lower_bj_x": the ball joints' inboard distance from the hub face.
    A range with lo == hi fixes that coordinate; a ball joint without bounds stays where it is.
    ``static_camber`` (deg) keeps that static camber instead of choosing one for the outside-wheel
    target. ``targets["kpi"]`` (deg) holds the static KPI exactly by placing one free ball joint.
    Returns a dict with the new front-view parameters, metrics before/after and the cost breakdown.
    """
    T = dict(DEFAULT_TARGETS); T.update(targets or {})
    if static_camber is not None:
        T["optimise_camber"] = float(static_camber)
    C = dict(DEFAULT_CAMBER); C.update(camber or {})
    ride = float(P0["bottom_y"])
    s0 = axle_state(Corner2D(P0))
    bj_bounds = {n: tuple(map(float, bounds.get(n) or [P0[BJ_KEYS[n]]] * 2)) for n in BALL_JOINTS}
    kpi = T.get("kpi")
    if kpi is not None:
        T["kpi"] = kpi = float(kpi)
    derived, searched_bj = knuckle_roles(bj_bounds, kpi)
    search_camber = kpi is not None and static_camber is None
    names = list(PIVOTS) + searched_bj + (["camber"] if search_camber else [])
    ctx = dict(targets=T, camber=C, knuckle=_knuckle(P0), half_track=float(s0["r"]["T"][0]), ride=ride,
               tire_d=float(P0["tire_d"]), compression=float(P0.get("compression", 0.0)),
               offset=float(P0["offset"]), tire_w=float(P0.get("tire_w", 200.0)), wheel_d=float(P0.get("wheel_d", 254.0)),
               roll_op=roll_op, heave_op=heave_op, names=names, derived=derived, bj_bounds=bj_bounds,
               bj_free=bool(searched_bj or derived), kpi_h=float(P0["huy"]) + float(P0["hly"]))
    bnds = []
    for k in names:
        if k == "camber":
            lo, hi = map(float, T["camber_range"])
        elif k in BALL_JOINTS:
            lo, hi = bj_bounds[k]
        else:
            lo, hi = map(float, bounds[k])
            if k.endswith("_y"):
                lo, hi = lo + ride, hi + ride           # to height above ground
        bnds.append((lo, hi))
    n = len(names)
    free = [i for i, (lo, hi) in enumerate(bnds) if hi > lo]
    fixed = {i: bnds[i][0] for i in range(n) if i not in free}

    def full(xf):
        x = np.empty(n)
        for j, i in enumerate(free):
            x[i] = xf[j]
        for i, v in fixed.items():
            x[i] = v
        return x

    t0 = time.time()
    if free:
        fb = [bnds[i] for i in free]
        res = differential_evolution(_cost_free, fb, args=(ctx, free, fixed, n), seed=seed,
                                     popsize=10 if quick else 18, maxiter=60 if quick else 160,
                                     tol=1e-7, polish=False, workers=workers,
                                     updating="deferred" if workers != 1 else "immediate")
        loc = minimize(_cost_free, res.x, args=(ctx, free, fixed, n), method="Nelder-Mead", bounds=fb,
                       options=dict(xatol=0.02, fatol=1e-6, maxiter=400))
        xf = loc.x if loc.fun <= res.fun else res.x
        xf = np.clip(xf, [b[0] for b in fb], [b[1] for b in fb])
        xbest = full(xf)
    else:
        xbest = full([])
    # round pivots and searched ball joints to 0.5 mm (staying inside the bounds); camber is picked below
    for i, k in enumerate(names):
        if k != "camber":
            xbest[i] = np.clip(np.round(xbest[i] * 2) / 2, bnds[i][0], bnds[i][1])
    v = dict(zip(names, xbest))
    xA, yA, xB, yB = (v[k] for k in PIVOTS)
    lp, up = (xA, yA), (xB, yB)
    J, terms, m_opt, _ = _cost(xbest, ctx, detail=True)
    searched = {k: v[k] for k in searched_bj}

    # static camber, and the ball joint that holds the KPI target
    if static_camber is not None:
        cam = float(static_camber)
        k = _knuckle_for(ctx, searched, cam)
    elif derived is None:
        k = _knuckle_for(ctx, searched, 0.0)
        cam = pick_static_camber(k, lp, up, ctx["half_track"], ride, ctx["tire_d"], ctx["compression"],
                                 ctx["offset"], C["outside_target"], tuple(C["low_speed"]), tuple(C["high_speed"]))
    else:
        cam, k = _pick_camber_tied(ctx, lp, up, searched)
    if derived:
        key = BJ_KEYS[derived]; lo, hi = bj_bounds[derived]
        if not lo - 1e-6 <= k[key] <= hi + 1e-6 and searched_bj:
            # the joint holding the KPI left its range: fix it at the limit and let the other joint hold the KPI
            other = searched_bj[0]
            base = dict(ctx, knuckle=dict(ctx["knuckle"], **{key: float(np.clip(k[key], lo, hi))}))
            if static_camber is None:
                cam, k = _pick_camber_tied(base, lp, up, {}, derived=other)
            else:
                k = _knuckle_for(base, {}, cam, derived=other)
            derived, key = other, BJ_KEYS[other]; lo, hi = bj_bounds[other]
        k[key] = round(float(k[key]), 2)        # VSusp's 0.01 mm step: KPI within ~0.002 deg
        if not lo - 0.005 <= k[key] <= hi + 0.005:
            progress(f"  note: {derived} {k[key]:.2f} mm is outside its bounds [{lo:g}, {hi:g}]: "
                     f"the KPI target of {kpi:g} deg needs it there")
    P = _build(ctx, k, lp, up, cam)
    nd = 2 if (static_camber is not None or kpi is not None) else 1   # 0.01 mm (VSusp's step) holds camber/KPI to ~0.005 deg
    P["upper_len"] = round(P["upper_len"], nd); P["lower_len"] = round(P["lower_len"], nd)
    progress(f"  optimised in {time.time() - t0:.0f} s, cost {J:.2f}")
    return dict(params=P, cost=J, terms=terms, static_camber=cam,
                before=metrics(P0, roll_op, heave_op), after=metrics(P, roll_op, heave_op),
                pivots=dict(lower_x=xA, lower_y=yA - ride, upper_x=xB, upper_y=yB - ride),
                ball_joints=dict(upper_bj_x=P["hux"], lower_bj_x=P["hlx"]), kpi_joint=derived)


def _cost_free(xf, ctx, free, fixed, n):
    x = np.empty(n)
    for j, i in enumerate(free):
        x[i] = xf[j]
    for i, v in fixed.items():
        x[i] = v
    return _cost(x, ctx)
