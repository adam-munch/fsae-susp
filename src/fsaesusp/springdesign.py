"""Generate and optimise a pushrod/pullrod, rocker and damper layout, then pick the spring and anti-roll-bar
rates.

Layout variables (per axle):

* ``at``: rod pickup, the fraction of the way from the arm's pivot axis to its ball joint;
* ``pivot_x``, ``pivot_y``, ``pivot_z``: the rocker pivot (x from this axle, y left, z up);
* ``rod_lever``, ``damper_lever``: the rocker's lever lengths;
* ``rod_lever_angle``, ``damper_lever_angle``: how far each lever is turned from square to its line at static
  (0 = the rod or damper is at 90 deg to its lever);
* ``damper_tilt``: turns the damper about the rod line, which tilts the rocker plane;
* ``damper_swing``: turns the damper within the rocker plane, away from ``damper_dir``.

Construction: the rocker plane holds the rod pickup, the pivot and the damper line, so the rocker axis is
square to both. The rod meets its lever where the rod is tangent to the lever circle (then turned by
``rod_lever_angle``). The damper lever is square to the damper line, on the side that compresses the damper in
bump (then turned by ``damper_lever_angle``). The damper's static length is placed in its stroke so that the
wheel has the same margin past the bump and droop travel targets before the damper reaches either stop.

The wheel's path does not depend on the rocker, so it is solved once. Each candidate layout then only needs the
closed-form rocker angle at every wheel position, which makes the search fast.
"""
from __future__ import annotations

import copy
import time

import numpy as np
from scipy.optimize import brentq, differential_evolution, minimize

from .clearance import ClearanceModel, segment_clearance
from .geometry3d import SolveError3D
from .springs import (TYRE_RATE_DEFAULT, _arm_frame, _unit, other_links, ride_frequency, roll_balance,
                      roll_stiffness, seg_dist, series, vehicle_values)

VARS = ("at", "pivot_x", "pivot_y", "pivot_z", "rod_lever", "damper_lever", "rod_lever_angle",
        "damper_lever_angle", "damper_tilt", "damper_swing")
# what ``lock`` can hold at the current spring block's value
LOCKABLE = ("at", "pivot_x", "pivot_y", "pivot_z", "rod_lever", "damper_lever")
DEFAULT_BOUNDS = dict(at=[0.8, 0.95], rod_lever=[30.0, 80.0], damper_lever=[30.0, 90.0],
                      rod_lever_angle=[-20.0, 20.0], damper_lever_angle=[-20.0, 20.0], damper_tilt=[-15.0, 15.0],
                      damper_swing=[-20.0, 20.0])
DEFAULT_TARGETS = dict(
    mr=None,                    # damper/wheel at static; default from the stroke and travel (see design_targets)
    progression=[0.0, 15.0],    # %, MR change from -25 to +25 mm relative to static
    travel=[25.0, 25.0],        # mm, bump and droop the damper must allow
    travel_margin=3.0,          # mm, extra past each before the damper's stops
    min_angle=60.0,             # deg, rod and damper to their levers over +/-25 mm (90 is square)
    max_rotation=30.0,          # deg, rocker rotation either way over +/-25 mm
    rod_clearance=5.0,          # mm, rod surface to the other links (tubes of wheel.link_radius)
    rim_clearance=3.0,          # mm, rod to the rim and tyre
    rod_force_max=2.0,          # rod force per unit vertical wheel force
    ride_frequency=None,        # Hz; picks the spring rate (else the spring block's spring_rate is kept)
)
LB_IN = 0.175126835             # N/mm per lb/in
BAD = 1e6


def _rot(v, n, deg):
    a = np.radians(deg)
    return v * np.cos(a) + np.cross(n, v) * np.sin(a) + n * (v @ n) * (1 - np.cos(a))


def _rot_many(V, n, phi):
    """Rodrigues for many vectors ``V`` (N, 3), one angle each (rad)."""
    c, s = np.cos(phi)[:, None], np.sin(phi)[:, None]
    return V * c + np.cross(n, V) * s + n[None, :] * (V @ n)[:, None] * (1 - c)


# ----------------------------------------------------------------- the wheel's path
class WheelPath:
    """The corner solved over travel once: arm frames, the links the rod must clear and the wheel."""

    def __init__(self, corner, arm, wheel, tyre_radius, zmax=60.0, step=1.0):
        if arm not in ("lower", "upper"):
            raise ValueError("spring_design: the rod goes on the lower or upper arm")
        self.arm = arm
        clr = ClearanceModel(corner, wheel, tyre_radius); self.clr = clr
        res = {}
        for side in (np.arange(0.0, zmax + 1e-9, step), np.arange(-step, -zmax - 1e-9, -step)):
            corner.reset()
            for z in side:
                try:
                    res[float(z)] = corner.solve(float(z))
                except SolveError3D:
                    break
        corner.reset()
        self.z = np.array(sorted(res))
        st = [res[z] for z in self.z]
        self.i0 = int(np.argmin(np.abs(self.z)))
        self.s0 = st[self.i0]
        fr = [_arm_frame(s, arm) for s in st]
        self.o = np.array([f[0] for f in fr]); self.R = np.array([f[1] for f in fr])
        segs = [other_links(s, arm, clr.nodes(s)) for s in st]
        self.links = [(np.array([x[k][1] for x in segs]), np.array([x[k][2] for x in segs])) for k in range(len(segs[0]))]
        self.WC = np.array([s["pts"]["WC"] for s in st]); self.a = np.array([s["a"] for s in st])
        self.link_radius = float(clr.W["link_radius"])

    def pickup(self, at, above):
        """Static pickup point and its path for ``at`` along the arm, ``above`` mm up."""
        from .springs import ARMS
        (_, f), (_, r), (_, bj) = ARMS[self.arm]
        F, Rr, BJ = self.s0["chassis"][f], self.s0["chassis"][r], self.s0["pts"][bj]
        e = _unit(Rr - F); foot = F + ((BJ - F) @ e) * e
        P0 = foot + at * (BJ - foot) + np.array([0.0, 0.0, above])
        local = self.R[self.i0] @ (P0 - self.o[self.i0])
        return P0, self.o + np.einsum("nji,j->ni", self.R, local)

    def index(self, z):
        return int(np.argmin(np.abs(self.z - z)))


# ----------------------------------------------------------------- one layout
def build(v: dict, ctx: dict) -> dict | None:
    """Static geometry for the variables ``v`` (see VARS), or None if it cannot be built."""
    path = ctx["path"]
    P0, P = path.pickup(v["at"], ctx["above"])
    C = np.array([ctx["axle_x"] + v["pivot_x"], v["pivot_y"], v["pivot_z"]])
    p = P0 - C; d = float(np.linalg.norm(p))
    rq, rd = v["rod_lever"], v["damper_lever"]
    if d < 1.5 * rq:
        return None
    rhat = -p / d
    u0 = _rot(ctx["damper_dir"], rhat, v["damper_tilt"])
    n = np.cross(rhat, u0)
    if np.linalg.norm(n) < 0.2:                                # damper nearly along the rod
        return None
    n = n / np.linalg.norm(n)
    ud = _unit(_rot(_unit(u0 - n * (u0 @ n)), n, v["damper_swing"]))
    ea = p / d; eb = np.cross(n, ea); ca = rq / d
    Q0 = C + _rot(rq * (ca * ea + np.sqrt(1 - ca * ca) * eb), n, v["rod_lever_angle"])
    rho = _unit(Q0 - P0)
    i0 = path.i0
    vP = (P[i0 + 1] - P[i0 - 1]) / (path.z[i0 + 1] - path.z[i0 - 1])
    t = np.cross(n, Q0 - C) @ rho
    if abs(t) < 0.05 * rq:
        return None
    omega = (vP @ rho) / t                                     # rocker rate in bump
    D0 = C - np.sign(omega) * rd * np.cross(n, ud)             # the side that compresses the damper
    D0 = C + _rot(D0 - C, n, v["damper_lever_angle"])
    return dict(P0=P0, P=P, C=C, n=n, Q0=Q0, D0=D0, ud=ud, L_rod=float(np.linalg.norm(Q0 - P0)))


def _rocker(g, path):
    """Rocker angle (rad) at every wheel position, NaN where the rod cannot reach."""
    C, n, Q0, P = g["C"], g["n"], g["Q0"], g["P"]
    u = Q0 - C; w = np.cross(n, u)
    dd = C[None, :] - P
    A, B = 2 * dd @ u, 2 * dd @ w
    Cc = g["L_rod"] ** 2 - np.sum(dd * dd, 1) - u @ u
    R = np.hypot(A, B)
    ok = (R > 0) & (np.abs(Cc) <= R)
    base = np.arctan2(B, A); half = np.arccos(np.clip(Cc / np.where(R > 0, R, 1), -1, 1))
    i0 = path.i0
    sgn = 1.0 if abs((base[i0] + half[i0] + np.pi) % (2 * np.pi) - np.pi) < \
        abs((base[i0] - half[i0] + np.pi) % (2 * np.pi) - np.pi) else -1.0
    phi = (base + sgn * half + np.pi) % (2 * np.pi) - np.pi
    phi[~ok] = np.nan
    # keep the stretch around static where the rocker assembles
    bad = ~np.isfinite(phi)
    lo = i0
    while lo > 0 and not bad[lo - 1]:
        lo -= 1
    hi = i0
    while hi < len(phi) - 1 and not bad[hi + 1]:
        hi += 1
    return phi, lo, hi


def evaluate(g: dict, ctx: dict, detail=False) -> dict | None:
    """Damper length and everything the cost needs; places the damper's static length in its stroke."""
    path, T = ctx["path"], ctx["targets"]
    phi, lo, hi = _rocker(g, path)
    z = path.z[lo:hi + 1]; ph = phi[lo:hi + 1]; i0 = path.i0 - lo
    B, Dt = T["travel"]
    if z[0] > -Dt or z[-1] < B or not np.isfinite(ph).all():
        return None
    C, n = g["C"], g["n"]
    Q = C + _rot_many(np.repeat((g["Q0"] - C)[None, :], len(ph), 0), n, ph)
    D = C + _rot_many(np.repeat((g["D0"] - C)[None, :], len(ph), 0), n, ph)
    Lmin, Lmax = ctx["damper"]; stroke = Lmax - Lmin
    L_static = 0.5 * (Lmin + Lmax)
    for _ in range(2):                                         # the damper's angle depends a little on its length
        Dc = g["D0"] + L_static * g["ud"]
        L = np.linalg.norm(D - Dc, axis=1)
        dL = L - L[i0]
        win = (z >= -(Dt + 30.0)) & (z <= B + 30.0)
        if not np.all(np.diff(dL[win]) < 0):                   # must shorten steadily in bump
            return None
        f = lambda m: np.interp(-(Dt + m), z, dL) - np.interp(B + m, z, dL) - stroke
        m_lo, m_hi = -min(B, Dt) + 0.5, min(z[-1] - B, -z[0] - Dt, 30.0)
        if f(m_hi) < 0:
            m = m_hi                                           # the linkage, not the damper, ends the travel
        elif f(m_lo) > 0:
            m = m_lo
        else:
            m = brentq(f, m_lo, m_hi, xtol=1e-3)
        L_static = Lmin - float(np.interp(B + m, z, dL))
    Dc = g["D0"] + L_static * g["ud"]
    L = np.linalg.norm(D - Dc, axis=1)
    mr = -np.gradient(L, z)
    j = (z >= -25.0 - 1e-9) & (z <= 25.0 + 1e-9)
    P = g["P"][lo:hi + 1]
    rod = (Q - P) / np.linalg.norm(Q - P, axis=1)[:, None]
    lev_q = (Q - C) / np.linalg.norm(Q - C, axis=1)[:, None]
    lev_d = (D - C) / np.linalg.norm(D - C, axis=1)[:, None]
    dmp = (Dc - D) / np.linalg.norm(Dc - D, axis=1)[:, None]
    ang_r = np.degrees(np.arccos(np.clip(np.abs(np.sum(lev_q * rod, 1)), 0, 1)))
    ang_d = np.degrees(np.arccos(np.clip(np.abs(np.sum(lev_d * dmp, 1)), 0, 1)))
    vP = (P[i0 + 1] - P[i0 - 1]) / (z[i0 + 1] - z[i0 - 1])
    rod_mr = float(vP @ rod[i0])
    out = dict(mr=float(mr[i0]), mr_bump25=float(np.interp(25.0, z, mr)), mr_droop25=float(np.interp(-25.0, z, mr)),
               margin=float(m), L_static=float(L_static), Dc=Dc, rod_mr=rod_mr,
               rod_force=1.0 / rod_mr if rod_mr > 0 else np.inf,
               min_rod_angle=float(ang_r[j].min()), min_damper_angle=float(ang_d[j].min()),
               max_rotation=float(np.degrees(np.abs(ph[j]).max())))
    out["progression"] = 100.0 * (out["mr_bump25"] - out["mr_droop25"]) / out["mr"] if out["mr"] > 0 else -BAD
    # clearance every 5 mm over the travel targets
    k = np.where((z >= -Dt - 1e-9) & (z <= B + 1e-9) & (np.abs(z / 5.0 - np.round(z / 5.0)) < 1e-9))[0]
    gap = np.inf
    for a, b in path.links:
        gap = min(gap, float(np.min(seg_dist(P[k], Q[k], a[lo:hi + 1][k], b[lo:hi + 1][k]))) - 2 * path.link_radius)
    out["rod_gap"] = gap
    rim = np.inf
    for kk in (k[0], np.argmin(np.abs(z)), k[-1]):
        ii = kk + lo
        rim = min(rim, segment_clearance(P[kk], Q[kk], path.WC[ii], path.a[ii], path.clr.W, path.clr.Rt, n=30))
    out["rim_gap"] = rim
    if ctx.get("damper_box"):
        box = ctx["damper_box"]; c = Dc - np.array([ctx["axle_x"], 0, 0]); over = 0.0
        for i, ax in enumerate("xyz"):
            if ax in box:
                lo_, hi_ = box[ax]; over += max(0.0, lo_ - c[i]) + max(0.0, c[i] - hi_)
        out["damper_box_miss"] = over
    if detail:
        out.update(z=z, mr_curve=mr, L=L, phi=np.degrees(ph))
    return out


def cost(v: dict, ctx: dict, detail=False):
    g = build(v, ctx)
    e = evaluate(g, ctx) if g is not None else None
    if e is None:
        return (BAD, {}, None, None) if detail else BAD
    T = ctx["targets"]
    lo, hi = T["progression"]
    terms = dict(
        mr=((e["mr"] - T["mr"]) / 0.01) ** 2,
        progression=(max(0, lo - e["progression"]) / 2) ** 2 + (max(0, e["progression"] - hi) / 2) ** 2,
        travel=10 * max(0, T["travel_margin"] - e["margin"]) ** 2,
        angles=(max(0, T["min_angle"] - e["min_rod_angle"]) / 3) ** 2 + (max(0, T["min_angle"] - e["min_damper_angle"]) / 3) ** 2,
        rotation=(max(0, e["max_rotation"] - T["max_rotation"]) / 2) ** 2,
        rod_clearance=10 * max(0, T["rod_clearance"] - e["rod_gap"]) ** 2,
        rim_clearance=10 * max(0, T["rim_clearance"] - e["rim_gap"]) ** 2,
        rod_force=(max(0, e["rod_force"] - T["rod_force_max"]) / 0.1) ** 2,
        damper_box=(e.get("damper_box_miss", 0.0) / 2) ** 2,
        # small preferences that break ties: a stiffer rod path and less rocker rotation
        prefer=0.02 * e["rod_force"] ** 2 + 1e-4 * e["max_rotation"] ** 2,
    )
    J = float(sum(terms.values()))
    return (J, terms, g, e) if detail else J


def _cost_vec(xf, ctx, free, fixed):
    v = dict(fixed); v.update(zip(free, xf))
    return cost(v, ctx)


# ----------------------------------------------------------------- set-up
def design_targets(sd_axle: dict, damper) -> dict:
    T = dict(DEFAULT_TARGETS); T.update(sd_axle.get("targets") or {})
    T["travel"] = [float(x) for x in T["travel"]]
    if T["mr"] is None:                                        # use about 95 % of the stroke over the travel
        T["mr"] = round(0.95 * (damper[1] - damper[0]) / (sum(T["travel"]) + 2 * T["travel_margin"]), 3)
    return T


def current_values(spec: dict) -> dict:
    """The lockable variables of an existing ``spring`` block."""
    out = {}
    rod = spec.get("rod") or {}
    if "at" in rod:
        out["at"] = float(rod["at"])
    rk = spec.get("rocker") or {}
    if "pivot" in rk:
        C = np.asarray(rk["pivot"], float)
        out.update(pivot_x=C[0], pivot_y=C[1], pivot_z=C[2])
        n = _unit(rk.get("axis", [1, 0, 0]))
        for key, name in (("rod", "rod_lever"), ("damper", "damper_lever")):
            if key in rk:
                d = np.asarray(rk[key], float) - C
                out[name] = float(np.linalg.norm(d - n * (d @ n)))
    return out


def setup(cfg: dict, axle: str, corner, lock_extra=(), unlock=()) -> dict:
    """Everything the search needs for one axle: the context, bounds and locks."""
    sd = (cfg.get("spring_design") or {}).get(axle)
    if not sd:
        raise ValueError(f"spring_design.{axle} is missing (see examples/fsae_aero_v2.yaml)")
    if "damper" not in sd:
        raise ValueError(f"spring_design.{axle}.damper: give [compressed, extended] eye-to-eye lengths (mm)")
    damper = sorted(float(x) for x in sd["damper"])
    from .config import axle_tyre_wheel
    tyre, wheel = axle_tyre_wheel(cfg, axle)
    arm = str(sd.get("arm", "lower")).lower()
    path = WheelPath(corner, arm, wheel, float(tyre["diameter"]) / 2)
    axle_x = 0.0 if axle == "front" else -float(cfg["vehicle"]["wheelbase"])
    spec = cfg[axle].get("spring") or {}
    cur = current_values(spec)
    bounds = dict(DEFAULT_BOUNDS)
    for k in ("pivot_x", "pivot_y", "pivot_z"):
        if k in cur:
            bounds[k] = [cur[k] - 25.0, cur[k] + 25.0]
    bounds.update({k: [float(b[0]), float(b[1])] for k, b in (sd.get("bounds") or {}).items()})
    bad = [k for k in bounds if k not in VARS]
    if bad:
        raise ValueError(f"spring_design.{axle}.bounds: unknown {', '.join(bad)} (use {', '.join(VARS)})")
    missing = [k for k in VARS if k not in bounds]
    if missing:
        raise ValueError(f"spring_design.{axle}.bounds: give {', '.join(missing)} "
                         f"(or add a {axle}.spring block to start from)")
    lock = sd.get("lock") or []
    lock = ([lock] if isinstance(lock, str) else list(lock)) + list(lock_extra)
    lock = [k for k in LOCKABLE if k in lock and k not in unlock]
    unknown = [k for k in (sd.get("lock") or []) if k not in LOCKABLE]
    if unknown:
        raise ValueError(f"spring_design.{axle}.lock: unknown {', '.join(unknown)} (use {', '.join(LOCKABLE)})")
    for k in lock:
        if k not in cur:
            raise ValueError(f"spring_design.{axle}.lock: {k} needs a value in {axle}.spring to keep")
        bounds[k] = [cur[k], cur[k]]
    T = design_targets(sd, damper)
    ctx = dict(path=path, targets=T, damper=damper, axle_x=axle_x, above=float(sd.get("above", 10.0)),
               damper_dir=_unit(sd.get("damper_dir", [0.0, -1.0, 0.0])), damper_box=sd.get("damper_box"))
    return dict(ctx=ctx, bounds=bounds, locked=lock, current=cur, sd=sd)


def construct(st: dict) -> dict:
    """The generated starting layout: every variable at the middle of its range, the levers square, and the
    damper lever sized for the target motion ratio (MR = rod motion ratio x damper lever / rod lever)."""
    b, ctx = st["bounds"], st["ctx"]
    v = {k: 0.5 * (lo + hi) for k, (lo, hi) in b.items()}
    for k in ("rod_lever_angle", "damper_lever_angle", "damper_tilt", "damper_swing"):
        v[k] = float(np.clip(0.0, *b[k]))
    g = build(v, ctx)
    e = evaluate(g, ctx) if g is not None else None
    if e is not None and e["mr"] > 0:
        # with square levers MR is proportional to damper lever / rod lever
        want = v["damper_lever"] * ctx["targets"]["mr"] / e["mr"]
        v["damper_lever"] = float(np.clip(want, *b["damper_lever"]))
        if v["damper_lever"] != want:
            v["rod_lever"] = float(np.clip(v["rod_lever"] * v["damper_lever"] / want, *b["rod_lever"]))
    return v


def optimise_axle(st: dict, quick=False, seed=0, workers=1, construct_only=False, progress=print) -> dict:
    ctx, b = st["ctx"], st["bounds"]
    v0 = construct(st)
    J0, terms0, g0, e0 = cost(v0, ctx, detail=True)
    free = [k for k in VARS if b[k][1] > b[k][0]]
    fixed = {k: b[k][0] for k in VARS if k not in free}
    t0 = time.time()
    v = dict(v0)
    if free and not construct_only:
        fb = [tuple(b[k]) for k in free]
        x0 = np.clip([v0[k] for k in free], [lo for lo, _ in fb], [hi for _, hi in fb])
        res = differential_evolution(_cost_vec, fb, args=(ctx, free, fixed), seed=seed, x0=x0,
                                     popsize=8 if quick else 15, maxiter=40 if quick else 150, tol=1e-8,
                                     polish=False, workers=workers,
                                     updating="deferred" if workers != 1 else "immediate")
        loc = minimize(_cost_vec, res.x, args=(ctx, free, fixed), method="Nelder-Mead", bounds=fb,
                       options=dict(xatol=1e-3, fatol=1e-7, maxiter=1500))
        xf = loc.x if loc.fun <= res.fun else res.x
        v = dict(fixed); v.update(zip(free, np.clip(xf, [lo for lo, _ in fb], [hi for _, hi in fb])))
    J, terms, g, e = cost(v, ctx, detail=True)
    if not construct_only:
        progress(f"  searched in {time.time() - t0:.0f} s, cost {J:.3g} (generated layout {J0:.3g})")
    return dict(values=v, cost=J, terms=terms, geometry=g, result=e, start=dict(values=v0, cost=J0, result=e0),
                free=free, locked=st["locked"])


def spring_block(r: dict, st: dict, spring_rate: float, arb=None) -> dict:
    """The design file's ``spring`` block for an optimised layout (points rounded to 0.1 mm)."""
    g, e, ctx = r["geometry"], r["result"], st["ctx"]
    off = np.array([ctx["axle_x"], 0.0, 0.0])
    rnd = lambda p: [round(float(x), 1) for x in (np.asarray(p) - off)]
    out = dict(rod=dict(arm=ctx["path"].arm, at=round(float(r["values"]["at"]), 4), above=ctx["above"]),
               rocker=dict(pivot=rnd(g["C"]), axis=[round(float(x), 5) for x in g["n"]], rod=rnd(g["Q0"]),
                           damper=rnd(g["D0"])),
               damper=dict(chassis=rnd(e["Dc"]), length=[float(x) for x in ctx["damper"]]),
               spring_rate=round(float(spring_rate), 2))
    if arb:
        out["arb"] = arb
    return out


# ----------------------------------------------------------------- rates
def spring_for_frequency(freq, sprung_mass, mr, tyre_rate, step_lb=25.0, rates=None):
    """Spring rate (N/mm) for a ride frequency: ride rate m (2 pi f)^2, the tyre taken out of series, divided by
    MR^2, then rounded to the nearest ``rates`` (N/mm) or multiple of ``step_lb`` lb/in. Returns (exact, chosen)."""
    k_ride = sprung_mass * (2 * np.pi * freq) ** 2 / 1000.0
    if k_ride >= tyre_rate:
        raise ValueError(f"a {freq:g} Hz ride frequency needs a ride rate above the tyre rate ({tyre_rate:g} N/mm)")
    k_w = k_ride * tyre_rate / (tyre_rate - k_ride)
    exact = k_w / mr ** 2
    if rates:
        chosen = min((float(r) for r in rates), key=lambda r: abs(r - exact))
    else:
        chosen = round(exact / LB_IN / step_lb) * step_lb * LB_IN
    return float(exact), float(chosen)


def arb_for_target(target: dict, axle: str, wheel_rates: dict, tyre_rates: dict, v: dict, cg_height, rc, track,
                   radius, arb_other=0.0) -> tuple:
    """Anti-roll-bar wheel rate (N/mm) on ``axle`` for ``target`` ({lltd_front: f} or {roll_gradient: deg/g}).
    Returns (rate, achieved, note)."""
    other = "rear" if axle == "front" else "front"

    def K(k_arb):
        Ks = {axle: roll_stiffness(series(wheel_rates[axle] + k_arb, tyre_rates[axle]), track[axle]),
              other: roll_stiffness(series(wheel_rates[other] + arb_other, tyre_rates[other]), track[other])}
        return roll_balance(Ks["front"], Ks["rear"], v, cg_height, rc, track, radius)

    key = "lltd_front" if "lltd_front" in target else "roll_gradient" if "roll_gradient" in target else None
    if key is None:
        raise ValueError("spring_design.arb: give lltd_front (0-1) or roll_gradient (deg/g)")
    want = float(target[key])
    f = lambda k: K(k)[key] - want
    lo, hi = 0.0, 400.0
    if f(lo) * f(hi) > 0:
        best = min((lo, hi), key=lambda k: abs(f(k)))
        return best, K(best)[key], f"{key} {want:g} is out of reach with a bar on the {axle} axle"
    k = brentq(f, lo, hi, xtol=1e-4)
    return float(k), K(k)[key], ""
