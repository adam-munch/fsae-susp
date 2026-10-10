"""Turn a front-view (VSusp-style) corner into 3D hardpoints.

Adds what the front view cannot hold: caster/trail (ball-joint fore/aft
offsets), pivot-axis fore/aft spacing and side-view tilt (anti-dive /
anti-squat), the steering arm / toe-link fore/aft position, and an inner
tie-rod point fitted for zero bump steer in 3D.
"""
from __future__ import annotations

import numpy as np
from scipy.optimize import least_squares

from .geometry2d import Corner2D, axle_state, rot2, zero_bump_steer_point
from .geometry3d import Corner3D, slope_for_anti


def static_2d(P):
    c = Corner2D(P); s = axle_state(c)
    return c, s, s["r"]


def corner_from_2d(P, axle_x=0.0, caster=0.0, kp_offset_x=0.0, lower_pivot_x=(150.0, -150.0),
                   upper_pivot_x=(120.0, -120.0), slope_l=0.0, slope_u=0.0, otr_x=-65.0,
                   otr_yz=None, itr=None, R_loaded=None) -> Corner3D:
    """Map the 2D corner into 3D (left side).

    caster: deg (top rearward +); kp_offset_x: kingpin x-offset from the wheel
    centre at wheel-centre height (+ ahead). lower/upper_pivot_x: pivot x positions
    relative to that arm's ball-joint x (front leg, rear leg). slope_*: side-view
    pivot-axis slope dz/dx through the front-view pivot at the ball joint's x.
    otr_x: outer tie-rod x relative to the axle; otr_yz overrides its front-view
    position. itr: inner tie-rod (x, y, z); default = front-view zero-bump-steer
    point at the OTR's x.
    """
    c, s, r = static_2d(P)
    WC = np.array([axle_x, r["W"][0], r["W"][1]]); zW = r["W"][1]
    t = np.tan(np.radians(caster))
    xU = axle_x + kp_offset_x - (r["U"][1] - zW) * t
    xL = axle_x + kp_offset_x + (zW - r["L"][1]) * t
    UBJ = np.array([xU, r["U"][0], r["U"][1]]); LBJ = np.array([xL, r["L"][0], r["L"][1]])

    def piv(x_bj, yz, dx, slope):
        return np.array([x_bj + dx, yz[0], yz[1] + slope * dx])
    LF, LR = piv(xL, r["A"], lower_pivot_x[0], slope_l), piv(xL, r["A"], lower_pivot_x[1], slope_l)
    UF, UR = piv(xU, r["B"], upper_pivot_x[0], slope_u), piv(xU, r["B"], upper_pivot_x[1], slope_u)
    oyz = r["O"] if otr_yz is None else np.asarray(otr_yz, float)
    OTR = np.array([axle_x + otr_x, oyz[0], oyz[1]])
    if itr is None:
        zb, _ = zero_bump_steer_point(c)
        itr = np.array([axle_x + otr_x, zb[0], zb[1] + P["bottom_y"]])
    cam = s["cam_r"]
    A = np.array([0.0, np.cos(np.radians(cam)), -np.sin(np.radians(cam))])
    RL = (P["tire_d"] / 2 - P.get("compression", 0.0)) if R_loaded is None else R_loaded
    hp = dict(UF=UF, UR=UR, LF=LF, LR=LR, ITR=np.asarray(itr, float), UBJ=UBJ, LBJ=LBJ, OTR=OTR, WC=WC, A=A)
    return Corner3D(hp, RL, roll_z=P["bottom_y"])


def kingpin_point_at(corner: Corner3D, z: float) -> np.ndarray:
    s = corner.solve(0); L, U = s["pts"]["LBJ"], s["pts"]["UBJ"]
    return L + (z - L[2]) / (U[2] - L[2]) * (U - L)


def otr_path(corner: Corner3D, zs) -> np.ndarray:
    """Outer tie-rod positions over travel with the steer angle held at 0."""
    out = []; q = np.zeros(6)
    for z in zs:
        sol = least_squares(corner.residual, q, args=(0.0, 0.0, z, 0.0), xtol=1e-13, ftol=1e-13, method="lm")
        q = sol.x; u, _ = corner.upright(q); out.append(u["OTR"])
    corner._last = np.zeros(6)
    return np.array(out)


def fit_inner_height(make_corner, x_in, y_in, z_guess, zs=(-25.0, -15.0, -5.0, 5.0, 15.0, 25.0)):
    """Height of the inner tie-rod / toe-link point (x, y fixed) that minimises the
    RMS toe change over ``zs`` (mm of wheel travel). Returns (point, RMS toe deg)."""
    from scipy.optimize import minimize_scalar
    from .geometry3d import SolveError3D

    def f(z):
        c = make_corner(np.array([x_in, y_in, z]))
        try:
            return float(sum(c.solve(zz)["toe_in"] ** 2 for zz in zs))
        except SolveError3D:
            return 1e6
    r = minimize_scalar(f, bounds=(z_guess - 25.0, z_guess + 25.0), method="bounded", options=dict(xatol=1e-5))
    return np.array([x_in, y_in, float(r.x)]), float(np.sqrt(r.fun / len(zs)))


def vsusp_tie_rod_params(P: dict, otr_yz) -> tuple:
    """Front-view OTR position -> VSusp knuckle inputs (hub_to_outer_tie_rod_x, _y)."""
    _, _, r = static_2d(P)
    loc = rot2(-r["delta"]) @ (np.asarray(otr_yz, float) - r["L"])
    return float(P["hlx"] - loc[0]), float(P["hly"] - loc[1])


def build_corner(P: dict, cfg: dict, vehicle: dict, front: bool) -> tuple:
    """High-level: build the 3D corner from the design file's ``three_d`` block.

    Returns (Corner3D, info) where info records the slopes, outer/inner tie-rod
    points, link length, fit residual and the matching VSusp tie-rod inputs.
    """
    wb, hcg, bias = vehicle["wheelbase"], vehicle["cg_height"], vehicle["front_brake_bias"]
    axle_x = 0.0 if front else -wb
    if cfg.get("pivot_axis_slope") is not None:
        slope = float(cfg["pivot_axis_slope"])
    else:
        slope = slope_for_anti(float(cfg.get("anti", 0.0)), front, wb, hcg, bias)
    common = dict(axle_x=axle_x, caster=float(cfg.get("caster", 0.0)), kp_offset_x=float(cfg.get("kingpin_offset_x", 0.0)),
                  lower_pivot_x=tuple(cfg.get("lower_pivot_x", (150.0, -150.0))),
                  upper_pivot_x=tuple(cfg.get("upper_pivot_x", (120.0, -120.0))), slope_l=slope, slope_u=slope)
    tr = cfg.get("tie_rod", {}) or {}
    otr_yz = None
    base = corner_from_2d(P, otr_x=-60.0, **common)
    if tr.get("design"):
        d = tr["design"]
        zo = base.hp["WC"][2] - float(d.get("below_hub", 20.0))
        kp = kingpin_point_at(base, zo)
        rear_side = str(d.get("side", "rear")).lower() == "rear"
        arm, ang = float(d.get("arm_length", 50.0)), np.radians(float(d.get("arm_angle", 15.0)))
        dx = -arm if rear_side else arm
        dy = -arm * np.tan(ang) if rear_side else arm * np.tan(ang)
        otr = np.array([kp[0] + dx, kp[1] + dy, zo])
        otr_x_rel = otr[0] - axle_x
        otr_yz = (otr[1], otr[2])
    else:
        otr_x_rel = float(tr.get("x", -65.0 if front else -70.0))
    inner = tr.get("inner", "auto")
    x_in = axle_x + otr_x_rel + float(tr.get("inner_x_offset", 0.0))
    if isinstance(inner, dict) and all(k in inner for k in ("x", "y", "z")):
        itr = np.array([inner["x"], inner["y"], inner["z"]], float); resid = float("nan")
    else:
        # Front-view zero-bump-steer point for this outer point gives the rack-end y
        # (moving along the tie rod barely changes bump steer); the height is then
        # solved in 3D so the toe change over +/-25 mm is minimal.
        P_fv = dict(P)
        if otr_yz is not None:
            P_fv["otr_x"], P_fv["otr_y"] = vsusp_tie_rod_params(P, otr_yz)
        tmp = corner_from_2d(P_fv, otr_x=otr_x_rel, **common)
        y_in = float(inner["y"]) if isinstance(inner, dict) and "y" in inner else float(tmp.hp["ITR"][1])
        make = lambda itr_: corner_from_2d(P, otr_x=otr_x_rel, otr_yz=otr_yz, itr=itr_, **common)
        itr, resid = fit_inner_height(make, x_in, y_in, float(tmp.hp["ITR"][2]))
    corner = corner_from_2d(P, otr_x=otr_x_rel, otr_yz=otr_yz, itr=itr, **common)
    oyz = corner.hp["OTR"][1:]
    info = dict(slope=slope, slope_deg=float(np.degrees(np.arctan(slope))), otr=corner.hp["OTR"].tolist(),
                itr=corner.hp["ITR"].tolist(), link_length=float(corner.L["TR"]), rms_toe_fit=resid,
                vsusp_tie_rod=vsusp_tie_rod_params(P, oyz), axle_x=axle_x)
    return corner, info
