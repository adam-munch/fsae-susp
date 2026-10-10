"""Steering checks on a pair of identical 3D front corners (left, and right as its mirror)."""
from __future__ import annotations

import numpy as np
from scipy.optimize import brentq

from .geometry3d import SolveError3D


def ackermann_percent(d_inside, d_outside, track, wheelbase):
    """Ackermann % = actual / ideal (100 %) inside-outside angle difference."""
    d_oi = np.degrees(np.arctan(1 / (1 / np.tan(np.radians(d_inside)) + track / wheelbase)))
    return 100 * (d_inside - d_outside) / (d_inside - d_oi), d_oi


def steer_pair(cL, cR, rack):
    """Rack travel ``rack`` (mm, + toward the car's left). Returns (left state,
    right state, left steer, right steer) with steer + = nose to the left."""
    sl = cL.solve_path(0, rack); sr = cR.solve_path(0, -rack)
    return sl, sr, sl["steer"], -sr["steer"]


def rack_for_left_steer(cL, target, step=2.0, rmax=60.0):
    """Rack travel that puts the left wheel at ``target`` deg (continuation search)."""
    cL.reset()
    for direction in (1.0, -1.0):
        prev_r, prev_v, r = 0.0, cL.solve(0, 0.0)["steer"] - target, 0.0
        while abs(r) < rmax:
            r += direction * step
            try:
                v = cL.solve(0, r)["steer"] - target
            except SolveError3D:
                break
            if prev_v * v <= 0:
                f = lambda rr: cL.solve(0, rr)["steer"] - target
                cL.solve(0, prev_r)
                root = brentq(f, min(prev_r, r), max(prev_r, r), xtol=1e-6)
                cL.reset()
                return root
            prev_r, prev_v = r, v
        cL.reset()
    raise SolveError3D(f"no rack travel reaches {target} deg of steer")


def arm_tie_rod_angle(s):
    """Plan-view angle between steering arm (kingpin -> OTR) and tie rod. Near 0 or
    180 deg means the linkage is close to locking over centre."""
    u, ch = s["pts"], s["chassis"]
    kp = u["LBJ"] + (u["OTR"][2] - u["LBJ"][2]) / (u["UBJ"][2] - u["LBJ"][2]) * (u["UBJ"] - u["LBJ"])
    arm = (u["OTR"] - kp)[:2]; tr = (ch["ITR"] - u["OTR"])[:2]
    return float(np.degrees(np.arccos(np.dot(arm, tr) / np.linalg.norm(arm) / np.linalg.norm(tr))))


def steering_report(cL, cR, track, wheelbase, lock_inside=27.0, ack_at=25.0, clearance=None,
                    heave_at_lock=(-15.0, 0.0, 15.0), n=9):
    rep = {}
    r_ack = rack_for_left_steer(cL, ack_at)
    _, _, di, do = steer_pair(cL, cR, r_ack)
    rep["ackermann_percent"], rep["ideal_outside_at_ack"] = ackermann_percent(di, do, track, wheelbase)
    rep["inside_at_ack"], rep["outside_at_ack"] = di, do
    r_lock = rack_for_left_steer(cL, lock_inside)
    sl, sr, di, do = steer_pair(cL, cR, r_lock)
    rep.update(rack_for_lock=abs(r_lock), inside_lock=di, outside_lock=do,
               camber_inside_lock=sl["camber"], camber_outside_lock=sr["camber"],
               trail_inside_lock=sl["trail"], trail_outside_lock=sr["trail"])
    s0 = cL.reset()
    rep["lift_inside_lock"] = float(sl["pts"]["LBJ"][2] - s0["pts"]["LBJ"][2])
    rep["lift_outside_lock"] = float(sr["pts"]["LBJ"][2] - s0["pts"]["LBJ"][2])
    s1 = cL.solve(0, 1.0); rep["rack_mm_per_deg"] = 1.0 / abs(s1["steer"]); cL.reset()
    rep["arm_tie_rod_angles"] = [arm_tie_rod_angle(cL.solve_path(0, r)) for r in (r_lock, 0.0, -r_lock)]
    cL.reset()
    # curves for plotting
    racks = np.linspace(0, r_lock, n)
    curve = dict(inside=[], outside=[], cam_in=[], cam_out=[], st_in=[], st_out=[])
    for r in racks:
        a, b, di_, do_ = steer_pair(cL, cR, r)
        curve["inside"].append(di_); curve["outside"].append(do_)
        curve["st_in"].append(a["steer"]); curve["cam_in"].append(a["camber"])
        sl2 = cL.solve_path(0, -r); curve["st_out"].append(-sl2["steer"]); curve["cam_out"].append(sl2["camber"])
    cL.reset(); cR.reset()
    rep["curve"] = curve
    if clearance is not None:
        worst, rows = {}, {}
        for z in heave_at_lock:
            rows[z] = []
            for r in np.linspace(-r_lock, r_lock, 2 * n - 1):
                s = cL.solve_path(z, r)
                c = clearance.evaluate(s)
                rows[z].append(dict(steer=s["steer"], **c))
                for k, v in c.items():
                    if k not in worst or v < worst[k][0]:
                        worst[k] = (v, z, s["steer"])
            cL.reset()
        rep["clearance_worst"] = {k: dict(clearance=v[0], heave=v[1], steer=v[2]) for k, v in worst.items()}
        rep["clearance_rows"] = rows
    return rep
