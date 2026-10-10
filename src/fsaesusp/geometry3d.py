"""3D double-wishbone kinematics for one corner (LEFT side of the car).

Coordinates (mm): x forward, y left (= outboard for this corner), z up, ground z = 0.
The upright is a rigid body carrying the upper/lower ball joints (UBJ, LBJ), the
outer tie rod or toe link (OTR), the wheel centre (WC) and the spin axis (A, a
unit vector pointing outboard). Constraints: each ball joint keeps its distance to
the two inner pivots of its arm; the tie rod keeps its length to the inner point
(ITR), which moves with the rack along global y; the contact patch (lowest point
of the loaded wheel circle) sits at a prescribed height. Body roll rotates every
chassis point about the x axis through (y = 0, z = roll_z).
"""
from __future__ import annotations

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

EZ = np.array([0.0, 0.0, 1.0])
CHASSIS = ("UF", "UR", "LF", "LR", "ITR")
UPRIGHT = ("UBJ", "LBJ", "OTR", "WC")


class SolveError3D(RuntimeError):
    pass


def unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def rot_x(deg):
    a = np.radians(deg)
    return np.array([[1, 0, 0], [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]])


class Corner3D:
    """``hp``: dict of 3-vectors UF, UR, LF, LR (arm pivots, F = front leg),
    ITR, UBJ, LBJ, OTR, WC and A (spin axis, outboard). ``R_loaded``: loaded
    radius (mm)."""

    def __init__(self, hp: dict, R_loaded: float, roll_z: float = 60.0):
        self.hp = {k: np.array(v, float) for k, v in hp.items()}
        self.R = float(R_loaded)
        self.roll_z = float(roll_z)
        self.C0 = self.hp["WC"].copy()
        self.a0 = unit(self.hp["A"])
        h = self.hp
        self.L = dict(UF=np.linalg.norm(h["UBJ"] - h["UF"]), UR=np.linalg.norm(h["UBJ"] - h["UR"]),
                      LF=np.linalg.norm(h["LBJ"] - h["LF"]), LR=np.linalg.norm(h["LBJ"] - h["LR"]),
                      TR=np.linalg.norm(h["OTR"] - h["ITR"]))
        self._last = np.zeros(6)

    # ------------------------------------------------------------- geometry
    def chassis(self, roll=0.0, rack=0.0):
        Rx = rot_x(roll); o = np.array([0.0, 0.0, self.roll_z]); out = {}
        for k in CHASSIS:
            p = self.hp[k].copy()
            if k == "ITR":
                p = p + np.array([0.0, rack, 0.0])
            out[k] = Rx @ (p - o) + o
        return out

    def upright(self, q):
        Rm = Rotation.from_rotvec(q[:3]).as_matrix()
        return {k: Rm @ (self.hp[k] - self.C0) + self.C0 + q[3:] for k in UPRIGHT}, Rm @ self.a0

    def contact(self, WC, a):
        d = -EZ - np.dot(-EZ, a) * a
        return WC + self.R * d / np.linalg.norm(d)

    def residual(self, q, roll, rack, z_cp, toe_lock=None):
        c = self.chassis(roll, rack); u, a = self.upright(q); cp = self.contact(u["WC"], a)
        r = [np.linalg.norm(u["UBJ"] - c["UF"]) - self.L["UF"], np.linalg.norm(u["UBJ"] - c["UR"]) - self.L["UR"],
             np.linalg.norm(u["LBJ"] - c["LF"]) - self.L["LF"], np.linalg.norm(u["LBJ"] - c["LR"]) - self.L["LR"]]
        if toe_lock is None:
            r.append(np.linalg.norm(u["OTR"] - c["ITR"]) - self.L["TR"])
        else:                       # hold steer angle instead of the tie rod (path tracing)
            hd = np.cross(a, EZ)
            r.append((np.degrees(np.arctan2(hd[1], hd[0])) - toe_lock) * 10.0)
        r.append(cp[2] - z_cp)
        return np.array(r)

    def solve(self, z=0.0, rack=0.0, roll=0.0, q0=None, toe_lock=None):
        """Pose with the contact patch at height ``z`` (+ = bump, body fixed)."""
        q0 = self._last if q0 is None else q0
        kw = dict(xtol=1e-13, ftol=1e-13, gtol=1e-13, method="lm")
        sol = least_squares(self.residual, q0, args=(roll, rack, z, toe_lock), **kw)
        if np.max(np.abs(sol.fun)) > 1e-6:
            sol = least_squares(self.residual, np.zeros(6), args=(roll, rack, z, toe_lock), **kw)
        if np.max(np.abs(sol.fun)) > 1e-6:
            raise SolveError3D(f"3D linkage cannot reach z={z}, rack={rack}, roll={roll}")
        self._last = sol.x
        return self.state(sol.x, roll, rack)

    def solve_path(self, z=0.0, rack=0.0, roll=0.0, step=4.0):
        """Solve with continuation in rack travel from the centre (robust at full lock)."""
        self.solve(z, 0.0, roll)
        n = max(1, int(np.ceil(abs(rack) / step)))
        s = None
        for r in (np.linspace(0, rack, n + 1)[1:] if rack else [0.0]):
            s = self.solve(z, r, roll)
        return s

    def reset(self):
        self._last = np.zeros(6)
        return self.solve(0.0)

    # --------------------------------------------------------------- output
    def state(self, q, roll=0.0, rack=0.0):
        u, a = self.upright(q); c = self.chassis(roll, rack); cp = self.contact(u["WC"], a)
        h = unit(np.cross(a, EZ)); k = unit(u["UBJ"] - u["LBJ"])
        s = dict(q=q, pts=u, chassis=c, a=a, CP=cp, h=h, k=k)
        s["camber"] = -np.degrees(np.arcsin(np.clip(a[2], -1, 1)))        # top-in negative
        s["steer"] = np.degrees(np.arctan2(h[1], h[0]))                     # nose to +y positive
        s["toe_in"] = -s["steer"]                                           # for this (left) wheel
        s["caster"] = np.degrees(np.arctan2(-k[0], k[2]))
        s["kpi"] = np.degrees(np.arctan2(-k[1], k[2]))
        g = u["LBJ"] + (cp[2] - u["LBJ"][2]) / k[2] * k
        s["trail"] = g[0] - cp[0]
        s["scrub"] = cp[1] - g[1]
        w = u["LBJ"] + (u["WC"][2] - u["LBJ"][2]) / k[2] * k
        s["kp_offset_wc"] = u["WC"][1] - w[1]
        return s


def anti_values(corner: Corner3D, front: bool, wheelbase: float, cg_height: float, front_brake_bias: float,
                inboard_drive: bool = True, h: float = 0.05) -> dict:
    """Anti-dive / anti-lift / anti-squat (%) from virtual displacements.

    Outboard brakes react at the contact patch; inboard drive reacts at the wheel
    centre (RCVD conventions). Front: anti-dive. Rear: anti-squat and anti-lift.
    """
    sp, sm = corner.solve(h), corner.solve(-h); corner.solve(0)
    dcp = (sp["CP"] - sm["CP"]) / (2 * h); dwc = (sp["pts"]["WC"] - sm["pts"]["WC"]) / (2 * h)
    k = wheelbase / cg_height
    if front:
        return {"anti_dive": 100 * front_brake_bias * dcp[0] * k}
    dx = dwc[0] if inboard_drive else dcp[0]
    return {"anti_squat": 100 * (-dx) * k, "anti_lift": 100 * (1 - front_brake_bias) * (-dcp[0]) * k}


def slope_for_anti(percent: float, front: bool, wheelbase: float, cg_height: float, front_brake_bias: float) -> float:
    """Side-view pivot-axis slope dz/dx (x forward) of *parallel* upper and lower
    axes giving the requested anti-dive (front) or anti-squat (rear)."""
    k = wheelbase / cg_height
    if front:
        return -(percent / 100.0) / (front_brake_bias * k)
    return (percent / 100.0) / k
