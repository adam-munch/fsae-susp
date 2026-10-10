"""Approximate wheel/tyre envelope clearance for the arms and tie rod.

The wheel is modelled as: an open barrel of inner radius ``lip_radius`` whose
inboard lip sits ``lip_inboard`` inboard of the wheel centreline, and a tyre of
outer radius ``tire_radius`` that extends ``lip_inboard`` inboard as well.
Links are tubes of radius ``link_radius``. Each A-arm is modelled the way a
10-inch FSAE arm is usually built: the two legs meet at a node a little inboard
of the rim lip and one stem runs from the node to the ball joint.

These are packaging *screening* numbers - confirm with the real wheel in CAD.
"""
from __future__ import annotations

import numpy as np

DEFAULT_WHEEL = dict(lip_inboard=107.0, lip_radius=122.0, link_radius=8.0, node_inboard_of_lip=35.0)


def point_clearance(p, WC, a, W, tire_radius):
    v = p - WC; ac = float(np.dot(v, a)); rho = float(np.linalg.norm(v - ac * a))
    f, rl, Rt = W["lip_inboard"], W["lip_radius"], tire_radius
    if ac >= -f:
        if ac > f + 50:
            c = 999.0
        elif rho <= rl:
            c = rl - rho
        elif rho >= Rt:
            c = rho - Rt
        else:
            c = -min(rho - rl, Rt - rho, ac + f)
    else:
        if rho <= rl:
            c = float(np.hypot(ac + f, rl - rho))
        elif rho <= Rt:
            c = -f - ac
        else:
            c = float(np.hypot(ac + f, rho - Rt))
    return c - W["link_radius"]


def segment_clearance(p0, p1, WC, a, W, tire_radius, n=60):
    return min(point_clearance(p0 + t * (p1 - p0), WC, a, W, tire_radius) for t in np.linspace(0, 1, n))


def node_fraction(bj, piv_f, piv_r, WC0, a0, W):
    """Fraction t along ball joint -> pivot-axis foot where the legs meet."""
    axis = (piv_r - piv_f) / np.linalg.norm(piv_r - piv_f)
    foot = piv_f + np.dot(bj - piv_f, axis) * axis
    target = -(W["lip_inboard"] + W["node_inboard_of_lip"])
    t = (target - np.dot(bj - WC0, a0)) / np.dot(foot - bj, a0)
    return float(min(max(t, 0.0), 0.95))


def node_position(bj, piv_f, piv_r, t):
    axis = (piv_r - piv_f) / np.linalg.norm(piv_r - piv_f)
    foot = piv_f + np.dot(bj - piv_f, axis) * axis
    return bj + t * (foot - bj)


class ClearanceModel:
    def __init__(self, corner, wheel: dict | None = None, tire_radius: float = 205.74):
        self.W = dict(DEFAULT_WHEEL); self.W.update(wheel or {})
        self.Rt = tire_radius
        s0 = corner.solve(0); h = corner.hp
        self.t = dict(UCA=node_fraction(h["UBJ"], h["UF"], h["UR"], s0["pts"]["WC"], s0["a"], self.W),
                      LCA=node_fraction(h["LBJ"], h["LF"], h["LR"], s0["pts"]["WC"], s0["a"], self.W))

    def nodes(self, s):
        u, ch = s["pts"], s["chassis"]
        return dict(UCA=node_position(u["UBJ"], ch["UF"], ch["UR"], self.t["UCA"]),
                    LCA=node_position(u["LBJ"], ch["LF"], ch["LR"], self.t["LCA"]))

    def evaluate(self, s) -> dict:
        WC, a = s["pts"]["WC"], s["a"]; u, ch = s["pts"], s["chassis"]; nd = self.nodes(s)
        seg = lambda p, q: segment_clearance(p, q, WC, a, self.W, self.Rt)
        out = {}
        for arm, bj, f, r in (("upper arm", "UBJ", "UF", "UR"), ("lower arm", "LBJ", "LF", "LR")):
            k = "UCA" if arm.startswith("upper") else "LCA"
            out[arm] = min(seg(u[bj], nd[k]), seg(nd[k], ch[f]), seg(nd[k], ch[r]))
        out["tie rod"] = seg(u["OTR"], ch["ITR"])
        return out
