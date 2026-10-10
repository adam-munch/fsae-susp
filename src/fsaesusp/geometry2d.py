"""Front-view (2D) double-wishbone kinematics, equivalent to VSusp (vsusp.com, v0.8).

The model reproduces VSusp's geometry exactly (verified against values read from
its own solver), but solves continuously instead of in VSusp's 1 mm steps, so
gradients such as camber gain and roll-centre migration are clean.

Conventions (same as VSusp)
---------------------------
* One corner is modelled: the *right* side in VSusp's drawing, x outboard (+),
  y up, ground at y = 0. Millimetres and degrees throughout.
* Chassis pivots are given relative to the frame-bottom centre, which sits at
  ``bottom_y`` (ride height). Body roll rotates the frame about that point.
* Knuckle points are measured from the **hub face** (``hlx``/``hux`` inboard,
  ``hly`` down / ``huy`` up). The wheel centreline is ``offset`` inboard of the
  hub face (positive offset = hub face outboard of the wheel centreline).
* Tyre contact = tread centre (``tire_d / 2 - compression`` below the wheel
  centre, along the wheel's own vertical axis), held at ``y = bump``.
* Camber: negative = top of the wheel leaning inboard.

Parameter dictionary (``FV``) keys, with the VSusp field each one maps to::

    bottom_y   frame.bottom_y                 ride height of frame bottom
    cx_l, by_l frame.center_to_lower_mount_x / bottom_to_lower_mount_y
    cx_u, by_u frame.center_to_upper_mount_x / bottom_to_upper_mount_y
    lower_len  control_arms.lower_length
    upper_len  control_arms.upper_length
    hux, huy   knuckles.hub_to_upper_x / hub_to_upper_y
    hlx, hly   knuckles.hub_to_lower_x / hub_to_lower_y
    otr_x/otr_y steering.hub_to_outer_tie_rod_x / _y
    offset     wheels.offset
    wheel_d    wheels.diameter_expl (rim diameter, drawing only)
    tire_d     tires.diameter_expl
    tire_w     tires.width_expl (drawing only)
    compression tires.compression
"""
from __future__ import annotations

import numpy as np
from scipy.optimize import brentq

FV_KEYS = ("bottom_y", "cx_l", "by_l", "cx_u", "by_u", "lower_len", "upper_len",
           "hux", "huy", "hlx", "hly", "otr_x", "otr_y", "offset", "wheel_d",
           "tire_d", "tire_w", "compression")


class SolveError(RuntimeError):
    """Raised when no assembly of the linkage satisfies the requested state."""


def rot2(deg: float) -> np.ndarray:
    a = np.radians(deg)
    return np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])


def line_intersect(p1, p2, q1, q2) -> np.ndarray:
    """Intersection of the infinite lines p1-p2 and q1-q2 (2D)."""
    d1, d2 = p2 - p1, q2 - q1
    A = np.array([d1, -d2]).T
    t = np.linalg.solve(A, q1 - p1)
    return p1 + t[0] * d1


def mirror(p: np.ndarray) -> np.ndarray:
    return np.array([-p[0], p[1]])


class Corner2D:
    """One front-view corner built from an ``FV`` parameter dict."""

    def __init__(self, P: dict):
        missing = [k for k in FV_KEYS if k not in P and k not in ("wheel_d", "tire_w", "compression")]
        if missing:
            raise KeyError(f"missing front-view parameters: {missing}")
        self.P = dict(P)
        self.ride = float(P["bottom_y"])
        self.A0 = np.array([P["cx_l"], P["by_l"]], float)
        self.B0 = np.array([P["cx_u"], P["by_u"]], float)
        self.Wl, self.Wu = float(P["lower_len"]), float(P["upper_len"])
        hlx, hly, hux, huy = (float(P[k]) for k in ("hlx", "hly", "hux", "huy"))
        self.ubj_loc = np.array([hlx - hux, hly + huy])          # relative to LBJ, knuckle frame
        self.hub_loc = np.array([hlx, hly])
        self.wc_loc = self.hub_loc + np.array([-float(P["offset"]), 0.0])
        self.RR = float(P["tire_d"]) / 2 - float(P.get("compression", 0.0))
        self.tread_loc = self.wc_loc + np.array([0.0, -self.RR])
        self.otr_loc = np.array([hlx - float(P["otr_x"]), hly - float(P["otr_y"])])
        self.k = float(np.linalg.norm(self.ubj_loc))
        self.ubj_ang0 = np.degrees(np.arctan2(self.ubj_loc[1], self.ubj_loc[0]))

    # ----------------------------------------------------------------- frame
    def frame_pts(self, roll: float = 0.0):
        o = np.array([0.0, self.ride])
        return rot2(roll) @ self.A0 + o, rot2(roll) @ self.B0 + o

    # ------------------------------------------------------------------ pose
    def pose_from_theta(self, theta: float, roll: float = 0.0):
        """Knuckle pose for a lower-arm angle ``theta`` (rad); None if it cannot close."""
        A, B = self.frame_pts(roll)
        L = A + self.Wl * np.array([np.cos(theta), np.sin(theta)])
        e, f = L - B
        p = np.hypot(e, f)
        kk = (p * p + self.Wu ** 2 - self.k ** 2) / (2 * p)
        h2 = self.Wu ** 2 - kk ** 2
        if h2 < 0:
            return None
        h = np.sqrt(h2)
        U = B + np.array([e, f]) * kk / p + np.array([-f, e]) / p * h   # VSusp 'out2' branch
        ang = np.degrees(np.arctan2(U[1] - L[1], U[0] - L[0]))
        delta = ang - self.ubj_ang0
        Rk = rot2(delta)
        return dict(A=A, B=B, L=L, U=U, H=L + Rk @ self.hub_loc, W=L + Rk @ self.wc_loc,
                    T=L + Rk @ self.tread_loc, O=L + Rk @ self.otr_loc,
                    delta=delta, camber=-delta, theta=theta)

    def _g(self, th, roll, bump):
        p = self.pose_from_theta(th, roll)
        return np.nan if p is None else p["T"][1] - bump

    def solve(self, roll: float = 0.0, bump: float = 0.0, theta0: float | None = None):
        """Pose with the tread centre at height ``bump`` (body roll ``roll`` deg).

        ``bump`` > 0 is jounce (the ground under this wheel is raised). If
        ``theta0`` (a previous solution) is given, a narrow bracket around it is
        tried first, which is much faster for sweeps.
        """
        if theta0 is not None:
            a, b = theta0 - 0.2, theta0 + 0.2
            ga, gb = self._g(a, roll, bump), self._g(b, roll, bump)
            if np.isfinite(ga) and np.isfinite(gb) and ga * gb < 0:
                th = brentq(self._g, a, b, args=(roll, bump), xtol=1e-13)
                return self.pose_from_theta(th, roll)
        ths = np.radians(np.linspace(-40, 40, 81))
        vals = np.array([self._g(t, roll, bump) for t in ths])
        ok = np.isfinite(vals)
        idx = np.where(ok[:-1] & ok[1:] & (np.sign(vals[:-1]) * np.sign(vals[1:]) < 0))[0]
        best = None
        for i in idx:
            th = brentq(self._g, ths[i], ths[i + 1], args=(roll, bump), xtol=1e-13)
            if best is None or abs(th) < abs(best):
                best = th
        if best is None:
            raise SolveError(f"front-view linkage cannot reach bump={bump} mm, roll={roll} deg")
        return self.pose_from_theta(best, roll)

    def solve_lbj_y(self, lbj_height: float, roll: float = 0.0):
        """VSusp-style pose with the lower ball joint at a given height (validation)."""
        A, _ = self.frame_pts(roll)
        return self.pose_from_theta(np.arcsin((lbj_height - A[1]) / self.Wl), roll)


def axle_state(c: Corner2D, roll: float = 0.0, bump_r: float = 0.0, bump_l: float = 0.0,
               rack_local=None, theta0=None) -> dict:
    """Both wheels of a symmetric axle (left = mirror of right solved with -roll).

    Positive roll lifts the right (+x) side, so the left wheel is the outside,
    compressed wheel. Returns poses, instant centres, roll centre and the usual
    alignment numbers for the right wheel.
    """
    r = c.solve(roll, bump_r, theta0)
    lm = c.solve(-roll, bump_l, theta0)
    l = {k: (mirror(v) if isinstance(v, np.ndarray) else v) for k, v in lm.items()}
    ic_r = line_intersect(r["B"], r["U"], r["A"], r["L"])
    ic_l = line_intersect(l["B"], l["U"], l["A"], l["L"])
    rc = line_intersect(r["T"], ic_r, l["T"], ic_l)
    out = dict(r=r, l=l, ic_r=ic_r, ic_l=ic_l, rc=rc, track=r["T"][0] - l["T"][0],
               fvsa_r=float(np.linalg.norm(ic_r - r["W"])), cam_r=r["camber"], cam_l=lm["camber"])
    L, U = r["L"], r["U"]
    out["kpi_r"] = float(np.degrees(np.arcsin((L[0] - U[0]) / c.k)))
    xg = L[0] + (U[0] - L[0]) * (bump_r - L[1]) / (U[1] - L[1])
    out["scrub_r"] = float(r["T"][0] - xg)
    xw = L[0] + (U[0] - L[0]) * (r["W"][1] - L[1]) / (U[1] - L[1])
    out["kp_offset_wc_r"] = float(r["W"][0] - xw)
    if rack_local is not None:
        rk = rot2(roll) @ np.asarray(rack_local) + np.array([0, c.ride])
        out["tierod_r"] = float(np.linalg.norm(rk - r["O"]))
        rkl = rot2(roll) @ np.array([-rack_local[0], rack_local[1]]) + np.array([0, c.ride])
        out["tierod_l"] = float(np.linalg.norm(rkl - l["O"]))
    return out


def vsusp_rack_point(c: Corner2D) -> np.ndarray:
    """VSusp's own suggested inner tie-rod point (frame-local): the line from the
    outer tie rod to the instant centre, intersected with the line through the
    inner pivots. Close to, but not exactly, the zero-bump-steer point."""
    s = axle_state(c)
    p = line_intersect(s["r"]["O"], s["ic_r"], s["r"]["B"], s["r"]["A"])
    return p - np.array([0.0, c.ride])


def zero_bump_steer_point(c: Corner2D, dz: float = 0.5):
    """Front-view centre of curvature of the outer tie-rod path at static
    (frame-local). A tie rod from here has zero bump steer to 2nd order in the
    front-view projection. Returns (point, tie-rod length)."""
    s = c.solve(0, 0)
    pts = [c.solve(0, z, s["theta"])["O"] - np.array([0, c.ride]) for z in (-dz, 0.0, dz)]
    p_m, p0, p_p = pts
    d1 = (p_p - p_m) / (2 * dz)
    d2 = (p_p - 2 * p0 + p_m) / dz ** 2
    cross = d1[0] * d2[1] - d1[1] * d2[0]
    ctr = p0 + (d1 @ d1) / cross * np.array([-d1[1], d1[0]])
    return ctr, float(np.linalg.norm(ctr - p0))


def build_from_pose(knuckle: dict, lower_pivot, upper_pivot, half_track: float, camber: float,
                    ride: float, tire_d: float, compression: float = 0.0, offset: float = 25.4,
                    tire_w: float = 200.0, wheel_d: float = 254.0) -> dict:
    """Return an ``FV`` dict whose static solution has the requested pose.

    ``lower_pivot``/``upper_pivot`` are (x from centre, height above ground);
    ``knuckle`` holds hux, huy, hlx, hly, otr_x, otr_y. Arm lengths are derived so
    the tread centre sits at ``half_track`` with ``camber`` (deg).
    """
    P = dict(knuckle)
    P.update(bottom_y=ride, offset=offset, tire_d=tire_d, tire_w=tire_w, wheel_d=wheel_d,
             compression=compression, cx_l=float(lower_pivot[0]), by_l=float(lower_pivot[1]) - ride,
             cx_u=float(upper_pivot[0]), by_u=float(upper_pivot[1]) - ride, lower_len=300.0, upper_len=300.0)
    tmp = Corner2D(P)
    Rk = rot2(-camber)
    L = np.array([half_track, 0.0]) - Rk @ tmp.tread_loc
    U = L + Rk @ tmp.ubj_loc
    P["lower_len"] = float(np.linalg.norm(L - np.asarray(lower_pivot, float)))
    P["upper_len"] = float(np.linalg.norm(U - np.asarray(upper_pivot, float)))
    return P
