"""Springs: pushrod or pullrod, rocker and damper on a 3D corner, and what follows from them.

Kinematics (one corner, body fixed): the rod's outboard end rides on the lower arm, the upper arm or the
upright; its inboard end is on a rocker that turns about a fixed axis; the damper (with a coaxial spring)
runs from the rocker to the chassis. As the wheel moves, the rocker angle that keeps the rod length is
solved in closed form, which gives the damper length and the motion ratio

    MR = -d(damper length) / d(contact-patch travel)      (damper mm per wheel mm, + = compresses in bump)

From the motion ratio and the rates: wheel rate k_s MR^2, ride rate (in series with the tyre), ride
frequency, roll stiffness (with an anti-roll bar), roll gradient and lateral load-transfer distribution
(geometric, elastic and unsprung parts, using the front-view roll-centre heights), and the travel in
braking and acceleration with the anti geometry.

Points in a ``spring`` block are 3D (x forward from **this axle's** centre line, y left, z up from the
ground at static ride height), for the left corner.
"""
from __future__ import annotations

import numpy as np
from scipy.optimize import brentq

from .geometry3d import Corner3D, SolveError3D

G = 9.81
# vehicle values the spring analysis needs; when the design file leaves them out these are used and listed
VEHICLE_DEFAULTS = dict(mass=280.0, front_weight=0.47, unsprung_mass=12.0, lateral_g=1.5, braking_g=1.5,
                        accel_g=1.0)
TYRE_RATE_DEFAULT = 110.0      # N/mm, typical 16x7.5-10 FSAE tyre near 12 psi: measure or take from tyre data


def _unit(v):
    v = np.asarray(v, float); return v / np.linalg.norm(v)


def _frame(o, a, b):
    """Orthonormal frame (rows) at ``o`` with e1 toward ``a`` and e2 in the plane of ``b``."""
    e1 = _unit(a - o); v = b - o; e2 = _unit(v - (v @ e1) * e1)
    return o, np.array([e1, e2, np.cross(e1, e2)])


def _rotate(v, n, phi):
    return v * np.cos(phi) + np.cross(n, v) * np.sin(phi) + n * (n @ v) * (1 - np.cos(phi))


ARMS = {"lower": (("chassis", "LF"), ("chassis", "LR"), ("pts", "LBJ")),
        "upper": (("chassis", "UF"), ("chassis", "UR"), ("pts", "UBJ")),
        "upright": (("pts", "LBJ"), ("pts", "UBJ"), ("pts", "OTR"))}


def _arm_frame(s, arm):
    (g1, k1), (g2, k2), (g3, k3) = ARMS[arm]
    return _frame(s[g1][k1], s[g2][k2], s[g3][k3])


class SpringLayout:
    """Rod, rocker and damper on ``corner``. ``spec`` is the design file's ``<axle>.spring`` block."""

    def __init__(self, corner: Corner3D, spec: dict, axle_x: float = 0.0):
        self.c = corner; self.spec = spec
        off = np.array([axle_x, 0.0, 0.0])
        pt = lambda key, d=spec: np.asarray(d[key], float) + off
        rod = spec.get("rod") or {}
        self.arm = str(rod.get("arm", "lower")).lower()
        if self.arm not in ARMS:
            raise ValueError(f"spring.rod.arm: use lower, upper or upright (not {self.arm})")
        s0 = corner.reset()
        o, R = _arm_frame(s0, self.arm)
        if "point" in rod:
            P0 = pt("point", rod)
        elif self.arm == "upright":
            raise ValueError("spring.rod: an upright pickup needs 'point: [x, y, z]'")
        else:
            (_, f), (_, r), (_, bj) = ARMS[self.arm]
            F, Rr, BJ = s0["chassis"][f], s0["chassis"][r], s0["pts"][bj]
            e = _unit(Rr - F); foot = F + ((BJ - F) @ e) * e
            P0 = foot + float(rod.get("at", 0.9)) * (BJ - foot) + np.array([0, 0, float(rod.get("above", 0.0))])
        self.P_local = R @ (P0 - o)
        rk = spec.get("rocker") or {}
        dm = spec.get("damper") or {}
        for key, d, where in (("pivot", rk, "rocker"), ("axis", rk, "rocker"), ("rod", rk, "rocker"),
                              ("damper", rk, "rocker"), ("chassis", dm, "damper")):
            if key not in d:
                raise ValueError(f"spring.{where}.{key} is missing")
        self.C = pt("pivot", rk); self.n = _unit(rk["axis"])
        self.Q0 = pt("rod", rk); self.D0 = pt("damper", rk); self.Dc = pt("chassis", dm)
        self.A0 = pt("arb", rk) if "arb" in rk else None
        self.P0 = P0
        self.L_rod = float(np.linalg.norm(self.Q0 - P0))
        self.L_damper0 = float(np.linalg.norm(self.D0 - self.Dc))
        self._phi = 0.0

    # -------------------------------------------------------------- kinematics
    def rod_end(self, s):
        o, R = _arm_frame(s, self.arm)
        return o + R.T @ self.P_local

    def rocker_angle(self, P, phi_prev=0.0):
        v = self.Q0 - self.C; vp = self.n * (self.n @ v); u = v - vp; w = np.cross(self.n, u)
        d = self.C + vp - P
        A, B = 2 * d @ u, 2 * d @ w
        Cc = self.L_rod ** 2 - d @ d - u @ u
        R = float(np.hypot(A, B))
        if R == 0 or abs(Cc) > R:
            raise SolveError3D("the rod cannot reach the rocker at this wheel position")
        base, half = float(np.arctan2(B, A)), float(np.arccos(Cc / R))
        cands = [base + half, base - half]
        wrap = lambda a: (a + np.pi) % (2 * np.pi) - np.pi
        return min(cands, key=lambda a: abs(wrap(a - phi_prev)))

    def state(self, z, phi_prev=None):
        """Rod, rocker and damper at contact-patch travel ``z`` (mm, + bump)."""
        s = self.c.solve(z)
        P = self.rod_end(s)
        phi = self.rocker_angle(P, self._phi if phi_prev is None else phi_prev)
        rot = lambda p: self.C + _rotate(p - self.C, self.n, phi)
        Q, D = rot(self.Q0), rot(self.D0)
        out = dict(z=float(z), s=s, P=P, Q=Q, D=D, phi=float(np.degrees(phi)),
                   damper=float(np.linalg.norm(D - self.Dc)))
        if self.A0 is not None:
            out["A"] = rot(self.A0)
        rod = _unit(Q - P)
        out["rod_dir"] = rod
        # transmission angles: rod and damper against the rocker lever (90 deg is ideal)
        for key, p, line in (("rod_angle", Q, rod), ("damper_angle", D, _unit(D - self.Dc))):
            lever = p - self.C; lever = lever - self.n * (self.n @ lever)
            out[key] = float(np.degrees(np.arccos(np.clip(abs(_unit(lever) @ line), -1, 1))))
        self._phi = phi
        return out

    def sweep(self, zs, h=0.25):
        """Motion ratios and lengths at the travels ``zs``, solved outward from static (bump, then droop)."""
        zs = [float(z) for z in zs]
        rows = {}
        for side in (sorted(z for z in zs if z >= 0), sorted((z for z in zs if z < 0), reverse=True)):
            self.c.reset(); self._phi = 0.0
            for z in side:
                rows[z] = self.point(z, h)
        self.c.reset(); self._phi = 0.0
        return [rows[z] for z in zs]

    def point(self, z, h=0.25):
        a, b = self.state(z + h), self.state(z - h)
        st = self.state(z)
        st["mr"] = -(a["damper"] - b["damper"]) / (2 * h)
        st["rod_mr"] = float(((a["P"] - b["P"]) / (2 * h)) @ st["rod_dir"])
        st["rocker_rate"] = (a["phi"] - b["phi"]) / (2 * h)
        if "A" in a:
            st["arb_disp"] = (a["A"] - b["A"]) / (2 * h)
        return st

    def travel_to_length(self, length, direction):
        """Wheel travel (mm) at which the damper reaches ``length``, searching from static in ``direction``
        (+1 bump, -1 droop). None if the linkage runs out first (beyond 100 mm)."""
        f = lambda z: self._len_from_static(z) - length
        prev = 0.0; fp = f(0.0)
        for z in np.arange(5.0, 105.0, 5.0) * direction:
            try:
                fz = f(z)
            except SolveError3D:
                return None
            if fp * fz <= 0:
                return float(brentq(f, min(prev, z), max(prev, z), xtol=1e-4))
            prev, fp = z, fz
        return None

    def _len_from_static(self, z):
        # continuation from static so the rocker stays on its branch
        self.c.reset(); self._phi = 0.0
        n = max(1, int(np.ceil(abs(z) / 5.0)))
        st = None
        for zz in np.linspace(0, z, n + 1)[1:] if z else [0.0]:
            st = self.state(zz)
        self.c.reset(); self._phi = 0.0
        return st["damper"]


# ------------------------------------------------------------------ rates
def series(*k):
    k = [float(x) for x in k if x is not None]
    return 1.0 / sum(1.0 / x for x in k) if all(x > 0 for x in k) else 0.0


def arb_wheel_rate(arb: dict, mr_link=None) -> float:
    """Anti-roll-bar rate at each wheel in roll (N/mm): ``wheel_rate`` directly, or from the bar's torsional
    ``bar_rate`` (Nm/deg), its ``lever`` (mm) and the motion ratio of the bar end to the wheel
    (``motion_ratio``, or derived from the rocker's ``arb`` point and the drop link)."""
    if not arb:
        return 0.0
    if "wheel_rate" in arb:
        return float(arb["wheel_rate"])
    if "bar_rate" not in arb or "lever" not in arb:
        raise ValueError("spring.arb: give wheel_rate (N/mm), or bar_rate (Nm/deg) and lever (mm)")
    mr = float(arb["motion_ratio"]) if "motion_ratio" in arb else mr_link
    if mr is None:
        raise ValueError("spring.arb: give motion_ratio, or rocker.arb and arb.link so it can be derived")
    K = float(arb["bar_rate"]) * 1000.0 * 180.0 / np.pi          # N mm / rad
    return 2.0 * K * mr ** 2 / float(arb["lever"]) ** 2


def roll_stiffness(k_wheel_roll, track):
    """Nm/deg for a wheel rate in roll (N/mm, each wheel) and a track (mm)."""
    return k_wheel_roll * 1000.0 * (track / 1000.0) ** 2 / 2.0 * np.pi / 180.0


def ride_frequency(k_ride, mass):
    return float(np.sqrt(k_ride * 1000.0 / mass) / (2 * np.pi)) if mass > 0 and k_ride > 0 else float("nan")


def vehicle_values(vehicle: dict) -> tuple:
    """(values, names taken from VEHICLE_DEFAULTS). ``unsprung_mass`` may be one number or [front, rear]."""
    out, assumed = {}, []
    for k, v in VEHICLE_DEFAULTS.items():
        if vehicle.get(k) is None:
            out[k] = v; assumed.append(k)
        else:
            out[k] = vehicle[k]
    um = out["unsprung_mass"]
    out["unsprung_mass"] = [float(um), float(um)] if np.ndim(um) == 0 else [float(um[0]), float(um[1])]
    for k in ("mass", "front_weight", "lateral_g", "braking_g", "accel_g"):
        out[k] = float(out[k])
    return out, assumed


# ------------------------------------------------------------------ analysis
def corner_springs(corner: Corner3D, spec: dict, axle_x: float, tyre: dict, travel=(-30.0, 30.0),
                   wheel=None) -> dict:
    """Motion ratio, damper and rod numbers and the rates of one corner. With ``wheel`` (the clearance model's
    wheel dict), also the rod's clearance to the other links and the rim over +/-25 mm."""
    lay = SpringLayout(corner, spec, axle_x)
    if "spring_rate" not in spec:
        raise ValueError("spring.spring_rate (N/mm) is missing")
    zs = np.arange(travel[0], travel[1] + 1e-9, 2.5)
    rows = []
    for z, r in zip(zs, _safe_sweep(lay, zs)):
        if r is not None:
            rows.append(r)
    st = {round(r["z"], 6): r for r in rows}
    if 0.0 not in st:
        raise SolveError3D("spring: the rod and rocker do not assemble at static")
    r0 = st[0.0]
    out = dict(curve=dict(z=[r["z"] for r in rows], mr=[r["mr"] for r in rows], damper=[r["damper"] for r in rows],
                          rocker=[r["phi"] for r in rows], rod_mr=[r["rod_mr"] for r in rows]),
               arm=lay.arm, rod_length=lay.L_rod, damper_static=r0["damper"], mr=r0["mr"], rod_mr=r0["rod_mr"],
               rod_angle=r0["rod_angle"], damper_angle=r0["damper_angle"])
    inner = [r for r in rows if -25.0 - 1e-9 <= r["z"] <= 25.0 + 1e-9]
    out["min_rod_angle"] = min(r["rod_angle"] for r in inner)
    out["min_damper_angle"] = min(r["damper_angle"] for r in inner)
    if wheel is not None:
        from .clearance import ClearanceModel
        clr = ClearanceModel(corner, wheel, float(tyre["diameter"]) / 2)
        gaps = rod_clearances([r for r in inner if abs(r["z"] / 5.0 - round(r["z"] / 5.0)) < 1e-9], lay.arm, clr)
        out["rod_gap_links"], out["rod_gap_rim"] = gaps["links"], gaps["rim"]
        corner.reset()
    if 25.0 in st and -25.0 in st:
        out["mr_bump25"], out["mr_droop25"] = st[25.0]["mr"], st[-25.0]["mr"]
        out["progression"] = 100.0 * (st[25.0]["mr"] - st[-25.0]["mr"]) / r0["mr"] if r0["mr"] else float("nan")
        out["damper_travel25"] = st[-25.0]["damper"] - st[25.0]["damper"]
        out["rocker_range25"] = (st[-25.0]["phi"], st[25.0]["phi"])
    out["rod_force_per_kn"] = 1000.0 / r0["rod_mr"] if r0["rod_mr"] else float("inf")
    lim = (spec.get("damper") or {}).get("length")
    if lim:
        lo, hi = sorted(float(v) for v in lim)
        out["damper_limits"] = (lo, hi)
        out["bump_available"] = lay.travel_to_length(lo, +1) if r0["damper"] > lo else 0.0
        out["droop_available"] = lay.travel_to_length(hi, -1) if r0["damper"] < hi else 0.0
        if out["droop_available"] is not None:
            out["droop_available"] = -out["droop_available"]
    # rates
    k_s = float(spec["spring_rate"])
    arb = spec.get("arb") or {}
    mr_link = None
    if arb.get("link") is not None and "arb_disp" in r0:
        link = np.asarray(arb["link"], float) + np.array([axle_x, 0.0, 0.0])
        mr_link = abs(float(r0["arb_disp"] @ _unit(link - lay.A0)))
        out["arb_mr"] = mr_link
    k_arb = arb_wheel_rate(arb, mr_link)
    k_t = float(tyre.get("rate") or TYRE_RATE_DEFAULT)
    k_w = k_s * r0["mr"] ** 2
    out.update(spring_rate=k_s, tyre_rate=k_t, wheel_rate=k_w, arb_rate=k_arb, ride_rate=series(k_w, k_t),
               roll_wheel_rate=series(k_w + k_arb, k_t) if k_w + k_arb > 0 else 0.0)
    out["wheel_rate_curve"] = [k_s * m ** 2 for m in out["curve"]["mr"]]
    out["layout"] = lay
    corner.reset()
    return out


def _safe_sweep(lay, zs):
    """``lay.sweep`` that stops at the travel where the linkage stops assembling (None beyond it)."""
    zs = list(zs); res = {}
    for side in (sorted(z for z in zs if z >= 0), sorted((z for z in zs if z < 0), reverse=True)):
        lay.c.reset(); lay._phi = 0.0
        for z in side:
            try:
                res[z] = lay.point(z)
            except SolveError3D:
                break
    lay.c.reset(); lay._phi = 0.0
    return [res.get(z) for z in zs]


def roll_balance(Kf, Kr, v, cg_height, rc, track, radius) -> dict:
    """Roll gradient and lateral load transfer from the axles' roll stiffnesses ``Kf``, ``Kr`` (Nm/deg).
    ``v``: ``vehicle_values`` output; ``rc``, ``track``, ``radius``: per axle (mm)."""
    m, fw = v["mass"], v["front_weight"]
    mu = dict(front=v["unsprung_mass"][0], rear=v["unsprung_mass"][1])
    ms = m - 2 * mu["front"] - 2 * mu["rear"]
    fs = (m * fw - 2 * mu["front"]) / ms                        # sprung mass on the front axle
    h_s = (m * cg_height - 2 * mu["front"] * radius["front"] - 2 * mu["rear"] * radius["rear"]) / ms
    h_ra = rc["front"] + (rc["rear"] - rc["front"]) * (1 - fs)  # roll axis under the sprung CG
    K = Kf + Kr
    out = dict(sprung_mass=ms, sprung_front=fs, sprung_cg_height=h_s, roll_axis_height=h_ra, roll_stiffness=K,
               roll_stiffness_front_share=Kf / K if K else float("nan"))
    out["roll_gradient"] = ms * G * (h_s - h_ra) / 1000.0 / K if K else float("inf")
    lt = {}
    for axle, Ka, sh in (("front", Kf, fs), ("rear", Kr, 1 - fs)):
        t = track[axle] / 1000.0
        lt[axle] = dict(elastic=ms * G * (h_s - h_ra) / 1000.0 * (Ka / K if K else 0) / t,
                        geometric=ms * sh * G * rc[axle] / 1000.0 / t,
                        unsprung=2 * mu[axle] * G * radius[axle] / 1000.0 / t)
        lt[axle]["total"] = sum(lt[axle].values())
    out["load_transfer"] = lt                                   # N per g of lateral acceleration, per axle
    tot = lt["front"]["total"] + lt["rear"]["total"]
    out["lltd_front"] = lt["front"]["total"] / tot if tot else float("nan")
    return out


def vehicle_springs(axles: dict, vehicle: dict, rc: dict, track: dict, radius: dict, anti: dict, corners: dict,
                    camber_cases=None) -> dict:
    """Ride, roll and pitch of the car from both corners' spring results ``axles`` (see ``corner_springs``),
    the front-view roll-centre heights ``rc``, tracks, loaded radii (unsprung CG heights), the anti
    percentages and the 3D corners (for camber and toe at the pitch travels)."""
    v, assumed = vehicle_values(vehicle)
    m, fw, L = v["mass"], v["front_weight"], float(vehicle["wheelbase"])
    mu = dict(front=v["unsprung_mass"][0], rear=v["unsprung_mass"][1])
    out = dict(assumed=assumed, values=v, axles={})
    for axle in ("front", "rear"):
        a = axles[axle]; share = fw if axle == "front" else 1 - fw
        sprung = m * share / 2 - mu[axle]
        a["sprung_corner_mass"] = sprung
        a["ride_frequency"] = ride_frequency(a["ride_rate"], sprung)
        a["roll_stiffness"] = float(roll_stiffness(a["roll_wheel_rate"], track[axle]))
        a["roll_stiffness_springs"] = float(roll_stiffness(series(a["wheel_rate"], a["tyre_rate"]), track[axle]))
    out.update(roll_balance(axles["front"]["roll_stiffness"], axles["rear"]["roll_stiffness"], v,
                            float(vehicle["cg_height"]), rc, track, radius))
    out["roll_at_lateral_g"] = out["roll_gradient"] * v["lateral_g"]
    if camber_cases:
        out["optimiser_roll"] = float(camber_cases["low_speed"][0])
    # pitch: longitudinal transfer, minus what the anti geometry carries, through the ride rates
    h = float(vehicle["cg_height"])
    dW_b = m * G * v["braking_g"] * h / L / 2                    # N per wheel
    dW_a = m * G * v["accel_g"] * h / L / 2
    kf, kr = axles["front"]["ride_rate"], axles["rear"]["ride_rate"]
    ad, al, asq = (anti["front"].get("anti_dive", 0.0), anti["rear"].get("anti_lift", 0.0),
                   anti["rear"].get("anti_squat", 0.0))
    brk = dict(front=dW_b * (1 - ad / 100) / kf, rear=-dW_b * (1 - al / 100) / kr)
    acc = dict(front=-dW_a / kf, rear=dW_a * (1 - asq / 100) / kr)
    out["pitch"] = {}
    for case, z in (("braking", brk), ("accel", acc)):
        row = dict(travel=z, pitch=float(np.degrees((z["front"] - z["rear"]) / L)))
        for axle in ("front", "rear"):
            try:
                corners[axle].reset(); s = corners[axle].solve(z[axle])
                row[f"camber_{axle}"], row[f"toe_{axle}"] = float(s["camber"]), float(s["toe_in"])
            except SolveError3D:
                row[f"camber_{axle}"] = row[f"toe_{axle}"] = float("nan")
            corners[axle].reset()
        out["pitch"][case] = row
    return out


# ------------------------------------------------------------------ rod clearance
def seg_dist(p1, q1, p2, q2):
    """Shortest distance between segments p1-q1 and p2-q2 (arrays of shape (..., 3))."""
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e = np.sum(d1 * d1, -1), np.sum(d2 * d2, -1)
    b, c, f = np.sum(d1 * d2, -1), np.sum(d1 * r, -1), np.sum(d2 * r, -1)
    den = a * e - b * b
    s = np.where(den > 1e-12 * a * e, np.clip((b * f - c * e) / np.where(den > 0, den, 1.0), 0.0, 1.0), 0.0)
    t = (b * s + f) / e
    s = np.where(t < 0, np.clip(-c / a, 0.0, 1.0), np.where(t > 1, np.clip((b - c) / a, 0.0, 1.0), s))
    t = np.clip(t, 0.0, 1.0)
    return np.linalg.norm(p1 + d1 * s[..., None] - p2 - d2 * t[..., None], axis=-1)


def other_links(s, arm, nodes):
    """Segments the rod must clear: the legs of the arm it is not on (and, for an upright pickup, both arms'
    legs from the node in) and the tie rod / toe link. ``nodes``: ``ClearanceModel.nodes(s)``."""
    u, ch = s["pts"], s["chassis"]; out = []
    for name, bj, f, r, nd in (("upper arm", "UBJ", "UF", "UR", "UCA"), ("lower arm", "LBJ", "LF", "LR", "LCA")):
        if name.startswith(arm):
            continue
        if arm != "upright":
            out.append((name, u[bj], nodes[nd]))
        out += [(name, nodes[nd], ch[f]), (name, nodes[nd], ch[r])]
    out.append(("tie rod", u["OTR"], ch["ITR"]))
    return out


def rod_clearances(rows, arm, clr) -> dict:
    """Smallest surface gap (mm) from the rod to the other links and to the rim/tyre over ``rows`` (states from
    ``SpringLayout``); both links are tubes of the wheel model's ``link_radius``."""
    from .clearance import segment_clearance
    links, rim = float("inf"), float("inf")
    rl = clr.W["link_radius"]
    for r in rows:
        s = r["s"]
        for _, a, b in other_links(s, arm, clr.nodes(s)):
            links = min(links, float(seg_dist(r["P"], r["Q"], a, b)) - 2 * rl)
        rim = min(rim, segment_clearance(r["P"], r["Q"], s["pts"]["WC"], s["a"], clr.W, clr.Rt, n=40))
    return dict(links=links, rim=rim)
