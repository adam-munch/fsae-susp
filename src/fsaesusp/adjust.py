"""Setup adjustments and rules checks.

Adjustments: how much the alignment and ride height move per unit of the adjusters a team turns at the
track (camber shims at the upper ball joint, tie-rod or toe-link turns, ride height, pushrod length and the
spring perch), with the links otherwise unchanged.

Rules: a few geometric checks against the competition rules. The limits are defaults that can be changed or
switched off (``null``) in the design file's ``rules`` block. Check them against the current rules of your
competition.
"""
from __future__ import annotations

import copy

import numpy as np
from scipy.optimize import brentq

from .analysis2d import metrics
from .geometry2d import Corner2D, SolveError, axle_state
from .geometry3d import SolveError3D

THREAD_PITCH = 25.4 / 24       # 3/8-24 UNF rod ends, common in FSAE
RULE_DEFAULTS = dict(wheelbase_min=1525.0, travel_min=25.0, track_ratio_min=None)


# ----------------------------------------------------------------- adjustments
def camber_shim(P: dict) -> dict:
    """Camber and roll-centre change per mm of shim added between the upright and the upper ball-joint
    mount (the joint moves 1 mm inboard on the upright), arm lengths unchanged."""
    s0 = axle_state(Corner2D(P))
    Q = dict(P); Q["hux"] = P["hux"] + 1.0
    s1 = axle_state(Corner2D(Q))
    return dict(camber=float(s1["cam_r"] - s0["cam_r"]), rc=float(s1["rc"][1] - s0["rc"][1]),
                track=float(s1["track"] - s0["track"]))


def tie_rod_turn(corner) -> float:
    """Static toe-in change (deg) per mm the tie rod or toe link is lengthened."""
    corner.reset(); L0 = corner.L["TR"]
    try:
        corner.L["TR"] = L0 + 0.5; a = corner.solve(0)["toe_in"]
        corner.L["TR"] = L0 - 0.5; b = corner.solve(0)["toe_in"]
    finally:
        corner.L["TR"] = L0; corner.reset()
    return float(a - b)


def ride_height(P: dict, corner, roll_op=1.5, heave_op=10.0) -> dict:
    """Per mm the car is raised (both wheels droop 1 mm relative to the body)."""
    m = metrics(P, roll_op, heave_op)
    h = 0.5
    corner.reset(); a = corner.solve(-h); corner.reset(); b = corner.solve(h); corner.reset()
    return dict(rc=-m["rc_heave_slope"], camber=-m["camber_gain"],
                toe=float((a["toe_in"] - b["toe_in"]) / (2 * h)))


def pushrod_length(layout) -> float | None:
    """Ride-height change (mm, + up) per mm the pushrod or pullrod is lengthened, with the spring load and so
    the damper length unchanged. None if it cannot be solved."""
    L0, D0 = layout.L_rod, layout.L_damper0
    try:
        layout.L_rod = L0 + 1.0
        f = lambda z: layout._len_from_static(z) - D0
        z = brentq(f, -15.0, 15.0, xtol=1e-5)
    except (ValueError, SolveError3D):
        return None
    finally:
        layout.L_rod = L0; layout.c.reset(); layout._phi = 0.0
    return float(-z)


def adjustments(cfg: dict, axle_fv: dict, corners: dict, springs=None) -> dict:
    """All adjustments per axle. ``springs`` is ``api.springs`` output (or None)."""
    adj = cfg.get("adjust") or {}
    pitch = float(adj.get("thread_pitch", THREAD_PITCH))
    turn = pitch * (2.0 if adj.get("turnbuckle") else 1.0)
    cam = cfg["optimise"]["camber"]
    out = dict(thread_pitch=pitch, turn_length=turn, turnbuckle=bool(adj.get("turnbuckle")))
    for axle, P in axle_fv.items():
        r = dict(shim=camber_shim(P))
        r["toe_per_mm"] = tie_rod_turn(corners[axle])
        r["toe_per_turn"] = r["toe_per_mm"] * turn
        r["ride"] = ride_height(P, corners[axle], cam["low_speed"][0], cam["high_speed"][1])
        sp = (springs or {}).get("axles", {}).get(axle)
        if sp:
            r["ride_per_rod_mm"] = pushrod_length(sp["layout"])
            r["ride_per_perch_mm"] = 1.0 / sp["mr"] if sp["mr"] else None
        out[axle] = r
    return out


# ----------------------------------------------------------------- rules
def rules_check(cfg: dict, springs=None) -> list:
    """Rows of (rule, value, limit, status) with status 'pass', 'FAIL' or 'not checked'."""
    from .config import axle_fv
    R = dict(RULE_DEFAULTS); R.update(cfg.get("rules") or {})
    bad = [k for k in R if k not in RULE_DEFAULTS]
    if bad:
        raise ValueError(f"rules: unknown {', '.join(bad)} (use {', '.join(RULE_DEFAULTS)})")
    fv = {a: axle_fv(cfg, a) for a in ("front", "rear")}
    rows = []
    if R.get("wheelbase_min") is not None:
        wb = float(cfg["vehicle"]["wheelbase"])
        rows.append(("Wheelbase (mm)", f"{wb:.0f}", f">= {R['wheelbase_min']:g}",
                     "pass" if wb >= R["wheelbase_min"] else "FAIL"))
    if R.get("track_ratio_min") is not None:
        tf, tr = (axle_state(Corner2D(fv[a]))["track"] for a in ("front", "rear"))
        ratio = min(tf, tr) / max(tf, tr)
        rows.append(("Smaller track / larger track", f"{ratio:.3f}", f">= {R['track_ratio_min']:g}",
                     "pass" if ratio >= R["track_ratio_min"] else "FAIL"))
    t = R.get("travel_min")
    if t is not None:
        t = float(t)
        for axle in ("front", "rear"):
            c = Corner2D(fv[axle])
            reach = []
            for z in (t, -t):
                try:
                    c.solve(0, z); reach.append(True)
                except SolveError:
                    reach.append(False)
            sp = (springs or {}).get("axles", {}).get(axle)
            for i, (lab, key) in enumerate((("jounce", "bump_available"), ("rebound", "droop_available"))):
                name = f"{axle.capitalize()} {lab} travel (mm)"
                if not reach[i]:
                    rows.append((name, "linkage locks", f">= {t:g}", "FAIL"))
                elif sp and "damper_limits" in sp:
                    v = sp[key]
                    shown = f"{v:.1f}" if v is not None else f"> {t:g} (linkage)"
                    rows.append((name, shown + " to damper limit", f">= {t:g}",
                                 "pass" if v is None or v >= t else "FAIL"))
                else:                       # the damper may still stop it first
                    rows.append((name, f"linkage reaches it; no {axle}.spring.damper.length", f">= {t:g}",
                                 "not checked"))
    return rows


def rules_warnings(rows) -> list:
    return [f"rules: {r[0].lower()} {r[1]} (needs {r[2]})" for r in rows if r[3] == "FAIL"]
