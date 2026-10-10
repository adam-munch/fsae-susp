"""High-level functions behind the command-line tool (also usable from Python)."""
from __future__ import annotations

import copy
import csv
from pathlib import Path

import numpy as np

from . import config as cfgmod
from .analysis2d import static_summary, metrics
from .optimize import optimise_axle
from .layout3d import build_corner
from .geometry3d import anti_values
from .clearance import ClearanceModel
from .steering import steering_report
from .geometry2d import Corner2D, axle_state

AXLES = ("front", "rear")


# ----------------------------------------------------------------- 2D
def analyze(cfg: dict) -> dict:
    cam = cfg["optimise"]["camber"]
    out = {"name": cfg.get("name", ""), "vsusp_link": cfgmod.to_vsusp_link(cfg)}
    for axle in AXLES:
        P = cfgmod.axle_fv(cfg, axle)
        s = static_summary(P)
        m = metrics(P, cam["low_speed"][0], cam["high_speed"][1])
        c = Corner2D(P)
        ls = axle_state(c, cam["low_speed"][0], cam["low_speed"][1], cam["low_speed"][1])
        hs = axle_state(c, cam["high_speed"][0], cam["high_speed"][1], cam["high_speed"][1])
        m["outside_camber_low_speed"] = float(ls["cam_l"]); m["outside_camber_high_speed"] = float(hs["cam_l"])
        out[axle] = dict(static=s, metrics=m, fv=P)
    return out


def optimize(cfg: dict, axles=AXLES, quick=False, seed=0, workers=1, progress=print) -> tuple:
    """Returns (new design dict, per-axle optimiser results)."""
    new = copy.deepcopy(cfg); results = {}
    cam = cfg["optimise"]["camber"]
    for axle in axles:
        P0 = cfgmod.axle_fv(cfg, axle)
        o = cfg["optimise"].get(axle, {}) or {}
        bounds = _default_bounds(P0); bounds.update(o.get("bounds", {}) or {})
        targets = {"rc_height": axle_state(Corner2D(P0))["rc"][1]}
        targets.update(o.get("targets", {}) or {})
        progress(f"Optimising {axle} pivots (bounds {bounds}) ...")
        r = optimise_axle(P0, bounds, targets, cam, roll_op=cam["low_speed"][0], heave_op=cam["high_speed"][1],
                          seed=seed, quick=quick, workers=workers, progress=progress)
        cfgmod.set_axle_fv(new, axle, r["params"])
        results[axle] = r
    return new, results


def _default_bounds(P):
    """+/-30 mm around the current pivots (VSusp coordinates)."""
    return dict(lower_x=[P["cx_l"] - 30, P["cx_l"] + 30], lower_y=[max(P["by_l"] - 30, -P["bottom_y"] + 5), P["by_l"] + 30],
                upper_x=[P["cx_u"] - 30, P["cx_u"] + 30], upper_y=[P["by_u"] - 30, P["by_u"] + 30])


# ----------------------------------------------------------------- 3D
def build_3d(cfg: dict, axle: str):
    P = cfgmod.axle_fv(cfg, axle)
    return build_corner(P, cfg[axle]["three_d"], cfg["vehicle"], front=(axle == "front"))


def check3d(cfg: dict, progress=print, steering=True) -> dict:
    veh = cfg["vehicle"]; res = {"wheelbase": veh["wheelbase"]}
    corners = {}
    for axle in AXLES:
        progress(f"3D: building {axle} corner ...")
        c, info = build_3d(cfg, axle)
        corners[axle] = c
        s0 = c.reset()
        r = dict(info=info, static={k: float(s0[k]) for k in ("camber", "toe_in", "caster", "kpi", "trail", "scrub", "kp_offset_wc")})
        r["anti"] = {k: float(v) for k, v in anti_values(c, axle == "front", veh["wheelbase"], veh["cg_height"],
                                                          veh["front_brake_bias"], veh.get("inboard_drive", True)).items()}
        zs = np.linspace(-30, 30, 13)
        bump = dict(z=[], toe_in=[], camber=[], caster=[], trail=[], cp_x=[])
        for z in zs:
            s = c.solve(z)
            bump["z"].append(float(z)); bump["toe_in"].append(float(s["toe_in"])); bump["camber"].append(float(s["camber"]))
            bump["caster"].append(float(s["caster"])); bump["trail"].append(float(s["trail"]))
            bump["cp_x"].append(float(s["CP"][0] - s0["CP"][0]))
        c.reset()
        r["bump"] = bump
        r["roll_steer"] = {float(p): float(c.solve(0, roll=p)["toe_in"]) for p in (-2.0, -1.0, 1.0, 2.0)}
        c.reset()
        tyre, wheel = cfgmod.axle_tyre_wheel(cfg, axle)
        clr = ClearanceModel(c, wheel, tyre["diameter"] / 2)
        r["clearance_model"] = clr
        if axle == "rear" or not steering:
            worst = {}
            for z in np.arange(-30, 31, 5.0):
                cc = clr.evaluate(c.solve(z))
                for k, v in cc.items():
                    if k not in worst or v < worst[k]["clearance"]:
                        worst[k] = dict(clearance=float(v), heave=float(z), steer=0.0)
            c.reset()
            r["clearance_worst"] = worst
        res[axle] = r
    sF = static_summary(cfgmod.axle_fv(cfg, "front"))
    res["track_front"] = sF["track"]
    if steering:
        progress("3D: steering sweep (Ackermann, lock, clearance) - this takes a minute ...")
        cR_mirror, _ = build_3d(cfg, "front")
        st_cfg = cfg["front"]["three_d"].get("steering", {}) or {}
        rep = steering_report(corners["front"], cR_mirror, sF["track"], veh["wheelbase"],
                              lock_inside=float(st_cfg.get("lock_inside", 27.0)),
                              ack_at=float(st_cfg.get("ackermann_at", 25.0)),
                              clearance=res["front"]["clearance_model"])
        res["steering"] = rep
        res["front"]["clearance_worst"] = rep["clearance_worst"]
        res["front"]["lock_states"] = [(corners["front"].solve_path(0, rep["rack_for_lock"]), (0, (4, 3))),
                                       (corners["front"].solve_path(0, -rep["rack_for_lock"]), (0, (1, 2)))]
        # sign: the left wheel is the inside wheel when it steers to the left (+)
        if res["front"]["lock_states"][0][0]["steer"] < 0:
            res["front"]["lock_states"].reverse()
        corners["front"].reset()
    res["corners"] = corners
    return res


HP_LABELS = [("UF", "upper arm front pivot"), ("UR", "upper arm rear pivot"), ("LF", "lower arm front pivot"),
             ("LR", "lower arm rear pivot"), ("UBJ", "upper ball joint"), ("LBJ", "lower ball joint"),
             ("OTR", "outer tie rod / toe link"), ("ITR", "inner tie rod / toe link"), ("WC", "wheel centre")]


def write_hardpoints_csv(res3d: dict, path):
    rows = [["axle", "point", "x_mm (forward, front axle = 0)", "y_mm (left of centreline)", "z_mm (above ground)", "note"]]
    for axle in AXLES:
        c = res3d["corners"][axle]; s = c.reset(); h = c.hp; clr = res3d[axle]["clearance_model"]; nd = clr.nodes(s)
        for key, lab in HP_LABELS:
            note = "chassis" if key in ("UF", "UR", "LF", "LR", "ITR") else "upright"
            rows.append([axle, lab, f"{h[key][0]:.1f}", f"{h[key][1]:.1f}", f"{h[key][2]:.1f}", note])
        for arm, lab in (("UCA", "upper arm leg node"), ("LCA", "lower arm leg node")):
            p = nd[arm]; rows.append([axle, lab, f"{p[0]:.1f}", f"{p[1]:.1f}", f"{p[2]:.1f}", "construction: legs meet here"])
        cp = s["CP"]; rows.append([axle, "contact patch centre", f"{cp[0]:.1f}", f"{cp[1]:.1f}", f"{cp[2]:.1f}", ""])
    with open(path, "w", newline="", encoding="utf-8") as f:
        csv.writer(f).writerows(rows)
