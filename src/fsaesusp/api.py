"""High-level functions behind the command-line tool (also usable from Python)."""
from __future__ import annotations

import copy
import csv
from pathlib import Path

import numpy as np

from . import config as cfgmod
from .analysis2d import static_summary, metrics
from .optimize import optimise_axle, LOCKABLE, BALL_JOINTS, knuckle_roles
from .steeropt import (STEERING_VARS, wants_steering, steering_values, optimise_steering,
                       default_bounds as steering_default_bounds, resolve_targets as resolve_steering_targets,
                       check_free as check_steering_free)
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


def optimize(cfg: dict, axles=AXLES, quick=False, seed=0, workers=1, progress=print, steering_only=False) -> tuple:
    """Returns (new design dict, per-axle optimiser results).

    Names in ``optimise.<axle>.lock`` keep the design's current value (see ``optimize.LOCKABLE``).
    With an ``ackermann``, ``rack_per_deg`` or ``steering_ratio`` target for the front, the outer tie rod and
    the rack line are placed after the front pivots (``results["front"]["steering"]``). ``steering_only``
    skips the pivots: only that steering stage runs.
    """
    new = copy.deepcopy(cfg); results = {}
    cam = cfg["optimise"]["camber"]
    setups = {}
    pivot_axles = () if steering_only else tuple(axles)
    for axle in pivot_axles:            # check every axle's settings before the first (long) search
        st = optimise_setup(cfg, axle)
        vals, locked = st["values"], st["locked"]
        bounds = dict(st["bounds"]); bounds.update({k: [vals[k]] * 2 for k in locked if k in bounds})
        try:
            knuckle_roles({k: bounds[k] for k in BALL_JOINTS}, st["targets"].get("kpi"))
        except ValueError as e:
            raise ValueError(f"optimise.{axle}: {e}") from None
        setups[axle] = (st, bounds)
    steer = None
    if steering_only or "front" in axles:
        st = optimise_setup(cfg, "front")
        if wants_steering(st["targets"]):
            sb = {k: st["bounds"][k] for k in STEERING_VARS}
            sb.update({k: [st["values"][k]] * 2 for k in st["locked"] if k in sb})
            try:
                resolve_steering_targets(cfg, st["targets"]); check_steering_free(sb)
            except ValueError as e:
                raise ValueError(f"optimise.front: {e}") from None
            steer = (st, sb)
        elif steering_only:
            raise ValueError("steering only: give optimise.front.targets an ackermann, rack_per_deg or "
                             "steering_ratio target")
    for axle in pivot_axles:
        st, bounds = setups[axle]
        P0, vals, locked, targets = st["P0"], st["values"], st["locked"], st["targets"]
        bounds = {k: b for k, b in bounds.items() if k not in STEERING_VARS}
        shown = {k: [round(v, 2) for v in b] for k, b in bounds.items()
                 if k not in locked and not (k in BALL_JOINTS and b[0] == b[1])}
        held = ", ".join(f"{k} {vals[k]:.2f}" for k in locked if k not in STEERING_VARS)
        what = "pivots and ball joints" if any(k in shown for k in BALL_JOINTS) else "pivots"
        kpi = targets.get("kpi")
        progress(f"Optimising {axle} {what} ("
                 + "; ".join(([f"bounds {shown}"] if shown else []) + ([f"locked {held}"] if held else [])
                             + ([f"KPI held at {float(kpi):g} deg"] if kpi is not None else [])) + ") ...")
        r = optimise_axle(P0, bounds, targets, cam, roll_op=cam["low_speed"][0], heave_op=cam["high_speed"][1],
                          seed=seed, quick=quick, workers=workers,
                          static_camber=vals["camber"] if "camber" in locked else None, progress=progress)
        r["locked"] = [k for k in locked if k not in STEERING_VARS]
        cfgmod.set_axle_fv(new, axle, r["params"])
        results[axle] = r
    if steer:
        st, sb = steer
        free = {k: [round(v, 2) for v in b] for k, b in sb.items() if b[1] > b[0]}
        T = {k: st["targets"][k] for k in ("ackermann", "rack_per_deg", "steering_ratio") if st["targets"].get(k) is not None}
        progress(f"Placing the front steering (targets {T}; bounds {free}) ...")
        r = optimise_steering(new, sb, st["targets"], quick=quick, progress=progress)
        r["locked"] = [k for k in st["locked"] if k in STEERING_VARS]
        new = r["cfg"]
        results.setdefault("front", {})["steering"] = r
    return new, results


def lock_list(cfg: dict, axle: str) -> list:
    """``optimise.<axle>.lock``, checked, in ``LOCKABLE`` order."""
    lock = (cfg["optimise"].get(axle) or {}).get("lock") or []
    lock = [lock] if isinstance(lock, str) else list(lock)
    bad = [str(k) for k in lock if k not in LOCKABLE]
    if bad:
        raise ValueError(f"optimise.{axle}.lock: unknown {', '.join(bad)} (use {', '.join(LOCKABLE)})")
    return [k for k in LOCKABLE if k in lock]


def optimise_setup(cfg: dict, axle: str) -> dict:
    """What the optimiser starts from on one axle: the front-view dict ``P0``, the current ``values`` of the
    lockable parameters (pivots in VSusp coordinates, ball joints in mm inboard of the hub face, static camber
    in deg; for the front also the steering variables in ``steeropt.STEERING_VARS``), the ``locked`` names, the
    search ``bounds`` from the design file (or the defaults: pivots +/-30 mm, ball joints fixed, outer tie rod
    +/-15 mm across and fore/aft) before the locks are applied, and the ``targets`` (RC height defaults to the
    current one)."""
    P0 = cfgmod.axle_fv(cfg, axle)
    s0 = axle_state(Corner2D(P0))
    values = dict(lower_x=P0["cx_l"], lower_y=P0["by_l"], upper_x=P0["cx_u"], upper_y=P0["by_u"],
                  upper_bj_x=P0["hux"], lower_bj_x=P0["hlx"], camber=float(s0["cam_r"]))
    o = cfg["optimise"].get(axle) or {}
    bounds = _default_bounds(P0)
    if axle == "front":                 # steering: outer tie rod and rack line (used with a steering target)
        values.update(steering_values(cfg)); bounds.update(steering_default_bounds(values))
    bounds.update({k: v for k, v in (o.get("bounds") or {}).items() if v is not None})
    targets = {"rc_height": float(s0["rc"][1])}
    targets.update(o.get("targets") or {})
    return dict(P0=P0, values=values, locked=lock_list(cfg, axle), bounds=bounds, targets=targets)


def _default_bounds(P):
    """+/-30 mm around the current pivots (VSusp coordinates); the ball joints stay where they are."""
    return dict(lower_x=[P["cx_l"] - 30, P["cx_l"] + 30], lower_y=[max(P["by_l"] - 30, -P["bottom_y"] + 5), P["by_l"] + 30],
                upper_x=[P["cx_u"] - 30, P["cx_u"] + 30], upper_y=[P["by_u"] - 30, P["by_u"] + 30],
                upper_bj_x=[P["hux"], P["hux"]], lower_bj_x=[P["hlx"], P["hlx"]])


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
        if st_cfg.get("c_factor"):          # mm of rack travel per steering-wheel turn
            rep["c_factor"] = float(st_cfg["c_factor"])
            rep["steering_ratio"] = rep["rack_mm_per_deg"] * 360.0 / rep["c_factor"]
        res["steering"] = rep
        res["front"]["clearance_worst"] = rep["clearance_worst"]
        res["front"]["lock_states"] = [(corners["front"].solve_path(0, rep["rack_for_lock"]), (0, (4, 3))),
                                       (corners["front"].solve_path(0, -rep["rack_for_lock"]), (0, (1, 2)))]
        # sign: the left wheel is the inside wheel when it steers to the left (+)
        if res["front"]["lock_states"][0][0]["steer"] < 0:
            res["front"]["lock_states"].reverse()
        corners["front"].reset()
    res["corners"] = corners
    from .adjust import adjustments, rules_check
    res["springs"] = springs(cfg, corners, anti={a: res[a]["anti"] for a in AXLES}, progress=progress)
    res["adjust"] = adjustments(cfg, {a: cfgmod.axle_fv(cfg, a) for a in AXLES}, corners, res["springs"])
    res["rules"] = rules_check(cfg, res["springs"])
    return res


def springs(cfg: dict, corners=None, anti=None, progress=print):
    """Pushrod/rocker/damper kinematics and the rates of each axle with a ``spring`` block, and, when both
    axles have one, ride, roll, load transfer and pitch for the car. None when no axle has a spring block.
    ``corners``/``anti`` from ``check3d`` are reused when given."""
    from .springs import corner_springs, vehicle_springs
    specs = {ax: cfg[ax].get("spring") for ax in AXLES if cfg[ax].get("spring")}
    if not specs:
        return None
    veh = cfg["vehicle"]; corners = dict(corners or {}); anti = dict(anti or {})
    out = {"axles": {}}
    for axle in AXLES:
        if axle not in corners:
            corners[axle], _ = build_3d(cfg, axle)
        if axle not in anti:
            anti[axle] = anti_values(corners[axle], axle == "front", veh["wheelbase"], veh["cg_height"],
                                     veh["front_brake_bias"], veh.get("inboard_drive", True))
            corners[axle].reset()
    for axle, spec in specs.items():
        progress(f"Springs: {axle} rod, rocker and damper ...")
        tyre, wheel = cfgmod.axle_tyre_wheel(cfg, axle)
        out["axles"][axle] = corner_springs(corners[axle], spec, 0.0 if axle == "front" else -veh["wheelbase"], tyre,
                                            wheel=wheel)
        out["axles"][axle]["tyre_rate_assumed"] = not tyre.get("rate")
    if len(specs) == 2:
        rc, track, radius = {}, {}, {}
        for axle in AXLES:
            P = cfgmod.axle_fv(cfg, axle); s = static_summary(P)
            rc[axle], track[axle] = s["rc_height"], s["track"]
            radius[axle] = P["tire_d"] / 2 - P.get("compression", 0.0)
        out["vehicle"] = vehicle_springs(out["axles"], veh, rc, track, radius, anti, corners,
                                         cfg["optimise"].get("camber"))
    return out


def design_springs(cfg: dict, axles=AXLES, quick=False, seed=0, workers=1, construct_only=False, lock=None,
                   unlock=None, progress=print) -> tuple:
    """Generate and optimise the rod, rocker and damper of each axle from ``spring_design``, pick the spring
    rates for the ride-frequency targets and size the anti-roll bar. Returns (new design, results); the
    results hold each axle's search, the rates and ``springs``, the full springs analysis of the new design."""
    from . import springdesign as sdm
    from .springs import arb_wheel_rate, vehicle_values
    sd = cfg.get("spring_design") or {}
    veh = cfg["vehicle"]; v, assumed = vehicle_values(veh)
    new = copy.deepcopy(cfg); results = {"assumed": assumed}
    for axle in axles:
        c, _ = build_3d(cfg, axle)
        st = sdm.setup(cfg, axle, c, (lock or {}).get(axle, ()), (unlock or {}).get(axle, ()))
        T = st["ctx"]["targets"]
        progress(f"Spring layout, {axle}: target motion ratio {T['mr']:g}, travel {T['travel'][0]:g} / "
                 f"{T['travel'][1]:g} mm + {T['travel_margin']:g} mm"
                 + (f"; locked {', '.join(st['locked'])}" if st["locked"] else "") + " ...")
        r = sdm.optimise_axle(st, quick=quick, seed=seed, workers=workers, construct_only=construct_only,
                              progress=progress)
        if r["result"] is None:
            raise ValueError(f"spring_design.{axle}: no layout in the bounds assembles - widen the bounds or move "
                             "the rocker box")
        tyre, _ = cfgmod.axle_tyre_wheel(cfg, axle)
        k_t = float(tyre.get("rate") or sdm.TYRE_RATE_DEFAULT)
        old = cfg[axle].get("spring") or {}
        share = v["front_weight"] if axle == "front" else 1 - v["front_weight"]
        m_s = v["mass"] * share / 2 - v["unsprung_mass"][0 if axle == "front" else 1]
        if T.get("ride_frequency"):
            exact, rate = sdm.spring_for_frequency(float(T["ride_frequency"]), m_s, r["result"]["mr"], k_t,
                                                   float(sd.get("spring_step", 25.0)), sd.get("spring_rates"))
        elif "spring_rate" in old:
            exact = rate = float(old["spring_rate"])
        else:
            raise ValueError(f"spring_design.{axle}.targets.ride_frequency (Hz) is needed to pick a spring rate")
        r.update(setup=st, spring_rate=rate, spring_rate_exact=exact, sprung_corner_mass=m_s, tyre_rate=k_t)
        new[axle]["spring"] = sdm.spring_block(r, st, rate, arb=old.get("arb"))
        results[axle] = r
    arb_t = sd.get("arb")
    if arb_t and all(new[a].get("spring") for a in AXLES):
        ax = str(arb_t.get("axle", "front")).lower()
        if ax not in AXLES:
            raise ValueError("spring_design.arb.axle is front or rear")
        sp = springs(new, progress=lambda *a: None)
        rc, track, radius = {}, {}, {}
        for a in AXLES:
            P = cfgmod.axle_fv(new, a); s = static_summary(P)
            rc[a], track[a], radius[a] = s["rc_height"], s["track"], P["tire_d"] / 2 - P.get("compression", 0.0)
        other = "rear" if ax == "front" else "front"
        k_other = arb_wheel_rate(new[other]["spring"].get("arb") or {})
        k, got, note = sdm.arb_for_target(arb_t, ax, {a: sp["axles"][a]["wheel_rate"] for a in AXLES},
                                          {a: sp["axles"][a]["tyre_rate"] for a in AXLES}, v,
                                          float(veh["cg_height"]), rc, track, radius, k_other)
        new[ax]["spring"]["arb"] = {"wheel_rate": round(k, 1)}
        results["arb"] = dict(axle=ax, wheel_rate=k, achieved=got, target=arb_t, note=note)
    results["springs"] = springs(new, progress=lambda *a: None)
    return new, results


def sensitivity(cfg: dict, tol=None, tol_upright=None, axles=AXLES, progress=print) -> dict:
    """Hardpoint tolerance sensitivity (see ``sensitivity.py``) for each axle. Tolerances (mm) not given
    come from the design file's ``sensitivity`` block (``tol``, ``tol_upright``), else 1 mm; its ``allow``
    overrides the allowances."""
    from . import sensitivity as sensmod
    from .sensitivity import DEFAULT_ALLOW, corner_sensitivity, tolerances
    sc = cfg.get("sensitivity") or {}
    tol = float(tol if tol is not None else sc.get("tol", 1.0))
    tol_upright = tol_upright if tol_upright is not None else sc.get("tol_upright")
    allow = sc.get("allow") or {}
    bad = [k for k in allow if k not in DEFAULT_ALLOW]
    if bad:
        raise ValueError(f"sensitivity.allow: unknown {', '.join(bad)} (use {', '.join(DEFAULT_ALLOW)})")
    tols = tolerances(tol, None if tol_upright is None else float(tol_upright))
    if min(tols.values()) <= 0:
        raise ValueError("sensitivity tolerances must be above 0 mm")
    veh = cfg["vehicle"]; res = {"tol": tols}
    for axle in axles:
        progress(f"Sensitivity: {axle} corner, {3 * len(sensmod.POINTS)} hardpoint coordinates ...")
        c, _ = build_3d(cfg, axle)
        st = (cfg[axle]["three_d"].get("steering") or {}) if axle == "front" else {}
        res[axle] = corner_sensitivity(c, axle == "front", veh, tols,
                                       track=static_summary(cfgmod.axle_fv(cfg, axle))["track"],
                                       ackermann_at=float(st.get("ackermann_at", 25.0)), allow=allow)
    return res


def write_sensitivity_csv(sens: dict, path):
    """One row per axle, point and axis: the tolerance, then each metric's change per mm."""
    from .sensitivity import METRICS
    keys = [k for k, *_ in METRICS]
    rows = [["axle", "point", "axis", "tol_mm"] + [f"{k}_per_mm" for k in keys]]
    for axle in AXLES:
        if axle in sens:
            for r in sens[axle]["rows"]:
                rows.append([axle, r["point"], r["axis"], r["tol"]]
                            + [f"{r['sens'][k]:.6g}" if k in r["sens"] else "" for k in keys])
    with open(path, "w", newline="", encoding="utf-8") as f:
        csv.writer(f).writerows(rows)


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
