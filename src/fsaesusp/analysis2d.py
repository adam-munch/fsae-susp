"""Front-view analysis: static numbers, heave/roll/travel sweeps and the
derivative metrics used by the optimiser."""
from __future__ import annotations

import numpy as np
from scipy.optimize import brentq

from .geometry2d import Corner2D, axle_state, build_from_pose, zero_bump_steer_point, vsusp_rack_point

H = 0.05  # finite-difference step (mm or deg)


def _ax(c, th0, roll=0.0, zr=0.0, zl=0.0):
    return axle_state(c, roll, zr, zl, theta0=th0)


def static_summary(P: dict) -> dict:
    """Static alignment and front-view numbers for one axle."""
    c = Corner2D(P)
    s = axle_state(c)
    r = s["r"]
    lca = np.degrees(np.arctan2(*(r["L"] - r["A"])[::-1]))
    uca = np.degrees(np.arctan2(*(r["U"] - r["B"])[::-1]))
    zb, Lz = zero_bump_steer_point(c)
    vr = vsusp_rack_point(c)
    return dict(
        camber=s["cam_r"], track=s["track"], rc_height=s["rc"][1], ic=s["ic_r"].tolist(),
        fvsa=s["fvsa_r"], kpi=s["kpi_r"], scrub=s["scrub_r"], kp_offset_wc=s["kp_offset_wc_r"],
        lca_incl=lca, uca_incl=uca, lower_len=P["lower_len"], upper_len=P["upper_len"],
        lbj=r["L"].tolist(), ubj=r["U"].tolist(), wheel_centre=r["W"].tolist(), contact=r["T"].tolist(),
        outer_tie_rod=r["O"].tolist(),
        zero_bump_steer_rack=[zb[0], zb[1] + P["bottom_y"]], tie_rod_len=Lz,
        vsusp_rack=[vr[0], vr[1] + P["bottom_y"]],
    )


def metrics(P: dict, roll_op: float = 1.5, heave_op: float = 10.0) -> dict:
    """Derivative metrics: camber gain, roll camber, RC heave slope and lateral
    migration, outside-wheel camber at an operating point, half-track change."""
    c = Corner2D(P)
    s0 = axle_state(c)
    th0 = s0["r"]["theta"]
    m = dict(rc=s0["rc"][1], camber=s0["cam_r"], fvsa=s0["fvsa_r"], track=s0["track"],
             kpi=s0["kpi_r"], scrub=s0["scrub_r"])
    a, b = _ax(c, th0, 0, H, H), _ax(c, th0, 0, -H, -H)
    m["rc_heave_slope"] = ((a["rc"][1] - H) - (b["rc"][1] + H)) / (2 * H)
    m["camber_gain"] = (a["cam_r"] - b["cam_r"]) / (2 * H)
    ap, am = _ax(c, th0, H), _ax(c, th0, -H)
    m["roll_camber"] = (ap["cam_l"] - am["cam_l"]) / (2 * H)          # outside wheel, deg/deg
    rr = _ax(c, th0, roll_op)
    m["rc_lateral"] = rr["rc"][0] / roll_op                            # mm per deg
    m["outside_camber_roll"] = rr["cam_l"]
    m["inside_camber_roll"] = rr["cam_r"]
    ra = _ax(c, th0, roll_op, heave_op, heave_op)
    m["outside_camber_roll_heave"] = ra["cam_l"]
    z10 = _ax(c, th0, 0, heave_op, heave_op)
    m["rc_at_heave"] = z10["rc"][1] - heave_op
    T0 = s0["r"]["T"][0]
    m["half_track_bump25"] = c.solve(0, 25, th0)["T"][0] - T0
    m["half_track_droop25"] = c.solve(0, -25, th0)["T"][0] - T0
    r = s0["r"]
    m["lca_incl"] = float(np.degrees(np.arctan2(*(r["L"] - r["A"])[::-1])))
    m["uca_incl"] = float(np.degrees(np.arctan2(*(r["U"] - r["B"])[::-1])))
    m["lower_len"], m["upper_len"] = P["lower_len"], P["upper_len"]
    return {k: (float(v) if np.ndim(v) == 0 else v) for k, v in m.items()}


def outside_camber(P: dict, roll: float, heave: float) -> float:
    c = Corner2D(P)
    return float(axle_state(c, roll, heave, heave)["cam_l"])


def heave_sweep(P: dict, zs) -> dict:
    c = Corner2D(P); th0 = c.solve()["theta"]
    rc, cam, track = [], [], []
    for z in zs:
        s = _ax(c, th0, 0, z, z)
        rc.append(s["rc"][1] - z); cam.append(s["cam_r"]); track.append(s["track"])
    return dict(z=list(map(float, zs)), rc_height=rc, camber=cam, track=track)


def roll_sweep(P: dict, rolls) -> dict:
    c = Corner2D(P); th0 = c.solve()["theta"]
    out = dict(roll=list(map(float, rolls)), rc_x=[], rc_y=[], camber_out=[], camber_in=[], track=[])
    for p in rolls:
        s = _ax(c, th0, p)
        out["rc_x"].append(s["rc"][0]); out["rc_y"].append(s["rc"][1])
        out["camber_out"].append(s["cam_l"]); out["camber_in"].append(s["cam_r"]); out["track"].append(s["track"])
    return out


def travel_sweep(P: dict, zs) -> dict:
    """Single-wheel travel with the body fixed (VSusp 'bump')."""
    c = Corner2D(P); s0 = c.solve(); th0 = s0["theta"]
    zb, L0 = zero_bump_steer_point(c)
    zb_w = zb + np.array([0, c.ride])
    out = dict(z=list(map(float, zs)), camber=[], half_track=[], tie_rod_error=[])
    for z in zs:
        s = c.solve(0, z, th0)
        out["camber"].append(s["camber"]); out["half_track"].append(s["T"][0] - s0["T"][0])
        out["tie_rod_error"].append(float(np.linalg.norm(zb_w - s["O"]) - L0))
    return out


def pick_static_camber(knuckle, lower_pivot, upper_pivot, half_track, ride, tire_d, compression, offset,
                       target=-1.5, low_speed=(1.5, 0.0), high_speed=(1.8, 10.0), tire_w=200.0, wheel_d=254.0):
    """Static camber so the outside wheel averages ``target`` over the low- and
    high-speed (roll deg, heave mm) cases."""
    def f(cam):
        P = build_from_pose(knuckle, lower_pivot, upper_pivot, half_track, cam, ride, tire_d, compression,
                            offset, tire_w, wheel_d)
        return 0.5 * (outside_camber(P, *low_speed) + outside_camber(P, *high_speed)) - target
    return brentq(f, -5.0, 1.0, xtol=1e-6)
