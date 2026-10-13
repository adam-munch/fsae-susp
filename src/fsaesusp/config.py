"""Design files (YAML or JSON): load, save, defaults, and conversion to the
front-view parameter dicts used by the solvers."""
from __future__ import annotations

import copy
import json
from pathlib import Path

import yaml

from . import vsusp

DEFAULTS = {
    "name": "FSAE design",
    "vehicle": {"wheelbase": 1550.0, "cg_height": 280.0, "front_brake_bias": 0.65, "inboard_drive": True},
    "tyre": {"diameter": 411.48, "width": 213.36, "static_deflection": 5.0},
    "wheel": {"rim_diameter": 254.0, "offset": 25.4, "lip_inboard": 107.0, "lip_radius": 122.0,
              "link_radius": 8.0, "node_inboard_of_lip": 35.0},
    "front": {"three_d": {"caster": 5.0, "kingpin_offset_x": 0.0, "anti": 20.0,
                          "lower_pivot_x": [150.0, -150.0], "upper_pivot_x": [120.0, -120.0],
                          "tie_rod": {"x": -60.0, "inner": "auto"},
                          "steering": {"lock_inside": 27.0, "ackermann_at": 25.0}}},
    "rear": {"three_d": {"caster": 0.0, "kingpin_offset_x": 0.0, "anti": 25.0,
                         "lower_pivot_x": [150.0, -150.0], "upper_pivot_x": [120.0, -120.0],
                         "tie_rod": {"x": -70.0, "inner": "auto"}}},
    "optimise": {"camber": {"outside_target": -1.5, "low_speed": [1.5, 0.0], "high_speed": [1.8, 10.0]},
                 "front": {"targets": {}, "bounds": {}, "lock": []}, "rear": {"targets": {}, "bounds": {}, "lock": []}},
}

AXLES = ("front", "rear")


def deep_merge(base: dict, over: dict) -> dict:
    out = copy.deepcopy(base)
    for k, v in (over or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def load(path) -> dict:
    p = Path(path)
    text = p.read_text(encoding="utf-8")
    data = json.loads(text) if p.suffix.lower() == ".json" else yaml.safe_load(text)
    cfg = deep_merge(DEFAULTS, data or {})
    for axle in AXLES:
        for key in ("ride_height", "lower_pivot", "upper_pivot", "lower_arm", "upper_arm", "knuckle"):
            if key not in cfg.get(axle, {}):
                raise ValueError(f"{path}: '{axle}.{key}' is missing")
    return cfg


AXLE_ORDER = ("ride_height", "lower_pivot", "upper_pivot", "lower_arm", "upper_arm", "knuckle", "tyre", "wheel", "three_d")
TOP_ORDER = ("name", "vehicle", "tyre", "wheel", "front", "rear", "optimise")


def _ordered(cfg: dict) -> dict:
    out = {k: cfg[k] for k in TOP_ORDER if k in cfg}
    out.update({k: v for k, v in cfg.items() if k not in out})
    for axle in AXLES:
        if axle in out:
            a = out[axle]
            o = {k: a[k] for k in AXLE_ORDER if k in a}
            o.update({k: v for k, v in a.items() if k not in o})
            out[axle] = o
    return out


def save(cfg: dict, path, header: str | None = None):
    p = Path(path)
    clean = _plain(_ordered(cfg))
    if p.suffix.lower() == ".json":
        p.write_text(json.dumps(clean, indent=2), encoding="utf-8")
    else:
        txt = yaml.safe_dump(clean, sort_keys=False, default_flow_style=None, width=100)
        p.write_text((f"# {header}\n" if header else "") + txt, encoding="utf-8")


def _plain(x):
    if isinstance(x, dict):
        return {k: _plain(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_plain(v) for v in x]
    try:
        import numpy as np
        if isinstance(x, np.generic):
            return x.item()
    except ImportError:
        pass
    if isinstance(x, float):
        return round(x, 4)
    return x


def axle_tyre_wheel(cfg, axle):
    tyre = deep_merge(cfg["tyre"], cfg[axle].get("tyre", {}))
    wheel = deep_merge(cfg["wheel"], cfg[axle].get("wheel", {}))
    return tyre, wheel


def axle_fv(cfg: dict, axle: str) -> dict:
    """Front-view parameter dict (see geometry2d) for one axle."""
    a = cfg[axle]; tyre, wheel = axle_tyre_wheel(cfg, axle); k = a["knuckle"]
    return dict(bottom_y=float(a["ride_height"]),
                cx_l=float(a["lower_pivot"][0]), by_l=float(a["lower_pivot"][1]),
                cx_u=float(a["upper_pivot"][0]), by_u=float(a["upper_pivot"][1]),
                lower_len=float(a["lower_arm"]), upper_len=float(a["upper_arm"]),
                hux=float(k["upper_ball_joint"][0]), huy=float(k["upper_ball_joint"][1]),
                hlx=float(k["lower_ball_joint"][0]), hly=float(k["lower_ball_joint"][1]),
                otr_x=float(k["tie_rod"][0]), otr_y=float(k["tie_rod"][1]),
                offset=float(wheel["offset"]), wheel_d=float(wheel["rim_diameter"]),
                tire_d=float(tyre["diameter"]), tire_w=float(tyre["width"]),
                compression=float(tyre["static_deflection"]))


def set_axle_fv(cfg: dict, axle: str, P: dict):
    """Write chassis pivots, arm lengths and knuckle points back into the design."""
    a = cfg[axle]
    a["ride_height"] = float(P["bottom_y"])
    a["lower_pivot"] = [round(float(P["cx_l"]), 2), round(float(P["by_l"]), 2)]
    a["upper_pivot"] = [round(float(P["cx_u"]), 2), round(float(P["by_u"]), 2)]
    a["lower_arm"] = round(float(P["lower_len"]), 2)
    a["upper_arm"] = round(float(P["upper_len"]), 2)
    a["knuckle"] = {"upper_ball_joint": [float(P["hux"]), float(P["huy"])],
                    "lower_ball_joint": [float(P["hlx"]), float(P["hly"])],
                    "tie_rod": [round(float(P["otr_x"]), 2), round(float(P["otr_y"]), 2)]}


def from_vsusp(link: str) -> dict:
    """New design dict from a VSusp link (3D/vehicle settings take defaults)."""
    d = vsusp.parse_link(link)
    cfg = copy.deepcopy(DEFAULTS)
    cfg["name"] = d["project_name"] or "Imported from VSusp"
    F = d["front"]
    cfg["tyre"] = {"diameter": F["tire_d"], "width": F["tire_w"], "static_deflection": F.get("compression", 0.0)}
    cfg["wheel"]["rim_diameter"] = F["wheel_d"]; cfg["wheel"]["offset"] = F["offset"]
    for axle in AXLES:
        P = d[axle]
        set_axle_fv(cfg, axle, P)
        for key, glob, sub in (("tire_d", "tyre", "diameter"), ("tire_w", "tyre", "width"),
                               ("compression", "tyre", "static_deflection"), ("offset", "wheel", "offset"),
                               ("wheel_d", "wheel", "rim_diameter")):
            if abs(float(P[key]) - float(cfg[glob][sub])) > 1e-9:
                cfg[axle].setdefault(glob, {})[sub] = P[key]
    return cfg


def to_vsusp_link(cfg: dict) -> str:
    return vsusp.build_link(axle_fv(cfg, "front"), axle_fv(cfg, "rear"), cfg.get("name", "fsae-susp design"))
