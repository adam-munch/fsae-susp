"""Read and write VSusp (vsusp.com) project links.

A VSusp link stores the whole project in the URL fragment, e.g.::

    https://www.vsusp.com/#0.8%26project_name%3AMy%20car%26trim%7B...%7D%26front%7B...%7D%26rear%7B...%7D%26pref%7B...%7D

Numeric fields are stored as integers scaled by 100 (mm x 100, deg x 100), except a
few enum/boolean fields.
"""
from __future__ import annotations

import urllib.parse

UNSCALED = {"frame.susp_type", "steering.active", "tires.size_convention"}

# VSusp field -> internal front-view key (see geometry2d.FV_KEYS)
FIELD_MAP = {
    "frame.bottom_y": "bottom_y",
    "frame.center_to_upper_mount_x": "cx_u",
    "frame.bottom_to_upper_mount_y": "by_u",
    "frame.center_to_lower_mount_x": "cx_l",
    "frame.bottom_to_lower_mount_y": "by_l",
    "control_arms.upper_length": "upper_len",
    "control_arms.lower_length": "lower_len",
    "knuckles.hub_to_upper_x": "hux",
    "knuckles.hub_to_upper_y": "huy",
    "knuckles.hub_to_lower_x": "hlx",
    "knuckles.hub_to_lower_y": "hly",
    "steering.hub_to_outer_tie_rod_x": "otr_x",
    "steering.hub_to_outer_tie_rod_y": "otr_y",
    "wheels.offset": "offset",
    "tires.compression": "compression",
}

DEFAULT_PREF = ("diag1.px_per_mm:200|diag1.front_or_rear:front|tab.active:0|units:1|show.f:1|show.ca:1|"
                "show.k:1|show.st:1|show.stl:1|show.w:1|show.t:1|show.rc:1|show.rcl:1|show.ic:1|show.icl:1|"
                "show.fvsa:0|show.tl:1|show.kpil:1|redraw_during_drag:1|chart.x_axis_center:0|"
                "chart.x_axis_window:4|chart.x_axis_num_steps:21|chart.x_axis_field:trim.body_roll_angle|"
                "chart.y_axis_fields:[FR].general.roll_center.y")


def _split_sections(raw: str):
    """Split 'a&b{...}&c{...}' at top-level '&' (not inside braces)."""
    parts, depth, cur = [], 0, []
    for ch in raw:
        if ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
        if ch == "&" and depth == 0:
            parts.append("".join(cur)); cur = []
        else:
            cur.append(ch)
    parts.append("".join(cur))
    return parts


def _parse_block(body: str) -> dict:
    out = {}
    for item in body.split("|"):
        if ":" not in item:
            continue
        k, v = item.split(":", 1)
        out[k] = v
    return out


def parse_link(link: str) -> dict:
    """Parse a VSusp link (or bare fragment). Returns
    ``{"version", "project_name", "front": FV, "rear": FV, "raw": {...}}``."""
    frag = link.split("#", 1)[1] if "#" in link else link
    raw = urllib.parse.unquote(frag)
    sections = _split_sections(raw)
    res = {"version": sections[0], "project_name": "", "raw": {}}
    for s in sections[1:]:
        if s.startswith("project_name:"):
            res["project_name"] = s.split(":", 1)[1]
        elif "{" in s and s.endswith("}"):
            name, body = s.split("{", 1)
            res["raw"][name] = _parse_block(body[:-1])
    for axle in ("front", "rear"):
        blk = res["raw"].get(axle)
        if blk is None:
            raise ValueError(f"link has no '{axle}' block")
        res[axle] = _block_to_fv(blk, axle)
    return res


def _num(blk, key, scaled=True):
    v = float(blk[key])
    return v if (not scaled or key in UNSCALED) else v / 100.0


def _block_to_fv(blk: dict, axle: str) -> dict:
    if int(_num(blk, "frame.susp_type", False)) != 0:
        raise ValueError(f"{axle}: only double-wishbone suspensions (susp_type 0) are supported")
    P = {ik: _num(blk, vk) for vk, ik in FIELD_MAP.items() if vk in blk}
    if int(_num(blk, "tires.size_convention", False)) == 1:      # explicit diameters
        P["wheel_d"] = _num(blk, "wheels.diameter_expl")
        P["tire_d"] = _num(blk, "tires.diameter_expl")
        P["tire_w"] = _num(blk, "tires.width_expl")
    else:                                                          # e.g. 195/45R15
        sw, ar, rim_in = _num(blk, "tires.section_width"), _num(blk, "tires.aspect_ratio"), _num(blk, "wheels.diameter")
        P["wheel_d"] = rim_in * 25.4
        P["tire_d"] = 2 * sw * ar / 100.0 + rim_in * 25.4
        P["tire_w"] = sw
    if "steering.active" in blk and int(_num(blk, "steering.active", False)) == 0:
        P.setdefault("otr_x", 0.0); P.setdefault("otr_y", 0.0)
    P.setdefault("compression", 0.0)
    return P


def _fv_to_block(P: dict) -> str:
    f = lambda v: str(int(round(float(v) * 100)))
    items = [("frame.susp_type", "0"), ("frame.bottom_y", f(P["bottom_y"])),
             ("frame.center_to_upper_mount_x", f(P["cx_u"])), ("frame.bottom_to_upper_mount_y", f(P["by_u"])),
             ("frame.center_to_lower_mount_x", f(P["cx_l"])), ("frame.bottom_to_lower_mount_y", f(P["by_l"])),
             ("control_arms.upper_length", f(P["upper_len"])), ("control_arms.lower_length", f(P["lower_len"])),
             ("knuckles.hub_to_upper_x", f(P["hux"])), ("knuckles.hub_to_lower_x", f(P["hlx"])),
             ("knuckles.hub_to_lower_y", f(P["hly"])), ("knuckles.hub_to_upper_y", f(P["huy"])),
             ("knuckles.hub_to_strut_axis", "14000"), ("knuckles.strut_incl", "8000"),
             ("steering.active", "1"), ("steering.hub_to_outer_tie_rod_x", f(P["otr_x"])),
             ("steering.hub_to_outer_tie_rod_y", f(P["otr_y"])),
             ("wheels.offset", f(P["offset"])), ("wheels.diameter", f(P.get("wheel_d", 254.0) / 25.4)),
             ("wheels.diameter_expl", f(P.get("wheel_d", 254.0))), ("tires.size_convention", "1"),
             ("tires.section_width", "19500"), ("tires.aspect_ratio", "4500"),
             ("tires.diameter_expl", f(P["tire_d"])), ("tires.width_expl", f(P.get("tire_w", 200.0))),
             ("tires.compression", f(P.get("compression", 0.0)))]
    return "|".join(f"{k}:{v}" for k, v in items)


def build_link(front: dict, rear: dict, project_name: str = "fsae-susp design", pref: str = DEFAULT_PREF) -> str:
    """Build a VSusp link from two front-view parameter dicts."""
    name = "".join(ch for ch in project_name if ch not in "&{}|:#%")
    raw = (f"0.8&project_name:{name}&trim{{body_roll_angle:0|front.left_bump:0|rear.left_bump:0|"
           f"front.right_bump:0|rear.right_bump:0}}&front{{{_fv_to_block(front)}}}&rear{{{_fv_to_block(rear)}}}"
           f"&pref{{{pref}}}")
    return "https://www.vsusp.com/#" + urllib.parse.quote(raw, safe="")
