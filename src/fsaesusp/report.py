"""Text tables, warnings and the Markdown report."""
from __future__ import annotations

from pathlib import Path

import numpy as np

AXLES = ("front", "rear")


def _z(v, n):
    """Round and drop the sign of negative zero (avoids '-0.00')."""
    return round(float(v), n) + 0.0


def table(rows, headers):
    w = [max(len(str(r[i])) for r in [headers] + rows) for i in range(len(headers))]
    line = lambda r: "  ".join(str(c).ljust(w[i]) for i, c in enumerate(r))
    return "\n".join([line(headers), line(["-" * x for x in w])] + [line(r) for r in rows])


def md_table(rows, headers):
    out = ["| " + " | ".join(headers) + " |", "|" + "|".join("---" for _ in headers) + "|"]
    out += ["| " + " | ".join(str(c) for c in r) + " |" for r in rows]
    return "\n".join(out)


def front_view_rows(res2d):
    f, r = res2d["front"], res2d["rear"]
    def g(ax, sec, k, fmt="{:.1f}"):
        return fmt.format(res2d[ax][sec][k])
    spec = [("Static camber (deg)", "static", "camber", "{:.2f}"), ("Track (mm)", "static", "track", "{:.1f}"),
            ("Roll-centre height (mm)", "static", "rc_height", "{:.1f}"),
            ("RC change per mm heave", "metrics", "rc_heave_slope", "{:+.2f}"),
            ("RC lateral migration (mm/deg)", "metrics", "rc_lateral", "{:.1f}"),
            ("Swing-arm length (mm)", "static", "fvsa", "{:.0f}"),
            ("Camber gain (deg/mm)", "metrics", "camber_gain", "{:+.4f}"),
            ("Outside camber per deg roll", "metrics", "roll_camber", "{:+.3f}"),
            ("Outside camber, low-speed case (deg)", "metrics", "outside_camber_low_speed", "{:+.2f}"),
            ("Outside camber, high-speed case (deg)", "metrics", "outside_camber_high_speed", "{:+.2f}"),
            ("KPI (deg)", "static", "kpi", "{:.2f}"), ("Scrub radius (mm)", "static", "scrub", "{:+.1f}"),
            ("Kingpin offset at hub height (mm)", "static", "kp_offset_wc", "{:.1f}"),
            ("Lower / upper arm incl. (deg)", None, None, None),
            ("Lower / upper arm length (mm)", None, None, None)]
    rows = []
    for lab, sec, k, fmt in spec:
        if sec is None:
            if "incl" in lab:
                vals = [f"{res2d[a]['static']['lca_incl']:.1f} / {res2d[a]['static']['uca_incl']:.1f}" for a in AXLES]
            else:
                vals = [f"{res2d[a]['static']['lower_len']:.1f} / {res2d[a]['static']['upper_len']:.1f}" for a in AXLES]
            rows.append([lab] + vals)
        else:
            rows.append([lab, g("front", sec, k, fmt), g("rear", sec, k, fmt)])
    return rows


def three_d_rows(res3d):
    rows = []
    for lab, fn in (("Caster (deg)", lambda a: f"{_z(res3d[a]['static']['caster'], 2):.2f}"),
                    ("Mechanical trail (mm)", lambda a: f"{_z(res3d[a]['static']['trail'], 1):.1f}"),
                    ("KPI (deg)", lambda a: f"{res3d[a]['static']['kpi']:.2f}"),
                    ("Scrub radius (mm)", lambda a: f"{res3d[a]['static']['scrub']:+.1f}"),
                    ("Anti geometry (%)", lambda a: ", ".join(f"{k.replace('_', '-')} {v:.0f}" for k, v in res3d[a]['anti'].items())),
                    ("Pivot-axis tilt (deg)", lambda a: f"{res3d[a]['info']['slope_deg']:+.2f}"),
                    ("Bump steer, max over ±30 mm (deg)", lambda a: f"{max(abs(np.array(res3d[a]['bump']['toe_in']))):.4f}"),
                    ("Roll steer, max over ±2 deg (deg)", lambda a: f"{max(abs(v) for v in res3d[a]['roll_steer'].values()):.4f}"),
                    ("Caster over ±30 mm (deg)", lambda a: f"{_z(min(res3d[a]['bump']['caster']), 2):.2f} to {_z(max(res3d[a]['bump']['caster']), 2):.2f}"),
                    ("Wheelbase change over ±30 mm (mm)", lambda a: f"{min(res3d[a]['bump']['cp_x']):+.1f} to {max(res3d[a]['bump']['cp_x']):+.1f}"),
                    ("Tie rod / toe link length (mm)", lambda a: f"{res3d[a]['info']['link_length']:.1f}"),
                    ("Inner tie rod / toe link (x, y, z)", lambda a: "({:.1f}, {:.1f}, {:.1f})".format(*res3d[a]['info']['itr'])),
                    ("VSusp tie-rod inputs (x, y)", lambda a: "{:.2f}, {:.2f}".format(*res3d[a]['info']['vsusp_tie_rod']))):
        rows.append([lab, fn("front"), fn("rear")])
    for a in AXLES:
        pass
    links = sorted(set(k for a in AXLES for k in res3d[a].get("clearance_worst", {})))
    for k in links:
        vals = []
        for a in AXLES:
            w = res3d[a].get("clearance_worst", {}).get(k)
            vals.append("—" if w is None else f"{w['clearance']:.1f} (at {w['heave']:+.0f} mm, {w['steer']:+.1f}°)")
        rows.append([f"Min clearance, {k} (mm)"] + vals)
    return rows


def steering_rows(st):
    return [["Ackermann", f"{st['ackermann_percent']:.0f}% at {st['inside_at_ack']:.0f}° inside"],
            ["Full lock (inside / outside)", f"{st['inside_lock']:.1f}° / {st['outside_lock']:.1f}°"],
            ["Rack travel for full lock", f"±{st['rack_for_lock']:.1f} mm"],
            ["Rack travel per degree (centre)", f"{st['rack_mm_per_deg']:.2f} mm/deg"],
            ["Camber at full lock (inside / outside)", f"{st['camber_inside_lock']:+.2f}° / {st['camber_outside_lock']:+.2f}°"],
            ["Trail at full lock (inside / outside)", f"{st['trail_inside_lock']:.1f} / {st['trail_outside_lock']:.1f} mm"],
            ["Lower-ball-joint lift at lock (inside / outside)", f"{st['lift_inside_lock']:+.2f} / {st['lift_outside_lock']:+.2f} mm"],
            ["Arm-tie-rod angle (lock, centre, lock)", " / ".join(f"{a:.0f}°" for a in st['arm_tie_rod_angles'])]]


def warnings(res2d, res3d=None, min_clearance=3.0):
    w = []
    for a in AXLES:
        m = res2d[a]["metrics"]
        if m["outside_camber_low_speed"] > -0.25:
            w.append(f"{a}: outside wheel only {m['outside_camber_low_speed']:+.2f} deg in the low-speed case - add static camber or camber recovery")
        if m["rc_at_heave"] < 0:
            w.append(f"{a}: roll centre below ground at the high-speed heave case ({m['rc_at_heave']:.1f} mm)")
        if abs(m["half_track_bump25"]) > 3 or abs(m["half_track_droop25"]) > 3:
            w.append(f"{a}: half-track change over +/-25 mm exceeds 3 mm")
    if res3d:
        for a in AXLES:
            for k, v in res3d[a].get("clearance_worst", {}).items():
                if v["clearance"] < min_clearance:
                    w.append(f"{a}: {k} clearance {v['clearance']:.1f} mm (< {min_clearance} mm) at {v['heave']:+.0f} mm, {v['steer']:+.1f} deg steer")
            if max(abs(np.array(res3d[a]["bump"]["toe_in"]))) > 0.05:
                w.append(f"{a}: bump steer above 0.05 deg over +/-30 mm")
        st = res3d.get("steering")
        if st:
            if not 0 <= st["ackermann_percent"] <= 110:
                w.append(f"front: Ackermann {st['ackermann_percent']:.0f}% is outside 0-110%")
            if min(st["arm_tie_rod_angles"]) < 20 or max(st["arm_tie_rod_angles"]) > 160:
                w.append("front: steering arm and tie rod approach a straight line at lock (lock-up risk)")
    return w


def write_markdown(path, cfg, res2d, res3d=None, figures=()):
    L = [f"# {cfg.get('name', 'Suspension report')}", "", f"[Open in VSusp]({res2d['vsusp_link']})", ""]
    L += ["## Front view", "", md_table(front_view_rows(res2d), ["", "Front", "Rear"]), ""]
    if res3d:
        L += ["## 3D", "", md_table(three_d_rows(res3d), ["", "Front", "Rear"]), ""]
        if res3d.get("steering"):
            L += ["## Steering", "", md_table(steering_rows(res3d["steering"]), ["", "Value"]), ""]
    ws = warnings(res2d, res3d)
    L += ["## Flags", ""] + ([f"- {x}" for x in ws] if ws else ["- none"]) + [""]
    for fig in figures:
        L += [f"![]({Path(fig).name})", ""]
    L += ["Clearance numbers use the wheel model in the design file (rim lip plane and radius, tube size) and are",
          "screening values: confirm them in CAD with your actual wheel, brake and upright.", ""]
    Path(path).write_text("\n".join(L), encoding="utf-8")
