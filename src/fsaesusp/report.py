"""Text tables, warnings and the Markdown report."""
from __future__ import annotations

import re
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


def compare_rows(results, labels):
    """Front-view metrics of several designs side by side. With two designs a last column holds B - A.
    Returns (rows, headers)."""
    per = [front_view_rows(r) for r in results]
    headers = ["Front view"] + list(labels) + (["Change"] if len(labels) == 2 else [])
    rows = []
    for col, axle in ((1, "Front"), (2, "Rear")):
        for i, row in enumerate(per[0]):
            vals = [p[i][col] for p in per]
            r = [f"{axle}: {row[0]}"] + vals
            if len(vals) == 2:
                try:
                    r.append(f"{float(vals[1]) - float(vals[0]):+.4g}")
                except ValueError:                  # "a / b" pairs
                    r.append("" if vals[0] == vals[1] else "changed")
            rows.append(r)
    return rows, headers


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
    rows = [["Ackermann", f"{st['ackermann_percent']:.0f}% at {st['inside_at_ack']:.0f}° inside"],
            ["Full lock (inside / outside)", f"{st['inside_lock']:.1f}° / {st['outside_lock']:.1f}°"],
            ["Rack travel for full lock", f"±{st['rack_for_lock']:.1f} mm"],
            ["Rack travel per degree (centre)", f"{st['rack_mm_per_deg']:.2f} mm/deg"]]
    if st.get("steering_ratio") is not None:
        rows.append(["Steering ratio (centre)", f"{st['steering_ratio']:.2f}:1 (c-factor {st['c_factor']:g} mm/rev)"])
    return rows + [
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
        w += spring_warnings(res3d.get("springs"))
        if res3d.get("rules"):
            from .adjust import rules_warnings
            w += rules_warnings(res3d["rules"])
    return w


def write_markdown(path, cfg, res2d, res3d=None, figures=()):
    L = [f"# {cfg.get('name', 'Suspension report')}", "", f"[Open in VSusp]({res2d['vsusp_link']})", ""]
    L += ["## Front view", "", md_table(front_view_rows(res2d), ["", "Front", "Rear"]), ""]
    if res3d:
        L += ["## 3D", "", md_table(three_d_rows(res3d), ["", "Front", "Rear"]), ""]
        if res3d.get("steering"):
            L += ["## Steering", "", md_table(steering_rows(res3d["steering"]), ["", "Value"]), ""]
        if res3d.get("springs"):
            L += springs_markdown(res3d["springs"])
        if res3d.get("adjust"):
            L += ["## Adjustments", "", md_table(adjust_rows(res3d["adjust"]), ["", "Front", "Rear"]), "",
                  "Shim: added between the upright and the upper ball-joint mount, so the joint sits 1 mm further "
                  "inboard. Each row is for one change, the rest of the linkage as built.", ""]
        if res3d.get("rules"):
            L += ["## Rules check", "", md_table(rules_rows(res3d["rules"]), RULES_HEADERS), "",
                  "Limits come from the design file's `rules` block (defaults: wheelbase 1525 mm, 25 mm jounce "
                  "and rebound). Check them against the current rules of your competition.", ""]
    ws = warnings(res2d, res3d)
    L += ["## Flags", ""] + ([f"- {x}" for x in ws] if ws else ["- none"]) + [""]
    for fig in figures:
        L += [f"![]({Path(fig).name})", ""]
    L += ["Clearance numbers use the wheel model in the design file (rim lip plane and radius, tube size) and are",
          "screening values: confirm them in CAD with your actual wheel, brake and upright.", ""]
    Path(path).write_text("\n".join(L), encoding="utf-8")


# ----------------------------------------------------------------- sensitivity
def _decimals(fmt):
    m = re.search(r"\.(\d+)f", fmt)
    return int(m.group(1)) if m else 2


def sensitivity_rows(ax_res):
    """Per metric: nominal, worst-case and RSS stack at the tolerances, the allowance, and the biggest
    contributors."""
    from .sensitivity import AS_BUILT, METRICS, top_contributors
    rows = []
    for k, lab, fmt, _ in METRICS:
        if k not in ax_res["metrics"]:
            continue
        n = _decimals(fmt); st = ax_res["stack"][k]
        top = ", ".join(f"{name} {share:.0%}" for name, share in top_contributors(ax_res, k))
        if k in AS_BUILT:
            allow, status = "set at alignment", ""
        else:
            allow, status = f"±{st['allow']:g}", "OVER" if st["over"] else "ok"
        rows.append([lab, fmt.format(_z(ax_res["nominal"][k], n)), f"±{st['worst']:.{n}f}", f"±{st['rss']:.{n}f}",
                     allow, status, top])
    return rows


SENS_HEADERS = ["", "Nominal", "Worst case", "RSS", "Allowance", "RSS vs allowance",
                "Biggest contributors (share of worst case)"]


def influence_rows(ax_res, n=8):
    """The coordinates that matter most: the kinematic metric each one moves most, against its allowance."""
    from .sensitivity import METRICS, POINT_LABELS, influence
    labels = {k: lab for k, lab, *_ in METRICS}; fmts = {k: f for k, _, f, _ in METRICS}
    rows = []
    for r, used, k in influence(ax_res)[:n]:
        d = _decimals(fmts[k]) + 1
        rows.append([f"{POINT_LABELS[r['point']]} ({r['point']}) {r['axis']}", f"±{r['tol']:g}",
                     labels[k], f"{r['effect'][k]:+.{d}f}", f"{used:.0%}"])
    return rows


INFLUENCE_HEADERS = ["Coordinate", "Tol (mm)", "Moves most", "Effect at +tol", "Of allowance"]


def sensitivity_warnings(sens):
    from .sensitivity import METRICS, top_contributors
    labels = {k: lab for k, lab, *_ in METRICS}
    w = []
    for a in AXLES:
        if a not in sens:
            continue
        for k, st in sens[a]["stack"].items():
            if st["over"]:
                top = ", ".join(f"{name} {share:.0%}" for name, share in top_contributors(sens[a], k))
                w.append(f"{a}: {labels[k].lower()} varies ±{st['rss']:.3g} (RSS) at these tolerances, over the "
                         f"±{st['allow']:g} allowance - tighten {top}")
    return w


def write_sensitivity_markdown(path, cfg, sens, figures=()):
    tol = sens["tol"]
    tc, tu = tol["UF"], tol["UBJ"]
    L = [f"# Hardpoint sensitivity - {cfg.get('name', '')}", "",
         f"Each hardpoint coordinate is moved by its tolerance (chassis ±{tc:g} mm, upright ±{tu:g} mm), one at a "
         "time. *As built* rows keep the nominal arm and tie-rod lengths, so they show the alignment adjustment "
         "needed after the build. All other rows have the links made to fit, so the static camber and toe are back "
         "at nominal and only the kinematics change. Worst case adds every coordinate's effect; RSS is the "
         "root-sum-square, closer to what a typical build sees. Values are for one corner. A metric is flagged "
         "when its RSS is over its allowance (`sensitivity.allow` in the design file).", ""]
    for axle in AXLES:
        if axle not in sens:
            continue
        L += [f"## {axle.capitalize()}", "", md_table(sensitivity_rows(sens[axle]), SENS_HEADERS), "",
              "Coordinates that use the most of an allowance on their own:", "",
              md_table(influence_rows(sens[axle]), INFLUENCE_HEADERS), ""]
    ws = sensitivity_warnings(sens)
    L += ["## Flags", ""] + ([f"- {x}" for x in ws] if ws else ["- none"]) + [""]
    for fig in figures:
        L += [f"![]({Path(fig).name})", ""]
    L += ["Coordinates: x forward, y left (outboard on the left corner), z up. The full table of sensitivities "
          "per mm is in `sensitivity.csv`.", ""]
    Path(path).write_text("\n".join(L), encoding="utf-8")


# ----------------------------------------------------------------- springs
def _f(v, fmt):
    return "—" if v is None or (isinstance(v, float) and not np.isfinite(v)) else fmt.format(v)


def spring_rows(sp):
    """Per-axle rod, rocker, damper and rate numbers (columns Front, Rear; '—' where an axle has none)."""
    A = sp["axles"]
    def g(fn):
        return [fn(A[a]) if a in A else "—" for a in AXLES]
    rows = [["Rod on / length (mm)", *g(lambda a: f"{a['arm']} arm, {a['rod_length']:.1f}")],
            ["Motion ratio, damper/wheel (static)", *g(lambda a: f"{a['mr']:.3f}")],
            ["Motion ratio at -25 / +25 mm", *g(lambda a: f"{a['mr_droop25']:.3f} / {a['mr_bump25']:.3f} ({a['progression']:+.1f}%)"
                                                if "mr_bump25" in a else "—")],
            ["Damper length static (mm)", *g(lambda a: f"{a['damper_static']:.1f}")],
            ["Damper travel over ±25 mm (mm)", *g(lambda a: _f(a.get("damper_travel25"), "{:.1f}"))],
            ["Wheel travel to damper limits, bump / droop (mm)",
             *g(lambda a: f"{_f(a['bump_available'], '{:.1f}')} / {_f(a['droop_available'], '{:.1f}')}"
                if "damper_limits" in a else "no damper.length")],
            ["Rocker rotation over ±25 mm (deg)", *g(lambda a: "{:+.1f} to {:+.1f}".format(*a["rocker_range25"])
                                                    if "rocker_range25" in a else "—")],
            ["Rod / damper angle to rocker, worst over ±25 mm (deg, 90 ideal)",
             *g(lambda a: f"{a['min_rod_angle']:.0f} / {a['min_damper_angle']:.0f}")],
            ["Rod force per kN vertical at the wheel (N)", *g(lambda a: f"{a['rod_force_per_kn']:.0f}")],
            ["Rod clearance to other links / rim over ±25 mm (mm)",
             *g(lambda a: f"{a['rod_gap_links']:.1f} / {a['rod_gap_rim']:.1f}" if "rod_gap_links" in a else "—")],
            ["Spring rate / wheel rate (N/mm)", *g(lambda a: f"{a['spring_rate']:.1f} / {a['wheel_rate']:.1f}")],
            ["Anti-roll bar rate at the wheel (N/mm)", *g(lambda a: f"{a['arb_rate']:.1f}"
                                                         + (f" (bar motion ratio {a['arb_mr']:.2f})" if "arb_mr" in a else ""))],
            ["Tyre rate / ride rate (N/mm)", *g(lambda a: f"{a['tyre_rate']:.0f}{' (assumed)' if a['tyre_rate_assumed'] else ''}"
                                                f" / {a['ride_rate']:.1f}")]]
    if sp.get("vehicle"):
        rows += [["Sprung corner mass (kg)", *g(lambda a: f"{a['sprung_corner_mass']:.1f}")],
                 ["Ride frequency (Hz)", *g(lambda a: f"{a['ride_frequency']:.2f}")],
                 ["Roll stiffness, springs + bar (Nm/deg)", *g(lambda a: f"{a['roll_stiffness']:.0f}")]]
    return rows


def vehicle_spring_rows(v):
    lt = v["load_transfer"]; vals = v["values"]; p = v["pitch"]
    rows = [["Roll stiffness (Nm/deg)", f"{v['roll_stiffness']:.0f} ({v['roll_stiffness_front_share']:.0%} front)"],
            ["Sprung mass / CG height (kg / mm)", f"{v['sprung_mass']:.0f} / {v['sprung_cg_height']:.0f}"],
            ["Roll axis height under the sprung CG (mm)", f"{v['roll_axis_height']:.1f}"],
            ["Roll gradient (deg/g)", f"{v['roll_gradient']:.2f}"],
            [f"Body roll at {vals['lateral_g']:g} g (deg)", f"{v['roll_at_lateral_g']:.2f}"
             + (f" (optimise.camber.low_speed uses {v['optimiser_roll']:g})" if "optimiser_roll" in v else "")]]
    for a in AXLES:
        x = lt[a]
        rows.append([f"{a.capitalize()} lateral load transfer per g (N)",
                     f"{x['total']:.0f} = elastic {x['elastic']:.0f} + geometric {x['geometric']:.0f} + unsprung {x['unsprung']:.0f}"])
    rows.append(["Lateral load transfer distribution, front", f"{v['lltd_front']:.1%} (front weight {vals['front_weight']:.1%})"])
    for case, lab, g in (("braking", "Braking", vals["braking_g"]), ("accel", "Acceleration", vals["accel_g"])):
        r = p[case]; t = r["travel"]
        rows.append([f"{lab} at {g:g} g: wheel travel front / rear (mm)", f"{t['front']:+.1f} / {t['rear']:+.1f} (pitch {r['pitch']:+.2f}°)"])
        rows.append([f"{lab} at {g:g} g: camber front / rear (deg)", f"{r['camber_front']:+.2f} / {r['camber_rear']:+.2f}"])
    return rows


def spring_warnings(sp, min_travel=25.0):
    w = []
    if not sp:
        return w
    for a, x in sp["axles"].items():
        if x["mr"] <= 0:
            w.append(f"{a}: the damper extends in bump (motion ratio {x['mr']:.2f}) - check the rocker points")
        if x.get("rod_gap_links", 99) < 3.0 or x.get("rod_gap_rim", 99) < 3.0:
            w.append(f"{a}: the rod comes within {min(x['rod_gap_links'], x['rod_gap_rim']):.1f} mm of another link "
                     "or the rim over +/-25 mm (< 3 mm)")
        if min(x["min_rod_angle"], x["min_damper_angle"]) < 45:
            w.append(f"{a}: the rod or damper gets within 45 deg of the rocker lever's line over +/-25 mm "
                     f"({x['min_rod_angle']:.0f} / {x['min_damper_angle']:.0f} deg) - the rocker loses leverage")
        if "damper_limits" in x:
            for key, lab in (("bump_available", "bump"), ("droop_available", "droop")):
                v = x[key]
                if v is not None and v < min_travel:
                    w.append(f"{a}: only {v:.1f} mm of {lab} before the damper reaches its "
                             f"{'compressed' if lab == 'bump' else 'extended'} length (rules check: {min_travel:g} mm)")
    v = sp.get("vehicle")
    if v:
        if v["assumed"]:
            w.append("springs: vehicle " + ", ".join(v["assumed"]) + " not in the design file - defaults used")
        if "optimiser_roll" in v and abs(v["roll_at_lateral_g"] - v["optimiser_roll"]) > 0.3:
            w.append(f"springs: the car rolls {v['roll_at_lateral_g']:.2f} deg at {v['values']['lateral_g']:g} g, "
                     f"but optimise.camber.low_speed assumes {v['optimiser_roll']:g} deg - update it so the static "
                     "camber is chosen for the roll the springs give")
    return w


def springs_markdown(sp):
    L = ["## Springs", "", md_table(spring_rows(sp), ["", "Front", "Rear"]), ""]
    if sp.get("vehicle"):
        L += ["### Ride, roll and pitch", "", md_table(vehicle_spring_rows(sp["vehicle"]), ["", "Value"]), "",
              "Load transfer is per g of lateral acceleration. Roll stiffness has the tyre in series. Pitch travel "
              "is the longitudinal load transfer less what the anti geometry carries, through the ride rates.", ""]
    return L


def write_springs_markdown(path, cfg, sp, figures=()):
    L = [f"# Springs - {cfg.get('name', '')}", ""] + springs_markdown(sp)[2:]
    ws = spring_warnings(sp)
    L += ["## Flags", ""] + ([f"- {x}" for x in ws] if ws else ["- none"]) + [""]
    for fig in figures:
        L += [f"![]({Path(fig).name})", ""]
    Path(path).write_text("\n".join(L), encoding="utf-8")


# ----------------------------------------------------------------- adjustments and rules
def adjust_rows(adj):
    def g(fn):
        return [fn(adj[a]) if a in adj else "—" for a in AXLES]
    turn = f"{adj['thread_pitch']:.3g} mm pitch" + (", turnbuckle" if adj["turnbuckle"] else "")
    rows = [["Camber per mm of shim at the upper ball joint (deg)", *g(lambda r: f"{r['shim']['camber']:+.3f}")],
            ["  roll-centre height per mm of that shim (mm)", *g(lambda r: f"{r['shim']['rc']:+.2f}")],
            ["Toe-in per mm longer tie rod / toe link (deg)", *g(lambda r: f"{r['toe_per_mm']:+.3f}")],
            [f"Toe-in per turn ({turn}) (deg)", *g(lambda r: f"{r['toe_per_turn']:+.3f}")],
            ["Per mm of ride height (car raised): RC height (mm)", *g(lambda r: f"{r['ride']['rc']:+.2f}")],
            ["  camber (deg)", *g(lambda r: f"{r['ride']['camber']:+.4f}")],
            ["  toe-in (deg)", *g(lambda r: f"{r['ride']['toe']:+.4f}")]]
    if any("ride_per_rod_mm" in adj[a] for a in AXLES if a in adj):
        rows += [["Ride height per mm longer pushrod / pullrod (mm)",
                  *g(lambda r: _f(r.get("ride_per_rod_mm"), "{:+.2f}"))],
                 ["Ride height per mm of spring perch wound up (mm)",
                  *g(lambda r: _f(r.get("ride_per_perch_mm"), "{:+.2f}"))]]
    return rows


def rules_rows(rows):
    return [list(r) for r in rows]


RULES_HEADERS = ["Rule", "Design", "Limit", "Status"]


# ----------------------------------------------------------------- spring design
LAYOUT_LABELS = dict(at="Rod pickup along the arm (fraction)", pivot_x="Rocker pivot x (mm)",
                     pivot_y="Rocker pivot y (mm)", pivot_z="Rocker pivot z (mm)", rod_lever="Rod lever (mm)",
                     damper_lever="Damper lever (mm)", rod_lever_angle="Rod lever turned from square (deg)",
                     damper_lever_angle="Damper lever turned from square (deg)",
                     damper_tilt="Damper tilted about the rod (deg)", damper_swing="Damper swung in the rocker plane (deg)")


def layout_rows(r):
    """The layout variables: generated start, optimised value and the range searched."""
    b = r["setup"]["bounds"]; rows = []
    for k, lab in LAYOUT_LABELS.items():
        fmt = "{:.3f}" if k == "at" else "{:.1f}"
        lo, hi = b[k]
        rng = "locked" if k in r["locked"] else (f"fixed" if lo == hi else f"{fmt.format(lo)} to {fmt.format(hi)}")
        rows.append([lab, fmt.format(r["start"]["values"][k]), fmt.format(r["values"][k]), rng])
    return rows


LAYOUT_HEADERS = ["Layout", "Generated", "Optimised", "Range"]


def design_target_rows(r):
    T = r["setup"]["ctx"]["targets"]; e, e0 = r["result"], r["start"]["result"]
    B, D = T["travel"]
    def g(res, fn):
        return "—" if res is None else fn(res)
    spec = [("Motion ratio (static)", f"{T['mr']:g}", lambda x: f"{x['mr']:.3f}"),
            ("Progression -25 to +25 mm (%)", "{:g} to {:g}".format(*T["progression"]), lambda x: f"{x['progression']:+.1f}"),
            ("Wheel travel to damper stops, bump / droop (mm)", f">= {B + T['travel_margin']:g} / {D + T['travel_margin']:g}",
             lambda x: f"{B + x['margin']:.1f} / {D + x['margin']:.1f}"),
            ("Rod / damper angle to lever, worst (deg)", f">= {T['min_angle']:g}",
             lambda x: f"{x['min_rod_angle']:.0f} / {x['min_damper_angle']:.0f}"),
            ("Rocker rotation, most either way (deg)", f"<= {T['max_rotation']:g}", lambda x: f"{x['max_rotation']:.1f}"),
            ("Rod gap to other links / rim (mm)", f">= {T['rod_clearance']:g} / {T['rim_clearance']:g}",
             lambda x: f"{x['rod_gap']:.1f} / {x['rim_gap']:.1f}"),
            ("Rod force per unit wheel force", f"<= {T['rod_force_max']:g}", lambda x: f"{x['rod_force']:.2f}")]
    return [[lab, tgt, g(e0, fn), g(e, fn)] for lab, tgt, fn in spec]


DESIGN_TARGET_HEADERS = ["Target", "Wanted", "Generated", "Optimised"]


def rate_lines(res):
    from .springdesign import LB_IN
    L = []
    for a in AXLES:
        if a not in res:
            continue
        r = res[a]; T = r["setup"]["ctx"]["targets"]
        if T.get("ride_frequency"):
            L.append(f"{a}: {T['ride_frequency']:g} Hz needs {r['spring_rate_exact']:.1f} N/mm "
                     f"({r['spring_rate_exact'] / LB_IN:.0f} lb/in); using {r['spring_rate']:.1f} N/mm "
                     f"({r['spring_rate'] / LB_IN:.0f} lb/in)")
        else:
            L.append(f"{a}: spring rate kept at {r['spring_rate']:.1f} N/mm (no ride_frequency target)")
    if res.get("arb"):
        x = res["arb"]; key = "lltd_front" if "lltd_front" in x["target"] else "roll_gradient"
        shown = (f"front LLTD {x['achieved']:.1%}" if key == "lltd_front" else f"roll gradient {x['achieved']:.2f} deg/g")
        L.append(f"anti-roll bar: {x['wheel_rate']:.1f} N/mm at each {x['axle']} wheel gives {shown}"
                 + (f" ({x['note']})" if x["note"] else ""))
    return L


def design_warnings(res):
    w = []
    for a in AXLES:
        if a not in res:
            continue
        t = res[a]["terms"]
        names = dict(mr="motion ratio", progression="progression", travel="travel to the damper stops",
                     angles="rod/damper angles", rotation="rocker rotation", rod_clearance="rod clearance to links",
                     rim_clearance="rod clearance to the rim", rod_force="rod force", damper_box="damper box")
        missed = [names[k] for k, v in t.items() if k in names and v > 0.05]
        if missed:
            w.append(f"{a}: the layout misses " + ", ".join(missed) + " - widen the bounds or ease the targets")
    if res.get("arb") and res["arb"]["note"]:
        w.append("anti-roll bar: " + res["arb"]["note"])
    return w
