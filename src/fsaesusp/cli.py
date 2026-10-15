"""Command-line interface: ``fsae-susp <command> ...`` (or ``python -m fsaesusp``)."""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

from . import __version__
from . import config as cfgmod
from . import api, plots, report
from .optimize import LOCKABLE
from .paths import default_design_path


def _out(path):
    p = Path(path); p.mkdir(parents=True, exist_ok=True); return p


def cmd_import(a):
    link = a.link
    if not link.lstrip().startswith(("http", "0.")) and Path(link).exists():
        link = Path(link).read_text(encoding="utf-8").strip()      # link saved in a text file
    cfg = cfgmod.from_vsusp(link)
    out = Path(a.output) if a.output else default_design_path(cfg["name"])
    out.parent.mkdir(parents=True, exist_ok=True)
    cfgmod.save(cfg, out, header="fsae-susp design file (imported from VSusp). 3D/vehicle values are defaults - edit them.")
    print(f"Wrote {out}  (project '{cfg['name']}')")
    print("Next: fsae-susp analyze", out)


def cmd_link(a):
    print(cfgmod.to_vsusp_link(cfgmod.load(a.design)))


def cmd_analyze(a):
    cfg = cfgmod.load(a.design)
    res = api.analyze(cfg)
    print(f"\n{cfg.get('name', '')}\n")
    print(report.table(report.front_view_rows(res), ["Front view", "Front", "Rear"]))
    ws = report.warnings(res)
    if ws:
        print("\nFlags:"); [print("  -", w) for w in ws]
    print("\nVSusp link:\n" + res["vsusp_link"])
    if a.output:
        out = _out(a.output)
        designs = [("", {ax: res[ax]["fv"] for ax in api.AXLES})]
        if a.compare:
            c2 = cfgmod.load(a.compare)
            designs[0] = (a.label or "design", designs[0][1])
            designs.append((a.compare_label or "compare", {ax: cfgmod.axle_fv(c2, ax) for ax in api.AXLES}))
        plots.front_view(designs, out / "front_view.png", title=f"Front-view kinematics - {cfg.get('name', '')}")
        report.write_markdown(out / "report_front_view.md", cfg, res, None, [out / "front_view.png"])
        print(f"\nWrote {out / 'front_view.png'} and {out / 'report_front_view.md'}")


def cmd_compare(a):
    if not 2 <= len(a.designs) <= 4:
        raise ValueError("compare takes 2 to 4 design files")
    cfgs = [cfgmod.load(d) for d in a.designs]
    labels = [Path(d).stem for d in a.designs]
    rows, headers = report.compare_rows([api.analyze(c) for c in cfgs], labels)
    print(); print(report.table(rows, headers))
    if a.output:
        out = _out(a.output)
        plots.front_view([(lab, {ax: cfgmod.axle_fv(c, ax) for ax in api.AXLES}) for lab, c in zip(labels, cfgs)],
                         out / "front_view.png", title="Front-view comparison: " + " vs ".join(labels))
        with open(out / "comparison.csv", "w", newline="", encoding="utf-8") as f:
            csv.writer(f).writerows([headers] + rows)
        names = "\n".join(f"* **{lab}**: {c.get('name', '')} (`{d}`)" for lab, c, d in zip(labels, cfgs, a.designs))
        (out / "comparison.md").write_text(f"# Comparison: {' vs '.join(labels)}\n\n{names}\n\n"
                                           + report.md_table(rows, headers) + "\n\n![Front view](front_view.png)\n",
                                           encoding="utf-8")
        print(f"\nWrote {out / 'front_view.png'}, comparison.csv and comparison.md")


def _lock_specs(specs, axles, lockable=LOCKABLE):
    """``--lock``/``--unlock`` values: comma-separated ``[axle.]name`` -> {axle: names}.
    A name without an axle applies to every axle being optimised."""
    out = {ax: set() for ax in api.AXLES}
    for item in (s.strip() for spec in specs or [] for s in spec.split(",")):
        if not item:
            continue
        ax, _, name = item.rpartition(".")
        if ax and ax not in api.AXLES:
            raise ValueError(f"lock '{item}': the axle is front or rear")
        if name not in lockable:
            raise ValueError(f"lock '{item}': unknown parameter (use {', '.join(lockable)})")
        for x in ((ax,) if ax else axles):
            out[x].add(name)
    return out


def _print_pivots(cfg, axle, r):
    """Before/after table of one axle's pivot (and ball-joint) stage."""
    b, f = r["before"], r["after"]
    rows = [[k, f"{b[k]:+.3f}", f"{f[k]:+.3f}"] for k in ("rc", "rc_heave_slope", "rc_lateral", "fvsa", "camber",
                                                              "camber_gain", "roll_camber", "kpi", "scrub",
                                                              "lca_incl", "uca_incl", "lower_len", "upper_len")]
    P0, bj = cfgmod.axle_fv(cfg, axle), r["ball_joints"]
    moved = abs(bj["upper_bj_x"] - P0["hux"]) > 1e-6 or abs(bj["lower_bj_x"] - P0["hlx"]) > 1e-6
    print(f"\n{axle}: new pivots (VSusp coords) lower ({r['pivots']['lower_x']:.1f}, {r['pivots']['lower_y']:.1f}), "
          f"upper ({r['pivots']['upper_x']:.1f}, {r['pivots']['upper_y']:.1f}); static camber {r['static_camber']:.2f} deg"
          + (f"; ball joints upper {bj['upper_bj_x']:.2f}, lower {bj['lower_bj_x']:.2f} mm inboard of the hub face "
             f"(were {P0['hux']:.2f}, {P0['hlx']:.2f})" if moved else "")
          + (f"; {r['kpi_joint'].split('_')[0]} ball joint placed for KPI" if r.get("kpi_joint") else "")
          + (f"; locked {', '.join(r['locked'])}" if r["locked"] else ""))
    print(report.table(rows, [axle, "before", "after"]))


def _print_steering(r):
    """Before/after table of the front steering stage."""
    b, f, T, v0, v = r["before"], r["after"], r["targets"], r["start"], r["values"]
    tgt = lambda k, fmt: format(T[k], fmt) if T.get(k) is not None else ""
    rows = [["Ackermann (%)", f"{b['ackermann']:.1f}", f"{f['ackermann']:.1f}", tgt("ackermann", ".1f")],
            ["Rack travel per degree (mm/deg)", f"{b['rack_per_deg']:.3f}", f"{f['rack_per_deg']:.3f}", tgt("rack_per_deg", ".3f")]]
    if "steering_ratio" in f:
        rows.append(["Steering ratio (:1)", f"{b['steering_ratio']:.2f}", f"{f['steering_ratio']:.2f}",
                     tgt("steering_ratio", ".2f")])
    rows += [["Outside wheel at full lock (deg)", f"{b['outside_lock']:.1f}", f"{f['outside_lock']:.1f}", ""],
             ["Rack travel for full lock (mm)", f"±{b['rack_for_lock']:.1f}", f"±{f['rack_for_lock']:.1f}", ""],
             ["Arm-tie-rod angle (lock/centre/lock, deg)", " / ".join(f"{x:.0f}" for x in b["angles"]),
              " / ".join(f"{x:.0f}" for x in f["angles"]), f">= {T['arm_tie_rod_angle_min']:g}"],
             ["Tie-rod clearance at lock (mm)", f"{b['clearance']['tie rod']:.1f}", f"{f['clearance']['tie rod']:.1f}",
              f">= {T['tie_rod_clearance_min']:.1f}"],
             ["Bump steer, max over ±30 mm (deg)", f"{b['bump_steer']:.4f}", f"{f['bump_steer']:.4f}",
              f"<= {T['bump_steer_max']:g}"]]
    moved = ", ".join(f"{k} {v0[k]:.1f} -> {v[k]:.1f}" for k in v if abs(v[k] - v0[k]) > 1e-6)
    print(f"\nfront steering: {moved + ' mm' if moved else 'nothing moved'}"
          + (f"; locked {', '.join(r['locked'])}" if r["locked"] else ""))
    print("  (tie_rod_in/down: outer tie rod inboard of the hub face / below the hub; tie_rod_x, rack_x: ahead of the axle)")
    print(report.table(rows, ["front steering", "before", "after", "target"]))


def cmd_optimize(a):
    cfg = cfgmod.load(a.design)
    axles = api.AXLES if a.axle == "both" else (a.axle,)
    lock, unlock = _lock_specs(a.lock, axles), _lock_specs(a.unlock, axles)
    for axle in api.AXLES:              # the locks used are saved with the new design
        keep = (set(api.lock_list(cfg, axle)) | lock[axle]) - unlock[axle]
        cfg["optimise"][axle] = dict(cfg["optimise"].get(axle) or {}, lock=[k for k in LOCKABLE if k in keep])
    new, results = api.optimize(cfg, axles, quick=a.quick, seed=a.seed, workers=a.workers,
                                steering_only=a.steering_only)
    for axle, r in results.items():
        if "pivots" in r:
            _print_pivots(cfg, axle, r)
        if "steering" in r:
            _print_steering(r["steering"])
    Path(a.output).parent.mkdir(parents=True, exist_ok=True)
    cfgmod.save(new, a.output, header=f"fsae-susp design file - optimised from {Path(a.design).name}")
    print(f"\nWrote {a.output}\nVSusp link:\n{cfgmod.to_vsusp_link(new)}")
    if a.plot:
        out = _out(Path(a.output).with_suffix("").as_posix() + "_plots")
        plots.front_view([("optimised", {ax: cfgmod.axle_fv(new, ax) for ax in api.AXLES}),
                          ("original", {ax: cfgmod.axle_fv(cfg, ax) for ax in api.AXLES})],
                         out / "optimised_vs_original.png", title="Optimised vs original")
        print(f"Wrote {out / 'optimised_vs_original.png'}")


def cmd_check3d(a):
    cfg = cfgmod.load(a.design)
    res2d = api.analyze(cfg)
    res = api.check3d(cfg, steering=not a.no_steering)
    print()
    print(report.table(report.three_d_rows(res), ["3D", "Front", "Rear"]))
    if res.get("steering"):
        print(); print(report.table(report.steering_rows(res["steering"]), ["Steering", "Value"]))
    if res.get("springs"):
        _print_springs(res["springs"])
    print(); print(report.table(report.adjust_rows(res["adjust"]), ["Adjustments", "Front", "Rear"]))
    print(); print(report.table(report.rules_rows(res["rules"]), report.RULES_HEADERS))
    ws = report.warnings(res2d, res)
    print("\nFlags:" if ws else "\nFlags: none"); [print("  -", w) for w in ws]
    if a.sync:
        for axle in api.AXLES:
            x, y = res[axle]["info"]["vsusp_tie_rod"]
            cfg[axle]["knuckle"]["tie_rod"] = [round(x, 2), round(y, 2)]
            tr = cfg[axle]["three_d"].setdefault("tie_rod", {})
            tr["x"] = round(res[axle]["info"]["otr"][0] - res[axle]["info"]["axle_x"], 2)
            tr.pop("design", None)
            tr["inner"] = dict(zip("xyz", (round(v, 2) for v in res[axle]["info"]["itr"])))
        cfgmod.save(cfg, a.design, header="fsae-susp design file (tie-rod points synced by check3d --sync)")
        print(f"\nUpdated {a.design}: knuckle.tie_rod and three_d.tie_rod now hold the 3D points.")
    if a.output:
        out = _out(a.output)
        api.write_hardpoints_csv(res, out / "hardpoints_3d.csv")
        plots.check3d(res, out / "check3d.png")
        tyre, _ = cfgmod.axle_tyre_wheel(cfg, "front")
        plots.layout(res["corners"]["front"], res["corners"]["rear"], res["front"]["clearance_model"],
                     res["rear"]["clearance_model"], tyre["diameter"], tyre["width"], out / "layout_3d.png",
                     res["front"].get("lock_states"))
        figs = [out / "check3d.png", out / "layout_3d.png"]
        if res.get("springs"):
            plots.springs(res["springs"], tyre, out / "springs.png", title=f"Springs - {cfg.get('name', '')}")
            figs.append(out / "springs.png")
        report.write_markdown(out / "report_3d.md", cfg, res2d, res, figs)
        print(f"\nWrote {out}/hardpoints_3d.csv, " + ", ".join(f.name for f in figs) + ", report_3d.md")


def _print_springs(sp):
    print(); print(report.table(report.spring_rows(sp), ["Springs", "Front", "Rear"]))
    if sp.get("vehicle"):
        print(); print(report.table(report.vehicle_spring_rows(sp["vehicle"]), ["Ride, roll and pitch", "Value"]))
    else:
        print("\n(Add a spring block to both axles for ride frequency, roll gradient, load transfer and pitch.)")


def cmd_springs(a):
    cfg = cfgmod.load(a.design)
    sp = api.springs(cfg)
    if sp is None:
        raise ValueError(f"{a.design} has no front.spring or rear.spring block (see examples/fsae_aero_v2.yaml)")
    print(f"\n{cfg.get('name', '')}")
    _print_springs(sp)
    ws = report.spring_warnings(sp)
    print("\nFlags:" if ws else "\nFlags: none"); [print("  -", w) for w in ws]
    if a.output:
        out = _out(a.output)
        tyre, _ = cfgmod.axle_tyre_wheel(cfg, "front")
        plots.springs(sp, tyre, out / "springs.png", title=f"Springs - {cfg.get('name', '')}")
        report.write_springs_markdown(out / "springs.md", cfg, sp, [out / "springs.png"])
        print(f"\nWrote {out / 'springs.png'} and {out / 'springs.md'}")


def cmd_design_springs(a):
    from .springdesign import LOCKABLE as SPRING_LOCKABLE
    cfg = cfgmod.load(a.design)
    axles = api.AXLES if a.axle == "both" else (a.axle,)
    lock = _lock_specs(a.lock, axles, SPRING_LOCKABLE); unlock = _lock_specs(a.unlock, axles, SPRING_LOCKABLE)
    new, res = api.design_springs(cfg, axles, quick=a.quick, seed=a.seed, workers=a.workers,
                                  construct_only=a.no_optimise, lock=lock, unlock=unlock)
    for axle in axles:
        print(f"\n{axle.capitalize()} spring layout")
        print(report.table(report.layout_rows(res[axle]), report.LAYOUT_HEADERS))
        print(); print(report.table(report.design_target_rows(res[axle]), report.DESIGN_TARGET_HEADERS))
    print(); [print(line) for line in report.rate_lines(res)]
    _print_springs(res["springs"])
    ws = report.design_warnings(res) + report.spring_warnings(res["springs"])
    print("\nFlags:" if ws else "\nFlags: none"); [print("  -", w) for w in ws]
    what = "generated" if a.no_optimise else "optimised"
    Path(a.output).parent.mkdir(parents=True, exist_ok=True)
    cfgmod.save(new, a.output, header=f"fsae-susp design file - spring layout {what} from {Path(a.design).name}")
    print(f"\nWrote {a.output}")
    if a.plot:
        out = _out(Path(a.output).with_suffix("").as_posix() + "_plots")
        tyre, _ = cfgmod.axle_tyre_wheel(new, "front")
        plots.springs(res["springs"], tyre, out / "springs.png", title=f"Springs ({what}) - {new.get('name', '')}")
        print(f"Wrote {out / 'springs.png'}")


def cmd_sensitivity(a):
    cfg = cfgmod.load(a.design)
    axles = api.AXLES if a.axle == "both" else (a.axle,)
    sens = api.sensitivity(cfg, a.tol, a.tol_upright, axles)
    print(f"\n{cfg.get('name', '')}: each hardpoint coordinate moved ±{sens['tol']['UF']:g} mm (chassis), "
          f"±{sens['tol']['UBJ']:g} mm (upright), one at a time.")
    print("'As built' keeps the link lengths; the rest have the links made to fit, so static camber and toe hold.")
    for axle in axles:
        print(f"\n{axle.capitalize()} corner")
        print(report.table(report.sensitivity_rows(sens[axle]), report.SENS_HEADERS))
        print("\nCoordinates that use the most of an allowance on their own:")
        print(report.table(report.influence_rows(sens[axle]), report.INFLUENCE_HEADERS))
    ws = report.sensitivity_warnings(sens)
    print("\nFlags:" if ws else "\nFlags: none"); [print("  -", w) for w in ws]
    if a.output:
        out = _out(a.output)
        api.write_sensitivity_csv(sens, out / "sensitivity.csv")
        plots.sensitivity(sens, out / "sensitivity.png", title=f"Hardpoint sensitivity - {cfg.get('name', '')}")
        report.write_sensitivity_markdown(out / "sensitivity.md", cfg, sens, [out / "sensitivity.png"])
        print(f"\nWrote {out}/sensitivity.csv, sensitivity.png, sensitivity.md")


def cmd_report(a):
    cfg = cfgmod.load(a.design); out = _out(a.output)
    res2d = api.analyze(cfg)
    res = api.check3d(cfg)
    plots.front_view([("", {ax: res2d[ax]["fv"] for ax in api.AXLES})], out / "front_view.png", title=f"Front-view kinematics - {cfg.get('name', '')}")
    plots.check3d(res, out / "check3d.png")
    tyre, _ = cfgmod.axle_tyre_wheel(cfg, "front")
    plots.layout(res["corners"]["front"], res["corners"]["rear"], res["front"]["clearance_model"], res["rear"]["clearance_model"],
                 tyre["diameter"], tyre["width"], out / "layout_3d.png", res["front"].get("lock_states"))
    figs = [out / "front_view.png", out / "check3d.png", out / "layout_3d.png"]
    if res.get("springs"):
        plots.springs(res["springs"], tyre, out / "springs.png", title=f"Springs - {cfg.get('name', '')}")
        figs.append(out / "springs.png")
    api.write_hardpoints_csv(res, out / "hardpoints_3d.csv")
    report.write_markdown(out / "report.md", cfg, res2d, res, figs)
    print(f"Wrote {out / 'report.md'} with figures and hardpoints_3d.csv")


def main(argv=None):
    p = argparse.ArgumentParser(prog="fsae-susp", description="FSAE double-wishbone suspension tools: VSusp-compatible "
                                "front-view analysis, pivot optimiser, 3D check (caster, anti, bump steer, Ackermann, clearance).")
    p.add_argument("--version", action="version", version=f"fsae-susp {__version__}")
    sub = p.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("import-vsusp", help="create a design file from a VSusp link")
    s.add_argument("link", help="the VSusp link in quotes, or a text file containing it")
    s.add_argument("-o", "--output", help="design file to write (default: designs/<VSusp project name>.yaml)")
    s.set_defaults(fn=cmd_import)

    s = sub.add_parser("link", help="print the VSusp link for a design file")
    s.add_argument("design"); s.set_defaults(fn=cmd_link)

    s = sub.add_parser("analyze", help="front-view analysis (roll centres, camber, migration)")
    s.add_argument("design"); s.add_argument("-o", "--output", help="folder for plots and report")
    s.add_argument("--compare", help="second design file to overlay on the plots")
    s.add_argument("--label"); s.add_argument("--compare-label"); s.set_defaults(fn=cmd_analyze)

    s = sub.add_parser("compare", help="front-view metrics of 2-4 designs side by side, with an overlay plot")
    s.add_argument("designs", nargs="+"); s.add_argument("-o", "--output", help="folder for the plot, CSV and Markdown table")
    s.set_defaults(fn=cmd_compare)

    s = sub.add_parser("optimize", aliases=["optimise"],
                       help="optimise chassis pivot positions (and ball joints, if their bounds allow), "
                            "then the front steering for any Ackermann or steering-ratio target")
    s.add_argument("design"); s.add_argument("-o", "--output", required=True, help="new design file to write")
    s.add_argument("--axle", choices=["front", "rear", "both"], default="both")
    s.add_argument("--quick", action="store_true", help="smaller search (about 4x faster)")
    s.add_argument("--seed", type=int, default=0); s.add_argument("--workers", type=int, default=1,
                   help="parallel processes for the search (-1 = all cores)")
    s.add_argument("--plot", action="store_true", help="also plot optimised vs original")
    s.add_argument("--lock", action="append", metavar="[AXLE.]NAME",
                   help=f"keep at the design's current value, as well as optimise.<axle>.lock: {', '.join(LOCKABLE)}. "
                        "Comma-separate or repeat. Without front. or rear. it applies to every axle optimised")
    s.add_argument("--unlock", action="append", metavar="[AXLE.]NAME", help="free a parameter the design file locks")
    s.add_argument("--steering-only", action="store_true",
                   help="only place the front outer tie rod and rack line for the steering targets "
                        "(ackermann, rack_per_deg or steering_ratio); the pivots and upright stay as they are")
    s.set_defaults(fn=cmd_optimize)

    s = sub.add_parser("check3d", help="3D check: caster, anti %%, bump/roll steer, Ackermann, lock, clearance")
    s.add_argument("design"); s.add_argument("-o", "--output", help="folder for CSV, plots and report")
    s.add_argument("--no-steering", action="store_true", help="skip the (slower) steering sweep")
    s.add_argument("--sync", action="store_true", help="write the 3D tie-rod points back into the design file")
    s.set_defaults(fn=cmd_check3d)

    s = sub.add_parser("springs", help="rod, rocker and damper: motion ratio, rates, ride frequency, roll and load transfer")
    s.add_argument("design"); s.add_argument("-o", "--output", help="folder for the plot and Markdown")
    s.set_defaults(fn=cmd_springs)

    s = sub.add_parser("design-springs", aliases=["optimize-springs", "optimise-springs"],
                       help="generate and optimise the rod, rocker and damper, and pick spring and bar rates")
    s.add_argument("design"); s.add_argument("-o", "--output", required=True, help="new design file to write")
    s.add_argument("--axle", choices=["front", "rear", "both"], default="both")
    s.add_argument("--quick", action="store_true", help="smaller search (about 4x faster)")
    s.add_argument("--no-optimise", "--no-optimize", dest="no_optimise", action="store_true",
                   help="write the generated starting layout without searching")
    s.add_argument("--seed", type=int, default=0)
    s.add_argument("--workers", type=int, default=1, help="parallel processes for the search (-1 = all cores)")
    s.add_argument("--plot", action="store_true", help="also write springs.png next to the new design")
    s.add_argument("--lock", action="append", metavar="[AXLE.]NAME",
                   help="keep the spring block's current value: at, pivot_x, pivot_y, pivot_z, rod_lever, "
                        "damper_lever (as well as spring_design.<axle>.lock)")
    s.add_argument("--unlock", action="append", metavar="[AXLE.]NAME", help="free a parameter the design file locks")
    s.set_defaults(fn=cmd_design_springs)

    s = sub.add_parser("sensitivity", help="how far each hardpoint's build tolerance moves the kinematics (3D)")
    s.add_argument("design"); s.add_argument("-o", "--output", help="folder for CSV, plot and Markdown")
    s.add_argument("--tol", type=float, help="tolerance of every hardpoint coordinate, mm "
                   "(default: sensitivity.tol in the design file, else 1)")
    s.add_argument("--tol-upright", type=float, help="tolerance of the upright points (ball joints, outer tie rod), "
                   "mm (default: sensitivity.tol_upright, else --tol)")
    s.add_argument("--axle", choices=["front", "rear", "both"], default="both")
    s.set_defaults(fn=cmd_sensitivity)

    s = sub.add_parser("report", help="front view + 3D check + all plots and a Markdown report")
    s.add_argument("design"); s.add_argument("-o", "--output", default="report"); s.set_defaults(fn=cmd_report)

    a = p.parse_args(argv)
    try:
        a.fn(a)
    except (ValueError, KeyError, FileNotFoundError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
