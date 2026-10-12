"""Command-line interface: ``fsae-susp <command> ...`` (or ``python -m fsaesusp``)."""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

from . import __version__
from . import config as cfgmod
from . import api, plots, report
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


def cmd_optimize(a):
    cfg = cfgmod.load(a.design)
    axles = api.AXLES if a.axle == "both" else (a.axle,)
    new, results = api.optimize(cfg, axles, quick=a.quick, seed=a.seed, workers=a.workers)
    for axle, r in results.items():
        b, f = r["before"], r["after"]
        rows = [[k, f"{b[k]:+.3f}", f"{f[k]:+.3f}"] for k in ("rc", "rc_heave_slope", "rc_lateral", "fvsa", "camber",
                                                                  "camber_gain", "roll_camber", "lca_incl", "uca_incl",
                                                                  "lower_len", "upper_len")]
        print(f"\n{axle}: new pivots (VSusp coords) lower ({r['pivots']['lower_x']:.1f}, {r['pivots']['lower_y']:.1f}), "
              f"upper ({r['pivots']['upper_x']:.1f}, {r['pivots']['upper_y']:.1f}); static camber {r['static_camber']:.2f} deg")
        print(report.table(rows, [axle, "before", "after"]))
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
        report.write_markdown(out / "report_3d.md", cfg, res2d, res, [out / "check3d.png", out / "layout_3d.png"])
        print(f"\nWrote {out}/hardpoints_3d.csv, check3d.png, layout_3d.png, report_3d.md")


def cmd_report(a):
    cfg = cfgmod.load(a.design); out = _out(a.output)
    res2d = api.analyze(cfg)
    res = api.check3d(cfg)
    plots.front_view([("", {ax: res2d[ax]["fv"] for ax in api.AXLES})], out / "front_view.png", title=f"Front-view kinematics - {cfg.get('name', '')}")
    plots.check3d(res, out / "check3d.png")
    tyre, _ = cfgmod.axle_tyre_wheel(cfg, "front")
    plots.layout(res["corners"]["front"], res["corners"]["rear"], res["front"]["clearance_model"], res["rear"]["clearance_model"],
                 tyre["diameter"], tyre["width"], out / "layout_3d.png", res["front"].get("lock_states"))
    api.write_hardpoints_csv(res, out / "hardpoints_3d.csv")
    report.write_markdown(out / "report.md", cfg, res2d, res, [out / "front_view.png", out / "check3d.png", out / "layout_3d.png"])
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

    s = sub.add_parser("optimize", aliases=["optimise"], help="optimise chassis pivot positions")
    s.add_argument("design"); s.add_argument("-o", "--output", required=True, help="new design file to write")
    s.add_argument("--axle", choices=["front", "rear", "both"], default="both")
    s.add_argument("--quick", action="store_true", help="smaller search (about 4x faster)")
    s.add_argument("--seed", type=int, default=0); s.add_argument("--workers", type=int, default=1,
                   help="parallel processes for the search (-1 = all cores)")
    s.add_argument("--plot", action="store_true", help="also plot optimised vs original")
    s.set_defaults(fn=cmd_optimize)

    s = sub.add_parser("check3d", help="3D check: caster, anti %%, bump/roll steer, Ackermann, lock, clearance")
    s.add_argument("design"); s.add_argument("-o", "--output", help="folder for CSV, plots and report")
    s.add_argument("--no-steering", action="store_true", help="skip the (slower) steering sweep")
    s.add_argument("--sync", action="store_true", help="write the 3D tie-rod points back into the design file")
    s.set_defaults(fn=cmd_check3d)

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
