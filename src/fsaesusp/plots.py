"""Charts (matplotlib, PNG)."""
from __future__ import annotations

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.lines
import matplotlib.ticker

from . import analysis2d
from .clearance import node_position

SURF, INK, INK2, MUTED, GRID, AXIS, BAND = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7", "#f0efec"
BLUE, ORANGE = "#2a78d6", "#eb6834"
AXLE_COL = {"front": BLUE, "rear": ORANGE}


def _style():
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10, "axes.edgecolor": AXIS, "axes.labelcolor": INK2,
                         "xtick.color": MUTED, "ytick.color": MUTED, "axes.facecolor": SURF, "figure.facecolor": SURF,
                         "axes.titlesize": 11, "axes.titleweight": "bold", "axes.titlecolor": INK,
                         "axes.titlelocation": "left", "axes.titlepad": 10})


def _axes(ax, aspect=False):
    ax.grid(True, color=GRID, lw=0.8); ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.tick_params(length=0)
    if aspect:
        ax.set_aspect("equal")


def front_view(designs, out_path, title="Front-view kinematics", roll_max=2.5, travel=30.0, heave=20.0):
    """designs: list of up to four (label, {"front": FV, "rear": FV}); solid, dashed, dotted, dash-dot."""
    _style()
    fig, axs = plt.subplots(2, 2, figsize=(11.5, 8.4))
    for ax in axs.flat:
        _axes(ax)
    styles = [dict(lw=2.2, ls="-", alpha=1.0), dict(lw=1.5, ls=(0, (4, 3)), alpha=0.6),
              dict(lw=1.6, ls=(0, (1, 1.5)), alpha=0.8), dict(lw=1.5, ls=(0, (6, 2, 1, 2)), alpha=0.7)]
    z = np.linspace(-travel, travel, 41); ph = np.linspace(0, roll_max, 21); hv = np.linspace(-heave, heave, 33)
    for i, (label, d) in enumerate(designs[:len(styles)]):
        st = styles[i]
        for axle in ("front", "rear"):
            P = d[axle]; col = AXLE_COL[axle]; lab = f"{axle.capitalize()} {label}".strip()
            tr = analysis2d.travel_sweep(P, z)
            axs[0, 0].plot(z, tr["camber"], color=col, label=lab, **st)
            rs = analysis2d.roll_sweep(P, ph)
            axs[0, 1].plot(ph, rs["camber_out"], color=col, label=lab, **st)
            axs[1, 1].plot(ph, rs["rc_x"], color=col, label=lab, **st)
            hs = analysis2d.heave_sweep(P, hv)
            axs[1, 0].plot(hv, hs["rc_height"], color=col, label=lab, **st)
    axs[0, 0].set_title("Camber vs single-wheel travel"); axs[0, 0].set_xlabel("Wheel travel, mm  (– droop | + bump)"); axs[0, 0].set_ylabel("Camber, deg")
    axs[0, 1].set_title("Outside-wheel camber vs body roll"); axs[0, 1].set_xlabel("Body roll, deg"); axs[0, 1].set_ylabel("Camber vs ground, deg")
    axs[0, 1].axhline(0, color=INK2, lw=0.9)
    axs[1, 0].set_title("Roll-centre height vs heave"); axs[1, 0].set_xlabel("Heave, mm  (– rebound | + compression)"); axs[1, 0].set_ylabel("RC height above ground, mm")
    axs[1, 0].axhline(0, color=INK2, lw=0.9)
    axs[1, 1].set_title("Roll-centre lateral migration vs roll"); axs[1, 1].set_xlabel("Body roll, deg"); axs[1, 1].set_ylabel("RC offset from centreline, mm")
    h, l = axs[0, 0].get_legend_handles_labels()
    fig.legend(h, l, loc="upper right", ncol=4, frameon=False, labelcolor=INK2, bbox_to_anchor=(0.99, 0.985))
    fig.suptitle(title, x=0.01, ha="left", fontsize=12.5, fontweight="bold", color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(out_path, dpi=150); plt.close(fig)


def check3d(res, out_path, title="3D check: steering, clearance and bump steer"):
    _style()
    fig, axs = plt.subplots(2, 2, figsize=(11.5, 8.4))
    for ax in axs.flat:
        _axes(ax)
    st = res.get("steering")
    tw, wb = res["track_front"], res["wheelbase"]
    if st:
        c = st["curve"]; ax = axs[0, 0]
        di = np.array(c["inside"]); do = np.array(c["outside"])
        x = np.linspace(0.01, max(di) + 0.5, 100)
        ax.plot(x, x, color=MUTED, lw=1.2, ls=(0, (2, 2)), label="parallel (0%)")
        ax.plot(x, np.degrees(np.arctan(1 / (1 / np.tan(np.radians(x)) + tw / wb))), color=MUTED, lw=1.2, ls=(0, (5, 3)), label="100% Ackermann")
        ax.plot(di, do, color=BLUE, lw=2.2, label=f"design ({st['ackermann_percent']:.0f}% at {st['inside_at_ack']:.0f}°)")
        ax.set_title("Ackermann: outside vs inside wheel"); ax.set_xlabel("Inside wheel steer, deg"); ax.set_ylabel("Outside wheel steer, deg")
        ax.legend(frameon=False, loc="upper left", labelcolor=INK2, fontsize=9)
        ax = axs[0, 1]
        ax.plot(c["st_in"], c["cam_in"], color=BLUE, lw=2.2, label="inside wheel")
        ax.plot(c["st_out"], c["cam_out"], color=ORANGE, lw=2.2, label="outside wheel")
        ax.axhline(0, color=INK2, lw=0.9)
        ax.set_title("Camber vs steer angle"); ax.set_xlabel("Steer angle of that wheel, deg"); ax.set_ylabel("Camber vs ground, deg")
        ax.legend(frameon=False, loc="center left", labelcolor=INK2)
        if "clearance_rows" in st:
            ax = axs[1, 0]
            for z, rows in st["clearance_rows"].items():
                ls = "-" if z == 0 else ((0, (4, 3)) if z > 0 else (0, (1, 2)))
                s_ = [r["steer"] for r in rows]
                ax.plot(s_, [r["lower arm"] for r in rows], color=BLUE, lw=1.8, ls=ls, label=f"lower arm, {z:+.0f} mm")
                ax.plot(s_, [r["tie rod"] for r in rows], color=ORANGE, lw=1.8, ls=ls, label=f"tie rod, {z:+.0f} mm")
            ax.axhspan(-50, 3, color=BAND, zorder=0)
            ymax = max(5.0, min(60.0, max(max(max(r["lower arm"], r["tie rod"]) for r in rows) for rows in st["clearance_rows"].values())))
            ax.set_ylim(min(-2.0, min(min(min(r["lower arm"], r["tie rod"]) for r in rows) for rows in st["clearance_rows"].values()) - 2), ymax + 3)
            ax.set_title("Rim/tyre clearance vs steer (front-left)"); ax.set_xlabel("Left-wheel steer, deg (+ = inside wheel of a left turn)"); ax.set_ylabel("Clearance, mm")
            ax.legend(frameon=False, loc="center left", bbox_to_anchor=(0.02, 0.62), ncol=2, labelcolor=INK2, fontsize=8)
    else:
        for ax in (axs[0, 0], axs[0, 1], axs[1, 0]):
            ax.text(0.5, 0.5, "no steering on this design", ha="center", va="center", color=MUTED, transform=ax.transAxes)
    ax = axs[1, 1]
    for axle in ("front", "rear"):
        b = res[axle]["bump"]
        ax.plot(b["z"], b["toe_in"], color=AXLE_COL[axle], lw=2.2, label=f"{axle} (3D)")
    ax.set_title("Bump steer (toe-in vs wheel travel)"); ax.set_xlabel("Wheel travel, mm"); ax.set_ylabel("Toe-in, deg")
    ax.legend(frameon=False, loc="upper center", labelcolor=INK2)
    fig.suptitle(title, x=0.01, ha="left", fontsize=12.5, fontweight="bold", color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(out_path, dpi=150); plt.close(fig)


SENS_SHORT = dict(rc_height=("RC height", "mm"), rc_heave_slope=("RC per mm heave", ""),
                  camber_gain=("Camber gain", "°/mm"), toe_bump=("Toe at +25 mm", "°"),
                  toe_droop=("Toe at −25 mm", "°"), roll_steer=("Roll steer", "°/°"), anti=("Anti", "%"),
                  caster=("Caster", "°"), trail=("Trail", "mm"), kpi=("KPI", "°"), scrub=("Scrub", "mm"),
                  ackermann=("Ackermann", "%"))
SEQ_BLUE = ("#f4f8fd", "#cde2fb", "#9ec5f4", "#6da7ec", "#3987e5", "#256abf", "#184f95", "#0d366b")


def sensitivity(sens, out_path, title="Hardpoint sensitivity"):
    """Heat map per axle: the fraction of each kinematic metric's allowance that each coordinate's tolerance
    uses on its own (colour capped at 100 %)."""
    from matplotlib.colors import LinearSegmentedColormap
    _style()
    axles = [a for a in ("front", "rear") if a in sens]
    cmap = LinearSegmentedColormap.from_list("seq", SEQ_BLUE)
    fig = plt.figure(figsize=(7.0 * len(axles) + 1.0, 9.6))
    gs = fig.add_gridspec(1, len(axles) + 1, width_ratios=[1] * len(axles) + [0.035], wspace=0.22,
                          left=0.065, right=0.95, top=0.8, bottom=0.03)
    im = None
    for n, axle in enumerate(axles):
        ax = fig.add_subplot(gs[0, n])
        res = sens[axle]; rows = res["rows"]; keys = [k for k in res["metrics"] if k in rows[0]["used"]]
        U = np.array([[np.nan_to_num(r["used"][k]) for k in keys] for r in rows])
        im = ax.imshow(np.clip(U, 0, 1), cmap=cmap, vmin=0, vmax=1, aspect="auto")
        ax.set_xticks(range(len(keys)))
        ax.set_xticklabels([f"{SENS_SHORT[k][0]}  ±{res['stack'][k]['allow']:g}{SENS_SHORT[k][1]}" for k in keys],
                           rotation=40, ha="left", rotation_mode="anchor", fontsize=8.5, color=INK2)
        ax.xaxis.tick_top()
        ax.set_yticks(range(len(rows)))
        ax.set_yticklabels([f"{r['point']} {r['axis']}  ±{r['tol']:g}" for r in rows], fontsize=8.5, color=INK2)
        ax.tick_params(length=0)
        for s in ax.spines.values():
            s.set_visible(False)
        # 2px surface gap between cells, and a wider one between hardpoints
        ax.set_xticks(np.arange(-.5, len(keys)), minor=True); ax.set_yticks(np.arange(-.5, len(rows)), minor=True)
        ax.tick_params(which="minor", length=0)
        ax.grid(which="minor", color=SURF, lw=1.5); ax.grid(which="major", visible=False)
        for y in np.arange(2.5, len(rows) - 1, 3):
            ax.axhline(y, color=SURF, lw=4)
        for i in range(len(rows)):
            for j in range(len(keys)):
                u = U[i, j]
                if u >= 0.1:
                    ax.text(j, i, f"{u:.0%}" if u < 10 else ">999%", ha="center", va="center", fontsize=7.5,
                            color="white" if u > 0.45 else INK)
        ax.set_title(f"{axle.capitalize()} corner", pad=92)
    cax = fig.add_subplot(gs[0, -1])
    cb = fig.colorbar(im, cax=cax)
    cb.outline.set_visible(False); cb.ax.tick_params(length=0, colors=MUTED)
    cb.set_label("Share of the metric's allowance used by this coordinate alone", color=INK2)
    cb.ax.yaxis.set_major_formatter(matplotlib.ticker.PercentFormatter(1.0, decimals=0))
    fig.suptitle(title, x=0.01, y=0.985, ha="left", fontsize=12.5, fontweight="bold", color=INK)
    fig.text(0.01, 0.955, "Each hardpoint coordinate moved by its tolerance (mm), one at a time, links made to fit. "
             "Columns show each metric's allowance. One corner; cells labelled at 10 % and over.",
             ha="left", fontsize=9.5, color=INK2)
    fig.savefig(out_path, dpi=150); plt.close(fig)


def springs(sp, tyre, out_path, title="Springs: rod, rocker and damper"):
    """Motion ratio and damper length over travel, and a front-view sketch of each corner's linkage
    (static solid, +25 mm bump dashed, -25 mm droop dotted)."""
    _style()
    A = sp["axles"]; axles = [a for a in ("front", "rear") if a in A]
    fig, axs = plt.subplots(2, 2, figsize=(11.5, 9.2))
    for ax in axs[0]:
        _axes(ax)
    ax = axs[0, 0]
    for a in axles:
        c = A[a]["curve"]; ax.plot(c["z"], c["mr"], color=AXLE_COL[a], lw=2.2, label=a)
    ax.set_title("Motion ratio vs wheel travel"); ax.set_xlabel("Wheel travel, mm  (– droop | + bump)")
    ax.set_ylabel("Damper mm per wheel mm"); ax.legend(frameon=False, labelcolor=INK2)
    ax = axs[0, 1]
    for a in axles:
        c = A[a]["curve"]; ax.plot(c["z"], c["damper"], color=AXLE_COL[a], lw=2.2, label=a)
        if "damper_limits" in A[a]:
            for v in A[a]["damper_limits"]:
                ax.axhline(v, color=AXLE_COL[a], lw=1.0, ls=(0, (4, 3)))
    h, l = ax.get_legend_handles_labels()
    if any("damper_limits" in A[a] for a in axles):
        h.append(matplotlib.lines.Line2D([], [], color=INK2, lw=1.0, ls=(0, (4, 3)))); l.append("damper fully compressed / extended")
    ax.set_title("Damper length vs wheel travel"); ax.set_xlabel("Wheel travel, mm  (– droop | + bump)")
    ax.set_ylabel("Eye to eye, mm")
    ax.legend(h, l, frameon=False, labelcolor=INK2, loc="upper right", bbox_to_anchor=(1.0, 0.92))
    for ax, a in zip(axs[1], ("front", "rear")):
        _axes(ax, aspect=True)
        if a not in A:
            ax.text(0.5, 0.5, f"no spring block on the {a} axle", ha="center", va="center", color=MUTED,
                    transform=ax.transAxes); ax.set_axis_off(); continue
        lay = A[a]["layout"]
        for z, ls, lw in ((0.0, "-", 2.2), (25.0, (0, (4, 3)), 1.2), (-25.0, (0, (1, 2)), 1.2)):
            try:
                st = lay.state(z) if z == 0 else _state_from_static(lay, z)
            except Exception:
                continue
            s = st["s"]; u, ch = s["pts"], s["chassis"]
            yz = lambda p: (p[1], p[2])
            for f, r, bj, col in (("UF", "UR", "UBJ", ORANGE), ("LF", "LR", "LBJ", BLUE)):
                piv = 0.5 * (ch[f] + ch[r])
                ax.plot(*zip(yz(piv), yz(u[bj])), color=col, lw=lw, ls=ls)
            ax.plot(*zip(yz(u["LBJ"]), yz(u["UBJ"])), color=INK2, lw=lw, ls=ls)
            ax.plot(*zip(yz(st["P"]), yz(st["Q"])), color=INK, lw=lw, ls=ls)
            ax.plot(*zip(yz(lay.C), yz(st["Q"]), yz(st["D"]), yz(lay.C)), color=INK2, lw=lw, ls=ls)
            ax.plot(*zip(yz(st["D"]), yz(lay.Dc)), color=MUTED, lw=lw + 2.5, ls=ls, solid_capstyle="butt")
            if z == 0:
                W = u["WC"]; R = tyre["diameter"] / 2; hw = tyre["width"] / 2
                ax.plot([W[1] - hw, W[1] + hw, W[1] + hw, W[1] - hw, W[1] - hw], [W[2] - R, W[2] - R, W[2] + R, W[2] + R, W[2] - R],
                        color=MUTED, lw=1.0)
                for p in (lay.C, lay.Dc, st["P"]):
                    ax.plot(*yz(p), "o", ms=5, color=INK, mec=SURF, mew=1.5)
                ax.annotate("rocker", yz(lay.C), xytext=(6, -12), textcoords="offset points", color=INK2, fontsize=8.5)
                ax.annotate("damper", yz(0.5 * (st["D"] + lay.Dc)), xytext=(0, 8), textcoords="offset points",
                            color=INK2, fontsize=8.5, ha="center")
                ax.annotate("rod", yz(0.5 * (st["P"] + st["Q"])), xytext=(6, 0), textcoords="offset points",
                            color=INK2, fontsize=8.5)
        lay.c.reset(); lay._phi = 0.0
        ax.axhline(0, color=INK2, lw=0.9)
        ax.set_title(f"{a.capitalize()}-left corner, front view (looking rearward)")
        ax.set_xlabel("y, mm from centreline"); ax.set_ylabel("z, mm")
        ax.set_xlim(-10, A[a]["layout"].c.hp["WC"][1] + tyre["width"] / 2 + 30); ax.set_ylim(-20, tyre["diameter"] + 70)
        ax.text(0.01, 0.025, "static solid, +25 mm bump dashed,\n−25 mm droop dotted;\nupper arm orange, lower arm blue",
                transform=ax.transAxes, ha="left", va="bottom", fontsize=8.5, color=INK2)
    fig.suptitle(title, x=0.01, ha="left", fontsize=12.5, fontweight="bold", color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.95), h_pad=3.0)
    fig.savefig(out_path, dpi=150); plt.close(fig)


def _state_from_static(lay, z, step=5.0):
    lay.c.reset(); lay._phi = 0.0
    st = None
    for zz in np.linspace(0, z, int(np.ceil(abs(z) / step)) + 1)[1:]:
        st = lay.state(zz)
    return st


def layout(cF, cR, clrF, clrR, tyre_d, tyre_w, out_path, lock_states=None):
    """Top view of the front-left corner and side views of both left corners."""
    _style()
    fig, axs = plt.subplots(1, 3, figsize=(15, 6.2), gridspec_kw=dict(width_ratios=[1.05, 1, 1]))
    for ax in axs:
        _axes(ax, aspect=True)

    def rect(s):
        WC, a = s["pts"]["WC"], s["a"]; h = np.cross(a, [0, 0, 1.0])
        a2 = a[:2] / np.linalg.norm(a[:2]); h2 = h[:2] / np.linalg.norm(h[:2]); c = WC[:2]
        return np.array([c + sx * tyre_d / 2 * h2 + sy * tyre_w / 2 * a2 for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1), (1, 1))])

    def arms_top(ax, s, clr):
        u, ch = s["pts"], s["chassis"]; nd = clr.nodes(s)
        for arm, bj, f, r, col in (("UCA", "UBJ", "UF", "UR", ORANGE), ("LCA", "LBJ", "LF", "LR", BLUE)):
            for p in (ch[f], ch[r]):
                ax.plot([nd[arm][1], p[1]], [nd[arm][0], p[0]], color=col, lw=1.8)
            ax.plot([u[bj][1], nd[arm][1]], [u[bj][0], nd[arm][0]], color=col, lw=2.4)
            ax.plot([ch[f][1], ch[r][1]], [ch[f][0], ch[r][0]], "o", ms=5, color=col, mec=SURF, mew=1.5)

    ax = axs[0]
    s0 = cF.reset()
    for s, ls in (lock_states or []):
        R = rect(s); ax.plot(R[:, 1], R[:, 0], color=MUTED, lw=1.0, ls=ls)
    cF.reset(); s0 = cF.reset()
    R = rect(s0); ax.plot(R[:, 1], R[:, 0], color=INK2, lw=1.3)
    arms_top(ax, s0, clrF)
    u, ch = s0["pts"], s0["chassis"]
    ax.plot([u["OTR"][1], ch["ITR"][1]], [u["OTR"][0], ch["ITR"][0]], color=INK, lw=2.0)
    for p, lab in ((u["UBJ"], "UBJ"), (u["LBJ"], "LBJ"), (u["OTR"], "outer tie rod"), (ch["ITR"], "inner tie rod")):
        ax.plot(p[1], p[0], "o", ms=5, color=INK, mec=SURF, mew=1.5)
        ax.annotate(lab, (p[1], p[0]), xytext=(5, -12), textcoords="offset points", color=INK2, fontsize=8.5)
    if lock_states:
        ax.text(0.98, 0.02, "wheel at full lock: dashed = inside,\ndotted = outside", transform=ax.transAxes,
                ha="right", va="bottom", color=INK2, fontsize=8.5)
    ax.set_title("Front-left corner, top view"); ax.set_xlabel("y, mm from centreline"); ax.set_ylabel("x, mm (forward)")
    yc = u["WC"][1]; ax.set_xlim(150, yc + 260); ax.set_ylim(-330, 330)

    def side(ax, c, title):
        s = c.reset(); u, ch = s["pts"], s["chassis"]; WC = u["WC"]; th = np.linspace(0, 2 * np.pi, 200)
        ax.plot(WC[0] + tyre_d / 2 * np.cos(th), WC[2] + tyre_d / 2 * np.sin(th), color=MUTED, lw=1.0)
        ax.axhline(0, color=INK2, lw=0.9)
        for f, r, col in (("UF", "UR", ORANGE), ("LF", "LR", BLUE)):
            p, q = ch[f], ch[r]; d = (q - p) / np.linalg.norm(q - p); e = np.array([p - d * 250, q + d * 250])
            ax.plot(e[:, 0], e[:, 2], color=col, lw=1.0, ls=(0, (4, 3)))
            ax.plot([p[0], q[0]], [p[2], q[2]], color=col, lw=2.2)
        k = s["k"]; L = u["LBJ"]; t0 = -L[2] / k[2]; t1 = (u["UBJ"][2] + 80 - L[2]) / k[2]
        ax.plot([L[0] + t0 * k[0], L[0] + t1 * k[0]], [0, L[2] + t1 * k[2]], color=INK, lw=1.4)
        for b in ("UBJ", "LBJ", "OTR"):
            ax.plot(u[b][0], u[b][2], "o", ms=5, color=INK, mec=SURF, mew=1.5)
        ax.set_title(title); ax.set_xlabel("x, mm (forward)"); ax.set_ylabel("z, mm")
        ax.set_xlim(WC[0] - 330, WC[0] + 330); ax.set_ylim(-20, tyre_d + 60)
    side(axs[1], cF, "Front-left corner, side view")
    side(axs[2], cR, "Rear-left corner, side view")
    fig.suptitle("3D layout (left side; right side is the mirror image)", x=0.01, ha="left", fontsize=12.5, fontweight="bold", color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(out_path, dpi=150); plt.close(fig)
