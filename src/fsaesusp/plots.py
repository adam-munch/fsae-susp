"""Charts (matplotlib, PNG)."""
from __future__ import annotations

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

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
