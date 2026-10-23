"""Generate all figures used by the Chapter 5 notes."""

import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Arc
import numpy as np
from matplotlib.patches import Circle, FancyArrowPatch, Polygon, Rectangle

HERE = os.path.dirname(os.path.abspath(__file__))
FIG = os.path.join(HERE, "figures")
os.makedirs(FIG, exist_ok=True)

plt.rcParams.update(
    {
        "font.size": 9,
        "axes.grid": True,
        "grid.alpha": 0.25,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "figure.dpi": 140,
        "savefig.dpi": 180,
        "savefig.bbox": "tight",
    }
)


# ---------------------------------------------------------------------------
# 1. ZMP concept
# ---------------------------------------------------------------------------
def fig_zmp_concept():
    fig, axes = plt.subplots(1, 2, figsize=(9.6, 3.6))

    # (a) multi-contact flat foot
    ax = axes[0]
    ax.set_title("(a) Flat terrain: CoP = ZMP")
    ax.set_aspect("equal")
    ax.set_xlim(-1.2, 2.0)
    ax.set_ylim(-0.15, 1.75)
    ax.axhline(0, color="0.3", lw=1)
    ax.fill_between([-0.9, 1.7], 0, 0.04, color="0.85", zorder=0)
    # support polygon
    ax.add_patch(Polygon([[-0.75, 0], [1.55, 0], [1.35, 0.12], [-0.55, 0.12]],
                         closed=True, color="#cfe3f3", alpha=0.8, zorder=1))
    # feet
    for x0 in (-0.75, 0.85):
        ax.add_patch(Rectangle((x0, 0), 0.45, 0.1, fc="#e8b04b",
                               ec="k", alpha=0.85, zorder=2))
    # body
    ax.add_patch(Circle((0.35, 1.25), 0.16, fc="#7fa8c9", ec="k", zorder=4))
    ax.plot([0.1, 0.3], [0.1, 1.1], color="k", lw=2)
    ax.plot([0.7, 0.42], [0.1, 1.1], color="k", lw=2)
    # CoM projection and ZMP
    ax.plot([0.35, 0.35], [1.25, 0], "k--", lw=0.9)
    ax.plot(0.35, 0, "ko", ms=6)
    ax.annotate("CoM projection", (0.35, 0.28), xytext=(0.5, 0.6),
                arrowprops=dict(arrowstyle="->", lw=0.8))
    ax.plot(0.5, 0, "r*", ms=15, zorder=6)
    ax.annotate("ZMP / CoP", (0.5, 0), xytext=(0.85, -0.11),
                color="r", arrowprops=dict(arrowstyle="->", color="r", lw=0.8))
    # ground reaction
    ax.add_patch(FancyArrowPatch((0.5, 0), (0.5, 0.55), arrowstyle="-|>",
                                 mutation_scale=14, color="g", lw=1.6))
    ax.text(0.53, 0.35, r"$f^{GRF}$", color="g")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("z (m)")

    # (b) LIPM
    ax = axes[1]
    ax.set_title("(b) LIPM: the ZMP is the input")
    ax.set_aspect("equal")
    ax.set_xlim(-1.0, 1.8)
    ax.set_ylim(-0.15, 1.65)
    ax.axhline(0, color="0.3", lw=1)
    ax.plot([0.55, 0.55], [0, 1.25], color="k", ls="--", lw=0.8)
    ax.plot([0.9, 0.55], [0, 1.25], color="k", lw=2.2)  # leg
    ax.add_patch(Circle((0.9, 1.25), 0.14, fc="#7fa8c9", ec="k", zorder=4))
    ax.plot(0.55, 0, "r*", ms=15, zorder=6)
    ax.text(0.58, -0.11, r"$p=x_{cp}$", color="r")
    ax.plot(0.9, 1.25, "ko", ms=4)
    ax.annotate(r"$x$", (0.9, 1.25), xytext=(0.98, 1.4))
    ax.annotate(r"$z$", (0.55, 0.62), xytext=(0.33, 0.62))
    ax.annotate("", xy=(0.55, 0.6), xytext=(0.55, 0.0),
                arrowprops=dict(arrowstyle="<->", lw=0.8))
    ax.annotate("", xy=(0.55, 1.25), xytext=(0.9, 1.25),
                arrowprops=dict(arrowstyle="<->", lw=0.8))
    ax.text(0.68, 1.32, r"$x-p$")
    ax.add_patch(FancyArrowPatch((0.9, 1.25), (1.1, 0.85), arrowstyle="-|>",
                                 mutation_scale=12, color="k", lw=1.2))
    ax.text(1.02, 0.95, r"$m g$")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("z (m)")

    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig5_1_zmp_concept.png"))
    plt.close(fig)


# ---------------------------------------------------------------------------
# 2. LIPM phase portrait and DCM
# ---------------------------------------------------------------------------
def fig_lipm_dcm():
    g, h = 9.81, 1.0
    w = np.sqrt(g / h)
    p = 0.0

    fig, axes = plt.subplots(1, 2, figsize=(9.6, 3.8))

    # (a) phase portrait
    ax = axes[0]
    ax.set_title(r"(a) LIPM phase portrait ($\dot x=\omega(\xi-x)$)")
    xs = np.linspace(-1.2, 1.2, 21)
    vs = np.linspace(-2.2, 2.2, 25)
    X, V = np.meshgrid(xs, vs)
    U = V
    W = w**2 * (X - p)
    speed = np.hypot(U, W)
    ax.streamplot(X, V, U / (speed + 1e-9), W / (speed + 1e-9),
                  color="0.6", linewidth=0.6, density=1.1, arrowsize=0.7)
    for xi in [-1.0, -0.5, 0.5, 1.0]:
        x_traj = np.linspace(-1.2, 1.2, 400)
        ax.plot(x_traj, w * (xi - x_traj), color="#c44e52", lw=1.0, alpha=0.7)
    ax.plot(p, 0, "ko", ms=6)
    ax.annotate("unstable\nfixed point", (p, 0), xytext=(0.1, 0.9))
    ax.set_xlabel(r"$x-p$ (m)")
    ax.set_ylabel(r"$\dot x/\omega$ (m)")

    # (b) constant ZMP response and DCM
    ax = axes[1]
    ax.set_title("(b) Constant ZMP: CoM vs. DCM")
    t = np.linspace(0, 1.5, 300)
    for x0, v0, c in [(0.0, 0.8, "#4c72b0"), (0.0, -0.4, "#dd8452")]:
        x = x0 * np.cosh(w * t) + (v0 / w) * np.sinh(w * t)
        v = x0 * w * np.sinh(w * t) + v0 * np.cosh(w * t)
        xi = x + v / w
        ax.plot(t, x, color=c, lw=1.4, label=fr"$x(t)$, $v_0={v0}$")
        ax.plot(t, xi, color=c, lw=1.0, ls="--",
                label=fr"$\xi(t)$, $\xi_0={x0 + v0 / w:.2f}$")
    ax.axhline(p, color="0.3", lw=0.8)
    ax.text(0.02, 0.05, r"$p$ (ZMP)", color="0.3")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("position (m)")
    ax.legend(fontsize=7, frameon=False)

    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig5_2_lipm_dcm.png"))
    plt.close(fig)


# ---------------------------------------------------------------------------
# 3. ZMP planner
# ---------------------------------------------------------------------------
def fig_zmp_planner():
    d = np.load(os.path.join(HERE, "ex51_zmp_planner_data.npz"))
    footsteps = d["footsteps"]
    t, com, comd, zmp = d["t"], d["com"], d["comd"], d["zmp"]
    t_sim, x_nom, x_pert = d["t_sim"], d["x_nom"], d["x_pert"]

    fig, axes = plt.subplots(2, 2, figsize=(9.8, 7.0))

    # (a) top view
    ax = axes[0, 0]
    ax.set_title("(a) Example 5.1: footstep plan (top view)")
    for x0, y0 in footsteps:
        ax.add_patch(Rectangle((x0 - 0.15, y0 - 0.05), 0.3, 0.1,
                               fc="0.85", ec="0.5"))
    ax.plot(zmp[:, 0], zmp[:, 1], "r", lw=1.4, label="desired ZMP")
    ax.plot(com[:, 0], com[:, 1], "b", lw=1.4, label="nominal CoM")
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.legend(fontsize=7, frameon=False, loc="upper left")
    ax.set_ylim(-0.28, 0.28)

    # (b) timing sensitivity
    ax = axes[0, 1]
    ax.set_title("(b) Double-support duration changes the CoM speed")
    for ds, c in [(0.2, "#4c72b0"), (0.5, "#55a868"), (0.8, "#c44e52")]:
        # recompute quickly with the same footsteps
        import json

        from pydrake.all import PiecewisePolynomial, ZmpPlanner

        ts_ = [0.0]
        z = [footsteps[0]]
        time = 0.0
        time += 1.0
        ts_.append(time)
        z.append(footsteps[0])
        for i in range(1, len(footsteps)):
            time += ds
            ts_.append(time)
            z.append(footsteps[i])
            time += 1.0
            ts_.append(time)
            z.append(footsteps[i])
        traj = PiecewisePolynomial.FirstOrderHold(ts_, np.array(z).T)
        pl = ZmpPlanner()
        pl.Plan(traj, np.zeros(4), 1.0)
        tt = np.linspace(0, traj.end_time(), 300)
        vx = np.array([pl.get_nominal_comd(s)[0] for s in tt])
        ax.plot(tt, vx, color=c, lw=1.3,
                label=fr"$T_{{ds}}={ds}$ s, $T={traj.end_time():.1f}$ s")
    ax.set_xlabel("t (s)")
    ax.set_ylabel(r"$\dot x$ (m/s)")
    ax.legend(fontsize=7, frameon=False)

    # (c) feedback under a push
    ax = axes[1, 0]
    ax.set_title("(c) Closed-loop policy rejects a velocity push")
    ax.plot(t_sim, x_nom[:, 0], "k", lw=1.3, label="nominal")
    ax.plot(t_sim, x_pert[:, 0], "r", lw=1.0, ls="--", label="perturbed")
    ax.plot(t_sim, x_pert[:, 1], "b", lw=1.0, ls=":", label=r"perturbed $y$")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("CoM (m)")
    ax.legend(fontsize=7, frameon=False)

    # (d) tracking error vs R
    ax = axes[1, 1]
    ax.set_title(r"(d) Tracking vs. control effort (weight $R$)")
    rs = [1e-4, 1e-2, 0.1, 1.0]
    # values from results_ex51_zmp.json
    import json

    with open(os.path.join(HERE, "results_ex51_zmp.json")) as f:
        res = json.load(f)
    err = [res["R_sensitivity"][str(r)]["max_tracking_error"] for r in rs]
    acc = [res["R_sensitivity"][str(r)]["max_abs_acceleration"] for r in rs]
    ax.loglog(acc, err, "o-", color="#8172b3")
    for r, a, e in zip(rs, acc, err):
        ax.annotate(fr"$R={r:g}$", (a, e), textcoords="offset points",
                    xytext=(4, 4), fontsize=7)
    ax.set_xlabel(r"max $|\ddot x|$ (m/s$^2$)")
    ax.set_ylabel("max ZMP tracking error (m)")

    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig5_3_zmp_planner.png"))
    plt.close(fig)


# ---------------------------------------------------------------------------
# 4. Exercise 5.2
# ---------------------------------------------------------------------------
def fig_ex52():
    d = np.load(os.path.join(HERE, "ex52_zmp_data.npz"))
    h, L1, L2, l = 0.08, 0.12, 0.08, 0.5
    theta_H, theta_T = d["theta_H"], d["theta_T"]

    fig = plt.figure(figsize=(9.8, 7.2))
    gs = fig.add_gridspec(2, 2, height_ratios=[1.0, 1.0])

    # (a) geometry
    ax = fig.add_subplot(gs[0, 0])
    ax.set_title("(a) Foot / leg / bob geometry")
    ax.set_aspect("equal")
    ax.set_xlim(-0.25, 0.45)
    ax.set_ylim(-0.06, 0.62)
    ax.axhline(0, color="0.3", lw=1)
    foot = np.array([[0, 0], [L1 + L2, 0], [L2, h]])
    ax.add_patch(Polygon(foot, closed=True, fc="#f0d6a8", ec="k", alpha=0.9))
    ax.plot([L2, L2], [0, h], "k:", lw=0.8)
    for ang, lab in [(np.pi / 2, r"$\theta=\pi/2$"), (1.15, r"$\theta$")]:
        pass
    th = 1.15
    bob = np.array([L2 + l * np.cos(th), h + l * np.sin(th)])
    ax.plot([L2, bob[0]], [h, bob[1]], "k", lw=2)
    ax.add_patch(Circle(bob, 0.025, fc="#7fa8c9", ec="k", zorder=4))
    ax.plot([L2, L2 + 0.14], [h, h], "k--", lw=0.7)
    arc = Arc((L2, h), 0.16, 0.16, theta1=0,
                                 theta2=np.degrees(th), color="k", lw=0.7)
    ax.add_patch(arc)
    ax.text(L2 + 0.09, h + 0.04, r"$\theta$")
    for pt, lab, off in [((0, 0), "H", (-0.03, -0.035)),
                         ((L1 + L2, 0), "T", (0.01, -0.035)),
                         ((L2, h), "P", (0.01, 0.015))]:
        ax.plot(*pt, "ko", ms=4)
        ax.text(pt[0] + off[0], pt[1] + off[1], lab, fontsize=11)
    ax.annotate("", xy=(L2, -0.03), xytext=(0, -0.03),
                arrowprops=dict(arrowstyle="<->", lw=0.7))
    ax.text(L2 / 2, -0.055, r"$L_2$", ha="center")
    ax.annotate("", xy=(L1 + L2, 0.05), xytext=(L2, 0.05),
                arrowprops=dict(arrowstyle="<->", lw=0.7))
    ax.text(L2 + L1 / 2, 0.06, r"$L_1$", ha="center")
    ax.set_xticks([])
    ax.set_yticks([])
    ax.grid(False)

    # (b) ZMP vs theta
    ax = fig.add_subplot(gs[0, 1])
    ax.set_title("(b) ZMP on the ground vs. $\\theta$")
    th = np.linspace(0.35, np.pi - 0.35, 800)
    ax.plot(th, L2 - h / np.tan(th), "r", lw=1.6,
            label=r"free leg: $L_2-h\cot\theta$")
    ax.plot(th, L2 + l * np.cos(th), "b", lw=1.6,
            label=r"ideal ankle: $L_2+l\cos\theta$")
    ax.axhline(0, color="0.4", lw=1)
    ax.axhline(L1 + L2, color="0.4", lw=1)
    ax.axvspan(theta_H, theta_T, color="0.85", alpha=0.5)
    ax.axvline(theta_H, color="0.3", ls=":", lw=0.8)
    ax.axvline(theta_T, color="0.3", ls=":", lw=0.8)
    ax.text(theta_H + 0.03, L1 + L2 + 0.012, r"$\theta_H$", fontsize=8)
    ax.text(theta_T - 0.16, L1 + L2 + 0.012, r"$\theta_T$", fontsize=8)
    ax.set_xlabel(r"$\theta$ (rad)")
    ax.set_ylabel("ZMP x (m)")
    ax.set_ylim(-0.12, L1 + L2 + 0.05)
    ax.legend(fontsize=7, frameon=False, loc="lower left")

    # (c) simulated free-leg motion
    ax = fig.add_subplot(gs[1, 0])
    ax.set_title("(c) Free leg falling from $\\theta_0=\\pi/2+0.1$")
    ax.plot(d["simulation_t"], d["simulation_theta"], "k", lw=1.3)
    ax.axhline(np.pi / 2, color="0.5", ls="--", lw=0.8)
    ax.axhline(theta_H, color="r", ls=":", lw=0.8)
    ax.text(0.05, theta_H + 0.03, r"$\theta_H$", color="r")
    ax.set_xlabel("t (s)")
    ax.set_ylabel(r"$\theta$ (rad)")

    # (d) design study
    ax = fig.add_subplot(gs[1, 1])
    ax.set_title("(d) Foot design: ankle placement")
    ax.plot(d["L1_grid"], d["angle_range"], "k", lw=1.4)
    i = np.nanargmax(d["angle_range"])
    ax.plot(d["L1_grid"][i], d["angle_range"][i], "r*", ms=12,
            label="maximum: ankle at a foot end")
    ax.plot(0.10, np.interp(0.10, d["L1_grid"], d["angle_range"]), "bo",
            ms=6, label="centered ankle")
    ax.set_xlabel(r"$L_1$ (m), with $L_1+L_2=0.2$ m")
    ax.set_ylabel(r"achievable $\theta$ range (rad)")
    ax.legend(fontsize=7, frameon=False)

    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig5_4_exercise52_zmp.png"))
    plt.close(fig)


# ---------------------------------------------------------------------------
# 5. Footstep plans
# ---------------------------------------------------------------------------
class Stone:
    def __init__(self, center, width, height):
        self.center = np.array(center, dtype=float)
        self.width = width
        self.height = height


class Terrain:
    """Same geometry as the exercise notebooks."""

    def __init__(self, bool_bridge):
        self.stones = []
        initial = Stone([0, 0], 1, 1)
        self.stones.append(initial)
        width_bridge = 0.2
        center = np.array([0.5, -0.5]) + np.array([0.3, 0.25])
        for i in np.where(bool_bridge)[0]:
            self.stones.append(Stone(center + [i * 0.4, 0], width_bridge, 0.5))
        c2g = np.array([1 + (len(bool_bridge) * 2 + 1) * 0.2, 0.0])
        self.stones.append(Stone(c2g, 1, 1))
        self.stones.append(Stone(c2g / 2 + [0, 0.8], 1 + c2g[0], 0.4))


def plot_terrain(ax, terrain):
    for s in terrain.stones:
        ax.add_patch(Rectangle(s.center - [s.width / 2, s.height / 2],
                               s.width, s.height, fc="#d9ead3",
                               ec="#38761d", alpha=0.6, lw=0.8))


def fig_footstep_plans():
    mip = np.load(os.path.join(HERE, "ex51_footstep_mip.npz"))
    gcs = np.load(os.path.join(HERE, "ex53_footstep_gcs.npz"))
    fig, axes = plt.subplots(2, 2, figsize=(10.0, 6.6))

    for col, (data, name) in enumerate([(mip, "MIP"), (gcs, "GCS")]):
        for row, (key, title) in enumerate([("A", "complete bridge"),
                                            ("B", "one bridge stone missing")]):
            ax = axes[row, col]
            bool_bridge = [1, 1, 1, 1] if key == "A" else [1, 1, 1, 0]
            terrain = Terrain(bool_bridge)
            plot_terrain(ax, terrain)
            left, right = data[f"left_{key}"], data[f"right_{key}"]
            ax.plot(left[:, 0], left[:, 1], "r.-", lw=0.9, ms=5,
                    label="left foot")
            ax.plot(right[:, 0], right[:, 1], "b.--", lw=0.9, ms=5,
                    label="right foot")
            ax.scatter(left[0, 0], left[0, 1], s=70, facecolors="none",
                       edgecolors="r")
            ax.scatter(right[0, 0], right[0, 1], s=70, facecolors="none",
                       edgecolors="b")
            ax.set_aspect("equal")
            ax.set_title(f"{name}, terrain {key} ({title})")
            ax.set_xlabel("x (m)")
            ax.set_ylabel("y (m)")
            ax.set_xlim(-0.7, 4.0)
            ax.set_ylim(-0.75, 1.15)
            if row == 0 and col == 0:
                ax.legend(fontsize=7, frameon=False, loc="upper right")

    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig5_5_footstep_plans.png"))
    plt.close(fig)


# ---------------------------------------------------------------------------
# 6. LittleDog
# ---------------------------------------------------------------------------
def fig_littledog():
    d = np.load(os.path.join(HERE, "ex52_littledog_solution.npz"), allow_pickle=True)
    t = d["t_sol"]
    com = d["com"]
    contact = d["contact_force"]  # (4, 3, N-1)
    in_stance = d["in_stance"]
    q = d["q"]

    fig, axes = plt.subplots(2, 2, figsize=(10.0, 6.8))

    # (a) contact schedule
    ax = axes[0, 0]
    ax.set_title("(a) Optimized contact schedule (walking trot)")
    labels = ["FL", "FR", "BL", "BR"]
    colors = ["#4c72b0", "#dd8452", "#55a868", "#c44e52"]
    for i in range(4):
        ax.step(t, in_stance[i] + 1.2 * i, where="post", color=colors[i], lw=1.5)
        ax.text(t[-1] + 0.005, 1.2 * i + 0.1, labels[i], fontsize=8,
                color=colors[i])
    ax.set_yticks([])
    ax.set_xlabel("t (s)")
    ax.set_xlim(t[0], t[-1] + 0.05)

    # (b) CoM height and forward velocity
    ax = axes[0, 1]
    ax.set_title("(b) CoM height and forward velocity")
    ax.plot(t, com[2], "b", lw=1.4, label=r"$z_{cm}$")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("z (m)", color="b")
    ax2 = ax.twinx()
    vx = np.gradient(com[0], t)
    ax2.plot(t, vx, "r", lw=1.2, label=r"$\dot x_{cm}$")
    ax2.set_ylabel(r"$\dot x$ (m/s)", color="r")
    ax2.grid(False)

    # (c) normal forces
    ax = axes[1, 0]
    ax.set_title("(c) Contact normal forces")
    for i in range(4):
        ax.plot(t[:-1], contact[i, 2, :], color=colors[i], lw=1.2,
                label=labels[i])
    ax.set_xlabel("t (s)")
    ax.set_ylabel(r"$f_z$ (N)")
    ax.legend(fontsize=7, frameon=False, ncol=4)

    # (d) joint angles
    ax = axes[1, 1]
    ax.set_title("(d) Joint angles (12 actuated coordinates)")
    for j in range(6, min(12, q.shape[0])):
        ax.plot(t, q[j], lw=1.0, label=f"q{j}")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("angle (rad)")
    ax.legend(fontsize=6, frameon=False, ncol=2)

    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig5_6_littledog.png"))
    plt.close(fig)


if __name__ == "__main__":
    fig_zmp_concept()
    fig_lipm_dcm()
    fig_zmp_planner()
    fig_ex52()
    fig_footstep_plans()
    fig_littledog()
    print("figures written to", FIG)
