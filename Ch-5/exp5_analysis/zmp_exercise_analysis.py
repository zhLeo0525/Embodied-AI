"""Analysis of the second Section 5.8 exercise: Understanding the ZMP.

Produces:
  fig5_11_zmp_exercise.png   geometry, unactuated ZMP, actuated ZMP range
  results_zmp_exercise.json  numerical boundary values
"""

import json
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams.update({
    "font.size": 12,
    "axes.titlesize": 12,
    "axes.labelsize": 12,
    "legend.fontsize": 9,
    "xtick.labelsize": 10,
    "ytick.labelsize": 10,
})
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "figures")
os.makedirs(OUT, exist_ok=True)

H = 0.08
L1 = 0.15
L2 = 0.08
L = 1.0


def main():
    # Unactuated case: the leg is a two-force member, so the ground reaction
    # is along the leg and the ZMP is where that line meets the ground.
    theta_T = np.pi - np.arctan(H / L1)   # ZMP reaches the toe
    theta_H = np.arctan(H / L2)           # ZMP reaches the heel
    # Actuated case (ankle torque cancels gravity): ZMP = CoM projection.
    theta_min = np.arccos(np.clip(L1 / L, -1.0, 1.0))
    theta_max = np.arccos(np.clip(-L2 / L, -1.0, 1.0))

    results = {
        "parameters": {"h": H, "L1": L1, "L2": L2, "l": L},
        "unactuated": {
            "zmp_expression": "-h * cot(theta)",
            "theta_zmp_at_toe_rad": float(theta_T),
            "theta_zmp_at_toe_deg": float(np.degrees(theta_T)),
            "theta_heel_lift_rad": float(theta_T),
            "theta_zmp_at_heel_rad": float(theta_H),
            "theta_zmp_at_heel_deg": float(np.degrees(theta_H)),
        },
        "actuated": {
            "zmp_expression": "l * cos(theta)",
            "theta_min_rad": float(theta_min),
            "theta_min_deg": float(np.degrees(theta_min)),
            "theta_max_rad": float(theta_max),
            "theta_max_deg": float(np.degrees(theta_max)),
            "range_rad": float(theta_max - theta_min),
            "range_deg": float(np.degrees(theta_max - theta_min)),
        },
    }

    # --- geometry schematic (separate figure) --------------------------
    fig_geom = plt.figure(figsize=(4.8, 5.2))
    ax = fig_geom.add_subplot(111)
    theta = np.radians(70)
    P = np.array([0.0, H])
    Hp = np.array([-L2, 0.0])
    Tp = np.array([L1, 0.0])
    mass = P + L * np.array([np.cos(theta), np.sin(theta)])
    ax.fill([Hp[0], P[0], Tp[0]], [0, H, 0], color="navajowhite",
            edgecolor="k", lw=1.2)
    ax.plot([P[0], mass[0]], [P[1], mass[1]], "k-", lw=2)
    ax.plot(*mass, "o", ms=14, mfc="navajowhite", mec="k")
    ax.text(mass[0], mass[1] + 0.045, "$m$", ha="center", fontsize=13)
    ax.plot([P[0] - 0.10, P[0] + 0.16], [H, H], "k--", lw=1)
    ax.annotate("", xy=(P[0] + 0.14 * np.cos(theta),
                        H + 0.14 * np.sin(theta)),
                xytext=(P[0] + 0.14, H),
                arrowprops=dict(arrowstyle="->", lw=1.2))
    ax.text(P[0] + 0.13, H + 0.035, r"$\theta$", fontsize=13)
    ax.annotate("", xy=(-L2, -0.045), xytext=(0, -0.045),
                arrowprops=dict(arrowstyle="<->", lw=1))
    ax.text(-L2 / 2, -0.075, "$L_2$", ha="center", fontsize=13)
    ax.annotate("", xy=(L1, -0.045), xytext=(0, -0.045),
                arrowprops=dict(arrowstyle="<->", lw=1))
    ax.text(L1 / 2, -0.075, "$L_1$", ha="center", fontsize=13)
    ax.annotate("", xy=(0.035, 0), xytext=(0.035, H),
                arrowprops=dict(arrowstyle="<->", lw=1))
    ax.text(0.045, H / 2, "$h$", fontsize=13)
    mid = (P + mass) / 2
    ax.text(mid[0] + 0.045, mid[1], "$l$", fontsize=13)
    ax.text(Hp[0] - 0.01, 0.012, "$H$", ha="right", fontsize=13)
    ax.text(Tp[0] + 0.005, 0.012, "$T$", fontsize=13)
    ax.text(-0.065, H + 0.01, "$P$", fontsize=13)
    ax.axhline(0, color="0.6", lw=4)
    ax.set_xlim(-0.35, 0.75)
    ax.set_ylim(-0.13, 1.13)
    ax.set_aspect("equal")
    ax.axis("off")
    ax.set_title("Exercise geometry", fontsize=13)
    fig_geom.tight_layout()
    fig_geom.savefig(os.path.join(OUT, "fig5_12_foot_geometry.png"), dpi=200)
    plt.close(fig_geom)

    # --- ZMP curves (two panels) ---------------------------------------
    fig = plt.figure(figsize=(10, 4.2))
    gs = fig.add_gridspec(1, 2)

    ax = fig.add_subplot(gs[0, 0])
    th = np.linspace(0.25, np.pi - 0.25, 500)
    for h_val, style in zip([0.05, 0.08, 0.12], [":", "-", "--"]):
        ax.plot(th, -h_val / np.tan(th), style, lw=1.8, label=f"h = {h_val} m")
    ax.axhline(L1, color="tab:red", lw=1, ls="-.")
    ax.axhline(-L2, color="tab:red", lw=1, ls="-.")
    ax.axhspan(-L2, L1, color="tab:green", alpha=0.06,
               label="support polygon")
    ax.axvspan(theta_H, theta_T, color="tab:orange", alpha=0.10,
               label="flat-foot range")
    ax.axvline(theta_T, color="0.4", ls=":", lw=1)
    ax.axvline(theta_H, color="0.4", ls=":", lw=1)
    ax.text(theta_T - 0.05, L1 + 0.03, r"$\theta_T$", ha="right", fontsize=12)
    ax.text(theta_H + 0.05, -L2 - 0.05, r"$\theta_H$", fontsize=12)
    ax.set_xlabel(r"$\theta$ [rad]")
    ax.set_ylabel(r"$x_{\mathrm{ZMP}}$ [m]")
    ax.set_ylim(-0.30, 0.30)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=9, loc="upper right")
    ax.set_title(r"Unactuated: $x_{\mathrm{ZMP}}=-h\cot\theta$")

    ax = fig.add_subplot(gs[0, 1])
    ax.plot(th, L * np.cos(th), "k-", lw=1.8, label=r"$l\cos\theta$")
    ax.axhline(L1, color="tab:red", lw=1, ls="-.")
    ax.axhline(-L2, color="tab:red", lw=1, ls="-.")
    ax.axvspan(theta_min, theta_max, color="tab:green", alpha=0.15,
               label="statically feasible")
    ax.axvline(theta_min, color="0.4", ls=":")
    ax.axvline(theta_max, color="0.4", ls=":")
    ax.set_xlabel(r"$\theta$ [rad]")
    ax.set_ylabel(r"$x_{\mathrm{ZMP}}$ [m]")
    ax.set_ylim(-0.25, 0.25)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=9, loc="upper right")
    ax.set_title(r"Actuated: $x_{\mathrm{ZMP}}=l\cos\theta$")

    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig5_11_zmp_exercise.png"), dpi=200)
    plt.close(fig)

    with open(os.path.join(HERE, "results_zmp_exercise.json"), "w") as f:
        json.dump(results, f, indent=2)
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
