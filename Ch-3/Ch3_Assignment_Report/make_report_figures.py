"""Regenerate the figures used in the consolidated report with English labels.

The numerical data come from the JSON files produced by the analysis scripts
in Ch.3-examples/exp1_analysis, exp3_2_analysis and exp3_5_analysis.
Figures are written to ./figures/ and replace the copies with Chinese labels.
"""

import json
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

FIG_DIR = "figures"
os.makedirs(FIG_DIR, exist_ok=True)

ROOT = ".."
EXP1 = os.path.join(ROOT, "Ch.3-examples", "exp1_analysis", "results.json")
EXP32 = os.path.join(ROOT, "Ch.3-examples", "exp3_2_analysis", "results.json")
EXP35 = os.path.join(ROOT, "Ch.3-examples", "exp3_5_analysis", "results.json")

D1 = json.load(open(EXP1))
D2 = json.load(open(EXP32))
D3 = json.load(open(EXP35))

plt.rcParams.update(
    {
        "font.size": 10,
        "axes.grid": True,
        "grid.linestyle": ":",
        "grid.linewidth": 0.5,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "figure.autolayout": True,
    }
)


def save(fig, name):
    fig.savefig(os.path.join(FIG_DIR, name), dpi=200, bbox_inches="tight")
    plt.close(fig)


# ---------------------------------------------------------------------------
# Example 3.1 -- acrobot
# ---------------------------------------------------------------------------


def acro_trajectories():
    fig, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    styles = [
        ("success_theta1", "Successful", "tab:green", "-"),
        ("failure_mixed", "Failed", "tab:red", "--"),
    ]
    for key, label, color, ls in styles:
        tr = D1["trajectories"][key]
        t = np.array(tr["t"])
        x = np.array(tr["x"])
        axes[0].plot(t, x[0], color=color, ls=ls, lw=1.4, label=label)
        axes[1].plot(t, x[1], color=color, ls=ls, lw=1.4, label=label)
    axes[0].set_ylabel(r"$\theta_1$ [rad]")
    axes[1].set_ylabel(r"$\theta_2$ [rad]")
    axes[1].set_xlabel("time [s]")
    axes[0].legend()
    axes[0].set_title("Acrobot closed loop with $|\\tau|\\leq 10$ N m")
    save(fig, "acro_fig1_success_vs_failure.png")


def acro_delta():
    rows = D1["delta_sweep"]
    delta = np.array([r["delta"] for r in rows])
    free = np.array([r["unbounded"]["success_rate"] for r in rows])
    sat = np.array([r["sat10"]["success_rate"] for r in rows])
    fig, ax = plt.subplots(figsize=(6.5, 4))
    ax.plot(delta, free, "o-", label="No saturation")
    ax.plot(delta, sat, "s--", label=r"$|\tau|\leq 10$ N m")
    ax.set_xlabel(r"disturbance scale $\delta$")
    ax.set_ylabel("success rate")
    ax.set_ylim(-0.02, 1.02)
    ax.legend()
    ax.set_title("Acrobot: basin of attraction vs. disturbance")
    save(fig, "acro_fig2_delta_sweep.png")


def acro_m2():
    rows = D1["m2_sweep"]
    m2 = np.array([r["value"] for r in rows])
    succ = np.array([r["summary"]["success_rate"] for r in rows])
    smin = np.array([r["sv"][-1] for r in rows])
    fig, ax = plt.subplots(figsize=(6.5, 4))
    ax.plot(m2, succ, "o-", color="tab:blue", label="success rate")
    ax.set_xlabel(r"distal mass $m_2$ [kg]")
    ax.set_ylabel("success rate", color="tab:blue")
    ax.tick_params(axis="y", labelcolor="tab:blue")
    ax.set_ylim(-0.02, 1.02)
    ax2 = ax.twinx()
    ax2.plot(m2, smin, "s--", color="tab:orange", label=r"$\sigma_{\min}(\mathbf{C})$")
    ax2.set_yscale("log")
    ax2.set_ylabel(r"$\sigma_{\min}(\mathbf{C})$", color="tab:orange")
    ax2.tick_params(axis="y", labelcolor="tab:orange")
    ax.set_title("Acrobot: effect of the distal mass")
    save(fig, "acro_fig3_m2_sweep.png")


# ---------------------------------------------------------------------------
# Example 3.2 -- cart-pole
# ---------------------------------------------------------------------------


def cp_trajectories():
    fig, axes = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
    for key, label, color, ls in [
        ("no_limit", "No force limit", "tab:green", "-"),
        ("lim2", r"$|u|\leq 2$ N", "tab:red", "--"),
    ]:
        tr = D2["trajectories"][key]
        t = np.array(tr["t"])
        x = np.array(tr["x"])
        u = np.array(tr["ucmd"])
        axes[0].plot(t, x[0], color=color, ls=ls, lw=1.3, label=label)
        axes[1].plot(t, x[1], color=color, ls=ls, lw=1.3, label=label)
        axes[2].plot(t, u, color=color, ls=ls, lw=1.2, label=label)
    axes[0].set_ylabel(r"$x$ [m]")
    axes[1].set_ylabel(r"$\theta$ [rad]")
    axes[2].set_ylabel(r"force [N]")
    axes[2].set_xlabel("time [s]")
    axes[0].legend()
    axes[0].set_title("Cart-pole: unlimited vs. saturated force")
    save(fig, "cp_fig1_success_vs_limit.png")


def cp_delta():
    rows = D2["delta_sweep"]
    delta = np.array([r["delta"] for r in rows])
    free = np.array([r["free"]["pole_rate"] for r in rows])
    lim5 = np.array([r["lim5"]["pole_rate"] for r in rows])
    fig, ax = plt.subplots(figsize=(6.5, 4))
    ax.plot(delta, free, "o-", label="No force limit")
    ax.plot(delta, lim5, "s--", label=r"$|u|\leq 5$ N")
    ax.set_xlabel(r"disturbance scale $\delta$")
    ax.set_ylabel("pole-balancing success rate")
    ax.set_ylim(-0.02, 1.02)
    ax.legend()
    ax.set_title("Cart-pole: pole balancing vs. disturbance")
    save(fig, "cp_fig2_delta_sweep.png")


def cp_force_limit():
    rows = D2["force_limit_sweep"]
    lim = np.array([r["limit"] for r in rows])
    pole = np.array([r["summary"]["pole_rate"] for r in rows])
    cart = np.array([r["summary"]["cart_rate"] for r in rows])
    fig, ax = plt.subplots(figsize=(6.5, 4))
    ax.plot(lim, pole, "o-", label="pole balanced")
    ax.plot(lim, cart, "s--", label="full state")
    ax.set_xscale("log")
    ax.set_xlabel("force limit [N]")
    ax.set_ylabel("success rate")
    ax.set_ylim(-0.02, 1.02)
    ax.legend()
    ax.set_title(r"Cart-pole: effect of the force limit ($\delta=0.1$)")
    save(fig, "cp_fig3_force_limit.png")


def cp_physical():
    rows = D2["mc_sweep"]
    mc = np.array([r["value"] for r in rows])
    peak = np.array([r["summary"]["peak"] for r in rows])
    cart = np.array([r["summary"]["cart_rate"] for r in rows])
    fig, ax = plt.subplots(figsize=(6.5, 4))
    ax.plot(mc, peak, "o-", color="tab:blue", label="median peak force")
    ax.set_xlabel(r"cart mass $m_c$ [kg]")
    ax.set_ylabel("force [N]", color="tab:blue")
    ax.tick_params(axis="y", labelcolor="tab:blue")
    ax2 = ax.twinx()
    ax2.plot(mc, cart, "s--", color="tab:orange", label="full-state success")
    ax2.set_ylabel("full-state success rate", color="tab:orange")
    ax2.set_ylim(-0.02, 1.02)
    ax2.tick_params(axis="y", labelcolor="tab:orange")
    ax.set_title("Cart-pole: effect of the cart mass")
    save(fig, "cp_fig4_physical.png")


# ---------------------------------------------------------------------------
# Example 3.5 -- comparison
# ---------------------------------------------------------------------------


def lqr_linear():
    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    for key, label, marker in [
        ("acrobot_linear", "Acrobot", "o"),
        ("cartpole_linear", "Cart-pole", "s"),
    ]:
        sv = np.array(D3[key]["sv"])
        axes[0].semilogy(
            np.arange(1, len(sv) + 1),
            sv / sv[0],
            marker=marker,
            label=label,
        )
    axes[0].set_xlabel("index")
    axes[0].set_ylabel(r"$\sigma_i(\mathbf{C})/\sigma_1(\mathbf{C})$")
    axes[0].set_title("Normalised controllability singular values")
    axes[0].legend()

    for key, label, ypos in [
        ("acrobot_linear", "Acrobot", 1.0),
        ("cartpole_linear", "Cart-pole", 0.0),
    ]:
        zeros = D3[key]["zeros"]
        for name, zs in zeros.items():
            zs = np.array(zs)
            axes[1].scatter(
                zs.real,
                np.full_like(zs.real, ypos),
                marker="x",
                s=60,
                label=f"{label}: {name}",
            )
        for z in zs.real:
            axes[1].annotate(
                f"{z:+.2f}",
                (z, ypos),
                textcoords="offset points",
                xytext=(0, 8 if ypos > 0.5 else -14),
                ha="center",
                fontsize=7,
            )
    axes[1].axvline(0, color="k", lw=0.8)
    axes[1].set_yticks([0, 1])
    axes[1].set_yticklabels(["Cart-pole", "Acrobot"])
    axes[1].set_ylim(-0.6, 1.6)
    axes[1].set_xlabel(r"Re$(z)$  (all zeros are real)")
    axes[1].set_title("Transmission zeros by system")
    axes[1].legend(fontsize=7, loc="lower right")
    save(fig, "lqr_fig1_linear.png")


def lqr_robustness():
    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    rows = D3["acrobot_robustness"]
    delta = np.array([r["delta"] for r in rows])
    axes[0].plot(delta, [r["free"]["success_rate"] for r in rows], "o-", label="No saturation")
    axes[0].plot(delta, [r["sat10"]["success_rate"] for r in rows], "s--", label=r"$|\tau|\leq10$ N m")
    axes[0].set_xlabel(r"$\delta$")
    axes[0].set_ylabel("success rate")
    axes[0].set_ylim(-0.02, 1.02)
    axes[0].set_title("Acrobot")
    axes[0].legend()

    rows = D3["cartpole_robustness"]
    delta = np.array([r["delta"] for r in rows])
    axes[1].plot(delta, [r["free"]["pole_rate"] for r in rows], "o-", label="No limit")
    axes[1].plot(delta, [r["lim20"]["pole_rate"] for r in rows], "s--", label=r"$|u|\leq20$ N")
    axes[1].plot(delta, [r["lim5"]["pole_rate"] for r in rows], "^:", label=r"$|u|\leq5$ N")
    axes[1].set_xlabel(r"$\delta$")
    axes[1].set_ylabel("pole-balancing success rate")
    axes[1].set_ylim(-0.02, 1.02)
    axes[1].set_title("Cart-pole")
    axes[1].legend()
    save(fig, "lqr_fig2_robustness.png")


def lqr_weights():
    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    for key, label, marker in [
        ("acrobot_R", "Acrobot", "o"),
        ("cartpole_R", "Cart-pole", "s"),
    ]:
        rows = D3[key]
        R = np.array([r["R"] for r in rows])
        axes[0].semilogx(R, [r["normD"] for r in rows], marker=marker, label=label)
        axes[1].semilogx(R, [r["slowest"] for r in rows], marker=marker, label=label)
    axes[0].set_xlabel("$R$")
    axes[0].set_ylabel(r"$\|D\|$")
    axes[0].set_title("Feedback-gain magnitude")
    axes[0].legend()
    axes[1].set_xlabel("$R$")
    axes[1].set_ylabel(r"slowest $|\lambda_{\mathrm{cl}}|$ [rad/s]")
    axes[1].set_title("Slowest closed-loop mode")
    axes[1].legend()
    save(fig, "lqr_fig3_weights.png")


def lqr_physical():
    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    rows = D3["acrobot_m2"]
    m2 = np.array([r["value"] for r in rows])
    axes[0].plot(m2, [r["summary"]["success_rate"] for r in rows], "o-")
    axes[0].set_xlabel(r"$m_2$ [kg]")
    axes[0].set_ylabel("success rate")
    axes[0].set_ylim(-0.02, 1.02)
    axes[0].set_title("Acrobot: success vs. distal mass")

    rows = D3["cartpole_mc"]
    mc = np.array([r["value"] for r in rows])
    axes[1].plot(mc, [r["summary"]["cart_rate"] for r in rows], "s-", color="tab:orange")
    axes[1].set_xlabel(r"$m_c$ [kg]")
    axes[1].set_ylabel("full-state success rate")
    axes[1].set_ylim(-0.02, 1.02)
    axes[1].set_title("Cart-pole: full-state success vs. cart mass")
    save(fig, "lqr_fig4_physical.png")


def main():
    acro_trajectories()
    acro_delta()
    acro_m2()
    cp_trajectories()
    cp_delta()
    cp_force_limit()
    cp_physical()
    lqr_linear()
    lqr_robustness()
    lqr_weights()
    lqr_physical()
    print("report figures written to", FIG_DIR)


if __name__ == "__main__":
    main()
