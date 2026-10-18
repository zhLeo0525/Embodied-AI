"""Example 4.3 -- rimless wheel: return map, fixed point and stability.

The analysis uses the formulas derived in the lecture notes (Section 4.4):
    omega_{n+1} = cos(2 alpha) sqrt(omega_n^2 + 4 (g/l) sin(alpha) sin(gamma))
    omega_1 = sqrt(2 (g/l) [1 - cos(gamma - alpha)])
    omega* = cot(2 alpha) sqrt(4 (g/l) sin(alpha) sin(gamma))
    d omega_{n+1}/d omega_n |_* = cos^2(2 alpha)

Outputs: results.json and figures/fig_rw_*.png
"""

import json
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

FIG = "figures"
os.makedirs(FIG, exist_ok=True)

G = 9.81
L = 1.0
ALPHA = np.pi / 8.0


def omega1(gamma):
    if gamma >= ALPHA:
        return 0.0
    return np.sqrt(2.0 * G / L * (1.0 - np.cos(gamma - ALPHA)))


def D(gamma):
    return 4.0 * G / L * np.sin(ALPHA) * np.sin(gamma)


def f_map(omega, gamma):
    return np.cos(2 * ALPHA) * np.sqrt(np.maximum(omega**2 + D(gamma), 0.0))


def fixed_point(gamma):
    return 1.0/np.tan(2 * ALPHA) * np.sqrt(D(gamma))


def main():
    slopes = [0.02, 0.05, 0.08, 0.12, 0.20]
    rows = []
    for gamma in slopes:
        ws = fixed_point(gamma)
        w1 = omega1(gamma)
        valid = ws > w1
        rows.append(
            {
                "gamma": gamma,
                "omega1": w1,
                "omega_star": ws,
                "rolling_fixed_point_exists": bool(valid),
                "multiplier_cos2_2alpha": float(np.cos(2 * ALPHA) ** 2),
            }
        )

    # Return map for a few slopes.
    fig, ax = plt.subplots(figsize=(7, 5))
    om = np.linspace(0, 4, 500)
    for gamma, color in zip([0.05, 0.08, 0.12], ["tab:blue", "tab:orange", "tab:green"]):
        ax.plot(om, f_map(om, gamma), color=color, label=fr"$\gamma={gamma}$")
        ws = fixed_point(gamma)
        if ws > omega1(gamma):
            ax.plot(ws, ws, "o", color=color, ms=5)
    ax.plot(om, om, "k--", lw=0.8, label="diagonal")
    ax.set_xlabel(r"$\omega_n^+$ [rad/s]")
    ax.set_ylabel(r"$\omega_{n+1}^+$ [rad/s]")
    ax.set_title("Rimless wheel: step-to-step return map")
    ax.set_xlim(0, 4)
    ax.set_ylim(0, 4)
    ax.grid(True, ls=":", lw=0.4)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig_rw_return_map.png"), dpi=200)
    plt.close(fig)

    # Cobweb for gamma=0.08, omega0=2.
    gamma = 0.08
    fig, ax = plt.subplots(figsize=(7, 5))
    om = np.linspace(0, 3, 400)
    ax.plot(om, f_map(om, gamma), "b", label=r"$f(\omega)$")
    ax.plot(om, om, "k--", lw=0.8)
    w = 2.0
    traj = [w]
    for _ in range(8):
        wn = float(f_map(w, gamma))
        ax.plot([w, w], [w, wn], "r-", lw=1)
        ax.plot([w, wn], [wn, wn], "r-", lw=1)
        w = wn
        traj.append(w)
    ws = fixed_point(gamma)
    ax.plot(ws, ws, "ko", ms=5, label=fr"fixed point $\omega^*={ws:.3f}$")
    ax.set_xlabel(r"$\omega_n^+$ [rad/s]")
    ax.set_ylabel(r"$\omega_{n+1}^+$ [rad/s]")
    ax.set_title(r"Cobweb iteration for $\gamma=0.08$")
    ax.grid(True, ls=":", lw=0.4)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig_rw_cobweb.png"), dpi=200)
    plt.close(fig)

    # Fixed point and threshold vs slope.
    gs = np.linspace(0.005, 0.3, 200)
    ws = np.array([fixed_point(g) for g in gs])
    w1 = np.array([omega1(g) for g in gs])
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.plot(gs, ws, label=r"rolling fixed point $\omega^*$")
    ax.plot(gs, w1, "--", label=r"step threshold $\omega_1$")
    ax.set_xlabel(r"slope $\gamma$ [rad]")
    ax.set_ylabel("angular velocity [rad/s]")
    ax.set_title("Rimless wheel: fixed point and threshold vs slope")
    ax.grid(True, ls=":", lw=0.4)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig_rw_fixed_point.png"), dpi=200)
    plt.close(fig)

    with open("results_rw.json", "w") as fh:
        json.dump(
            {
                "alpha": ALPHA,
                "length": L,
                "gravity": G,
                "slopes": rows,
                "cobweb_gamma": gamma,
                "cobweb_iterates": traj,
            },
            fh,
            indent=2,
        )

    print("slope  omega1   omega*   fixed point exists")
    for r in rows:
        print(
            f"{r['gamma']:.3f}  {r['omega1']:.4f}  {r['omega_star']:.4f}  "
            f"{r['rolling_fixed_point_exists']}"
        )


if __name__ == "__main__":
    main()
