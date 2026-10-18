"""
Exercise 3.4 -- Comparison between stability of a nonlinear system and its
linearization, using the undamped / damped pendulum

    theta_ddot = -g sin(theta) - b theta_dot,     m = l = 1, g = 9.81.

Equilibria: x* = [0, 0] (hanging down) and x* = [pi, 0] (upright).
Running this script produces
    figures/ex3_4_phase_down.png
    figures/ex3_4_phase_up.png
    results_ex3_4.json
in the current directory.
"""

import json
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp

FIG_DIR = "figures"
os.makedirs(FIG_DIR, exist_ok=True)

G = 9.81  # m/s^2


# ---------------------------------------------------------------------------
# Dynamics
# ---------------------------------------------------------------------------


def f_nonlinear(x, b):
    """Nonlinear pendulum with damping coefficient b (per unit m*l^2)."""
    theta, theta_dot = x
    return np.array([theta_dot, -G * np.sin(theta) - b * theta_dot])


def f_linear(x, b, theta_star):
    """Linearization of f_nonlinear about [theta_star, 0]."""
    theta, theta_dot = x
    delta = theta - theta_star
    return np.array(
        [
            theta_dot,
            -G * np.cos(theta_star) * delta - b * theta_dot,
        ]
    )


def elevation(x, u):
    """Vectorized nonlinear field for streamplot (u is b)."""
    theta, theta_dot = x
    return np.array([theta_dot, -G * np.sin(theta) - u * theta_dot])


def elevation_linear(x, u):
    """Vectorized linearized field for streamplot (u = [b, theta_star])."""
    b, th_star = u
    theta, theta_dot = x
    return np.array([theta_dot, -G * np.cos(th_star) * (theta - th_star) - b * theta_dot])


def jacobian(b, theta_star):
    return np.array([[0.0, 1.0], [-G * np.cos(theta_star), -b]])


# ---------------------------------------------------------------------------
# Phase portraits
# ---------------------------------------------------------------------------


def phase_panel(
    ax,
    field,
    field_args,
    xlim,
    ylim,
    title,
    trajectories=None,
    equi=None,
    n_grid=240,
):
    """Stream plot + sample trajectories for a 2D autonomous system."""
    th = np.linspace(xlim[0], xlim[1], n_grid)
    thd = np.linspace(ylim[0], ylim[1], n_grid)
    TH, THD = np.meshgrid(th, thd)

    dth, dthd = field([TH, THD], field_args)
    speed = np.hypot(dth, dthd)
    ax.streamplot(
        TH,
        THD,
        dth,
        dthd,
        color=speed,
        cmap="viridis",
        density=1.25,
        linewidth=0.6,
        arrowsize=0.8,
    )

    if trajectories is not None:
        for x0, color, style in trajectories:
            # Integrate forward and backward so that the orbit is visible.
            for direction in (1.0, -1.0):
                t_span = (0.0, direction * 8.0)
                sol = solve_ivp(
                    lambda t, x: field(x, field_args),
                    t_span,
                    x0,
                    rtol=1e-8,
                    atol=1e-10,
                    max_step=0.01,
                )
                if sol.y.shape[1] > 1:
                    ax.plot(
                        sol.y[0],
                        sol.y[1],
                        color=color,
                        lw=1.2,
                        ls=style,
                        alpha=0.95,
                    )

    if equi is not None:
        ax.plot(equi[0], equi[1], "o", ms=8, mfc="red", mec="black", zorder=5)
        ax.annotate(
            r"$x^*$",
            equi,
            textcoords="offset points",
            xytext=(8, 8),
            fontsize=12,
            color="red",
        )

    ax.set_xlim(xlim)
    ax.set_ylim(ylim)
    ax.set_xlabel(r"$\theta$")
    ax.set_ylabel(r"$\dot\theta$")
    ax.set_title(title, fontsize=11)
    ax.grid(True, ls=":", lw=0.4, alpha=0.6)


def make_down_figure():
    xlim = (-np.pi, np.pi)
    ylim = (-7.5, 7.5)
    fig, axes = plt.subplots(2, 2, figsize=(13, 10))

    for row, b in enumerate([0.0, 1.0]):
        nl_traj = [
            (np.array([0.6, 2.0]), "black", "-"),
            (np.array([1.2, 0.0]), "dimgray", "-"),
            (np.array([2.4, 0.0]), "black", "--"),
            (np.array([0.0, 5.0]), "dimgray", "--"),
        ]
        lin_traj = [
            (np.array([0.6, 2.0]), "black", "-"),
            (np.array([1.2, 0.0]), "dimgray", "-"),
            (np.array([2.4, 0.0]), "black", "--"),
            (np.array([0.0, 5.0]), "dimgray", "--"),
        ]
        phase_panel(
            axes[row, 0],
            elevation,
            b,
            xlim,
            ylim,
            f"Nonlinear pendulum, b={b:g}, around $x^*=(0,0)$",
            nl_traj,
            equi=(0.0, 0.0),
        )
        phase_panel(
            axes[row, 1],
            elevation_linear,
            (b, 0.0),
            xlim,
            ylim,
            f"Linearization, b={b:g}, around $x^*=(0,0)$",
            lin_traj,
            equi=(0.0, 0.0),
        )

    fig.suptitle(
        r"Exercise 3.4: phase portraits around the hanging equilibrium $(0,0)$",
        fontsize=14,
    )
    fig.tight_layout()
    fig.savefig(os.path.join(FIG_DIR, "ex3_4_phase_down.png"), dpi=200)
    plt.close(fig)


def make_up_figure():
    x_star = np.pi
    xlim = (x_star - 1.3, x_star + 1.3)
    ylim = (-7.5, 7.5)
    fig, axes = plt.subplots(2, 2, figsize=(13, 10))

    for row, b in enumerate([0.0, 1.0]):
        nl_traj = [
            (np.array([x_star + 0.3, 0.0]), "black", "-"),
            (np.array([x_star, 1.0]), "dimgray", "-"),
            (np.array([x_star - 0.5, 0.8]), "black", "--"),
            (np.array([x_star + 0.6, -1.5]), "dimgray", "--"),
        ]
        lin_traj = [
            (np.array([x_star + 0.3, 0.0]), "black", "-"),
            (np.array([x_star, 1.0]), "dimgray", "-"),
            (np.array([x_star - 0.5, 0.8]), "black", "--"),
            (np.array([x_star + 0.6, -1.5]), "dimgray", "--"),
        ]
        phase_panel(
            axes[row, 0],
            elevation,
            b,
            xlim,
            ylim,
            rf"Nonlinear pendulum, b={b:g}, around $x^*=(\pi,0)$",
            nl_traj,
            equi=(x_star, 0.0),
        )
        phase_panel(
            axes[row, 1],
            elevation_linear,
            (b, x_star),
            xlim,
            ylim,
            rf"Linearization, b={b:g}, around $x^*=(\pi,0)$",
            lin_traj,
            equi=(x_star, 0.0),
        )

    fig.suptitle(
        r"Exercise 3.4: phase portraits around the upright equilibrium $(\pi,0)$",
        fontsize=14,
    )
    fig.tight_layout()
    fig.savefig(os.path.join(FIG_DIR, "ex3_4_phase_up.png"), dpi=200)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Eigenvalues and the summary table
# ---------------------------------------------------------------------------


def eigen_data():
    data = {}
    for b in [0.0, 1.0]:
        for name, th in [("down", 0.0), ("up", np.pi)]:
            A = jacobian(b, th)
            eigs = np.linalg.eigvals(A)
            data[f"b={b:g}_{name}"] = {
                "theta_star": th,
                "A": A.tolist(),
                "eigenvalues": [[float(z.real), float(z.imag)] for z in eigs],
                "sign_Re": [int(np.sign(np.round(z.real, 10))) for z in eigs],
                "linear_stable_isl": bool(np.all(np.real(eigs) <= 1e-12)),
                "linear_asymptotically_stable": bool(np.all(np.real(eigs) < -1e-12)),
                "linear_exponentially_stable": bool(np.all(np.real(eigs) < -1e-12)),
            }
    return data


STABILITY_TABLE = [
    {
        "system": "Nonlinear, b=0, x*=(0,0)",
        "sign_Re_lambda1": "N/A",
        "sign_Re_lambda2": "N/A",
        "stable_isl": True,
        "asymptotically_stable": False,
        "exponentially_stable": False,
        "comment": "Closed orbits: Lyapunov stable, but not attracting.",
    },
    {
        "system": "Nonlinear, b=0, x*=(pi,0)",
        "sign_Re_lambda1": "N/A",
        "sign_Re_lambda2": "N/A",
        "stable_isl": False,
        "asymptotically_stable": False,
        "exponentially_stable": False,
        "comment": "Saddle: the upright pendulum falls for almost every perturbation.",
    },
    {
        "system": "Linearization, b=0, x*=(0,0)",
        "sign_Re_lambda1": 0,
        "sign_Re_lambda2": 0,
        "stable_isl": True,
        "asymptotically_stable": False,
        "exponentially_stable": False,
        "comment": "Eigenvalues +/-j*sqrt(g): a center.",
    },
    {
        "system": "Linearization, b=0, x*=(pi,0)",
        "sign_Re_lambda1": 1,
        "sign_Re_lambda2": -1,
        "stable_isl": False,
        "asymptotically_stable": False,
        "exponentially_stable": False,
        "comment": "Eigenvalues +/-sqrt(g): unstable saddle.",
    },
    {
        "system": "Nonlinear, b=1, x*=(0,0)",
        "sign_Re_lambda1": "N/A",
        "sign_Re_lambda2": "N/A",
        "stable_isl": True,
        "asymptotically_stable": True,
        "exponentially_stable": True,
        "comment": "Damping dissipates energy; locally exponential.",
    },
    {
        "system": "Nonlinear, b=1, x*=(pi,0)",
        "sign_Re_lambda1": "N/A",
        "sign_Re_lambda2": "N/A",
        "stable_isl": False,
        "asymptotically_stable": False,
        "exponentially_stable": False,
        "comment": "Still unstable; damping does not stabilize the inverted state.",
    },
    {
        "system": "Linearization, b=1, x*=(0,0)",
        "sign_Re_lambda1": -1,
        "sign_Re_lambda2": -1,
        "stable_isl": True,
        "asymptotically_stable": True,
        "exponentially_stable": True,
        "comment": "Eigenvalues -0.5 +/- 3.09j.",
    },
    {
        "system": "Linearization, b=1, x*=(pi,0)",
        "sign_Re_lambda1": 1,
        "sign_Re_lambda2": -1,
        "stable_isl": False,
        "asymptotically_stable": False,
        "exponentially_stable": False,
        "comment": "Eigenvalues 2.67 and -3.67 (one positive).",
    },
]


def main():
    make_down_figure()
    make_up_figure()
    eig = eigen_data()

    results = {
        "eigenvalues": eig,
        "stability_table": STABILITY_TABLE,
    }
    with open("results_ex3_4.json", "w") as f:
        json.dump(results, f, indent=2)

    print("=== Eigenvalues of the Jacobian ===")
    for key, val in eig.items():
        eigs = np.array(val["eigenvalues"])
        print(f"{key:12s} theta*={val['theta_star']:.4f}  "
              f"lambda = {np.round(eigs[:, 0], 4) + 1j*np.round(eigs[:, 1], 4)}")

    print("\n=== Stability table ===")
    for row in STABILITY_TABLE:
        print(
            f"{row['system']:38s} "
            f"signRe=({row['sign_Re_lambda1']},{row['sign_Re_lambda2']}) "
            f"i.s.L.={row['stable_isl']} "
            f"asymp={row['asymptotically_stable']} "
            f"exp={row['exponentially_stable']}"
        )


if __name__ == "__main__":
    main()
