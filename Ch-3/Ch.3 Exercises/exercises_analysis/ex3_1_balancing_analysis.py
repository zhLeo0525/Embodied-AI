"""
Exercise 3.1 -- extra analysis for the cart-pole linearization / LQR notebook.

The notebook fixes m_c = m_p = 1, l = 1, g = 9.81 and
x = [x, theta, xdot, thetadot], u = [f_x].

This script
  * rebuilds the nonlinear dynamics f and the linearization (A, B),
  * recomputes the six linearization errors of the notebook,
  * designs the LQR controller for Q = I, R = 1 (as in the notebook),
  * simulates the nonlinear closed loop from the six initial states,
  * writes results_ex3_1.json and figures/ex3_1_linearization_error.png.

The LQR gain is computed with scipy (CARE), so this script does not require
Drake; the dynamics are exactly the equations used in the notebook.
"""

import json
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp
from scipy.linalg import solve_continuous_are

FIG_DIR = "figures"
os.makedirs(FIG_DIR, exist_ok=True)

G = 9.81
X_STAR = np.array([0.0, np.pi, 0.0, 0.0])


def f(x, u):
    """Cart-pole dynamics (m_c = m_p = l = 1)."""
    s = np.sin(x[1])
    c = np.cos(x[1])
    f3 = (u[0] + s * (x[3] ** 2 + G * c)) / (1.0 + s ** 2)
    f4 = (-u[0] * c - x[3] ** 2 * c * s - 2.0 * G * s) / (1.0 + s ** 2)
    return np.array([x[2], x[3], f3, f4])


A_LIN = np.array(
    [
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        [0.0, G, 0.0, 0.0],
        [0.0, 2.0 * G, 0.0, 0.0],
    ]
)
B_LIN = np.array([[0.0], [0.0], [1.0], [1.0]])


def f_lin(x, u):
    return A_LIN @ (x - X_STAR) + B_LIN @ (u - np.array([0.0]))


def linearization_error(x, u):
    return np.linalg.norm(f(x, u) - f_lin(x, u))


def lqr_gain(Q, R):
    P = solve_continuous_are(A_LIN, B_LIN, Q, R)
    K = np.linalg.solve(R, B_LIN.T @ P)
    return K, P


def closed_loop(t, x, K):
    u = -K @ (x - X_STAR)
    return f(x, u)


def simulate(x0, K, t_final=5.0):
    sol = solve_ivp(
        lambda t, x: closed_loop(t, x, K),
        (0.0, t_final),
        np.asarray(x0, dtype=float),
        rtol=1e-10,
        atol=1e-12,
        max_step=0.002,
    )
    return sol.t, sol.y


def main():
    x_list = [
        np.array([0.0, 0.99 * np.pi, 0.0, 0.0]),
        np.array([0.0, 0.90 * np.pi, 0.0, 0.0]),
        np.array([0.0, 0.85 * np.pi, 0.0, 0.0]),
        np.array([0.0, 0.50 * np.pi, 0.0, 0.0]),
        np.array([0.0, 0.00 * np.pi, 0.0, 0.0]),
        np.array([1.0, np.pi, 0.0, 0.0]),
    ]
    u_list = [
        np.array([0.0]),
        np.array([-10.0]),
        np.array([0.0]),
        np.array([0.0]),
        np.array([0.0]),
        np.array([10.0]),
    ]

    errors = [linearization_error(x, u) for x, u in zip(x_list, u_list)]
    f_norms = [np.linalg.norm(f(x, u)) for x, u in zip(x_list, u_list)]

    Q = np.eye(4)
    R = np.eye(1)
    K, P = lqr_gain(Q, R)
    eig_cl = np.linalg.eigvals(A_LIN - B_LIN @ K)

    # Nonlinear closed-loop simulations.
    sims = []
    for i, x0 in enumerate(x_list):
        t, y = simulate(x0, K)
        theta_err = np.angle(np.exp(1j * (y[1, -1] - np.pi)))
        recovered = abs(theta_err) < 0.1 and abs(y[3, -1]) < 0.5
        sims.append(
            {
                "index": i,
                "x0": x0.tolist(),
                "u0": u_list[i].tolist(),
                "error": float(errors[i]),
                "norm_f": float(f_norms[i]),
                "relative_error": float(errors[i] / f_norms[i])
                if f_norms[i] > 1e-9
                else None,
                "final_state": y[:, -1].tolist(),
                "final_theta_error": float(theta_err),
                "recovered": bool(recovered),
                "t": t.tolist(),
                "theta": y[1].tolist(),
                "x": y[0].tolist(),
            }
        )

    recovered = [s["index"] for s in sims if s["recovered"]]

    # Figure: linearization error (and |f|) for the six conditions.
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.5))
    idx = np.arange(6)
    axes[0].bar(idx - 0.18, errors, width=0.36, label="linearization error")
    axes[0].bar(idx + 0.18, f_norms, width=0.36, label=r"$\|f(x,u)\|$")
    axes[0].set_yscale("log")
    axes[0].set_xticks(idx)
    axes[0].set_xticklabels([str(i) for i in idx])
    axes[0].set_xlabel("condition index")
    axes[0].legend()
    axes[0].grid(True, which="both", ls=":", lw=0.4)
    axes[0].set_title("Absolute linearization error vs. field magnitude")

    rel = [
        e / n if n > 1e-9 else np.nan for e, n in zip(errors, f_norms)
    ]
    axes[1].bar(idx, rel)
    axes[1].set_yscale("log")
    axes[1].set_xticks(idx)
    axes[1].set_xticklabels([str(i) for i in idx])
    axes[1].set_xlabel("condition index")
    axes[1].set_ylabel("error / ||f||")
    axes[1].grid(True, which="both", ls=":", lw=0.4)
    axes[1].set_title(
        "Relative linearization error (case 4 omitted: f(x,u) = 0)"
    )
    fig.tight_layout()
    fig.savefig(os.path.join(FIG_DIR, "ex3_1_linearization_error.png"), dpi=200)
    plt.close(fig)

    # Figure: theta(t) for the six closed-loop simulations.
    fig, ax = plt.subplots(figsize=(9, 4.5))
    for s in sims:
        label = f"case {s['index']} (" + ("recovered" if s["recovered"] else "failed") + ")"
        ax.plot(s["t"], s["theta"], lw=1.4, label=label)
    ax.axhline(np.pi, color="black", ls="--", lw=0.8)
    ax.set_xlabel("time [s]")
    ax.set_ylabel(r"$\theta$ [rad]")
    ax.set_title("Nonlinear closed loop with the linear LQR controller (Q=I, R=1)")
    ax.legend(fontsize=8, ncol=2)
    ax.grid(True, ls=":", lw=0.4)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG_DIR, "ex3_1_closed_loop_theta.png"), dpi=200)
    plt.close(fig)

    results = {
        "A_lin": A_LIN.tolist(),
        "B_lin": B_LIN.tolist(),
        "Q": Q.tolist(),
        "R": R.tolist(),
        "K": K.tolist(),
        "closed_loop_eigenvalues": [
            [float(z.real), float(z.imag)] for z in eig_cl
        ],
        "conditions": [
            {
                k: v
                for k, v in s.items()
                if k not in ("t", "theta", "x")
            }
            for s in sims
        ],
        "recovered_state_indices": recovered,
    }
    with open("results_ex3_1.json", "w") as fh:
        json.dump(results, fh, indent=2)

    np.set_printoptions(precision=4, suppress=True)
    print("A_lin =\n", A_LIN)
    print("B_lin =\n", B_LIN)
    print("K =", K)
    print("closed-loop eigenvalues:", np.round(eig_cl, 4))
    print("\nidx  error        |f|       rel.err   recovered  final theta")
    for s in sims:
        rel_txt = (
            f"{s['relative_error']:9.4f}"
            if s["relative_error"] is not None
            else "      n/a"
        )
        print(
            f"{s['index']}   {s['error']:10.4f} {s['norm_f']:9.4f} "
            f"{rel_txt}   {str(s['recovered']):5s}     "
            f"{s['final_state'][1]:.4f}"
        )
    print("\nStates from which the system recovers:", recovered)


if __name__ == "__main__":
    main()
