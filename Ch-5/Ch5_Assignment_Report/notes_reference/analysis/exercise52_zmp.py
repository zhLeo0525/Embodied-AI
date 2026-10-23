"""Exercise 5.2 -- Understanding the zero-moment point.

Numerical checks for the analytic solution of the foot / leg / bob model.

Geometry (origin at the heel H, +x toward the toe T):
    H = (0, 0), P = (L2, h), T = (L1 + L2, 0)
    bob = P + l (cos theta, sin theta)

Part (a,b,c,d): no actuator at P.  The massless leg is a two-force member, so
the ground reaction passes through P and the bob.  Its intersection with the
ground is the ZMP:

    x_zmp(theta) = L2 - h cot(theta).

Part (e,f): an ideal ankle actuator cancels the gravitational torque, so the
whole robot is quasi-static.  The ZMP is the vertical projection of the CoM:

    x_zmp(theta) = L2 + l cos(theta).

Run with:  /Users/leo/venvs/drake/bin/python exercise52_zmp.py
"""

import json
import os

import matplotlib

matplotlib.use("Agg")
import numpy as np
from scipy.integrate import solve_ivp

HERE = os.path.dirname(os.path.abspath(__file__))

m, g, l = 1.0, 9.81, 0.5
h, L1, L2 = 0.08, 0.12, 0.08


def zmp_free_leg(theta):
    """ZMP for the freely swinging leg (no ankle actuator)."""
    return L2 - h / np.tan(theta)


def zmp_actuated(theta):
    """ZMP with an ideal ankle actuator (quasi-static)."""
    return L2 + l * np.cos(theta)


def simulate_free_leg(theta0=np.pi / 2 + 0.1, theta_dot0=0.0, tmax=2.0):
    """Simulate the free pendulum about P and compare two ZMP computations."""
    def rhs(t, y):
        theta, omega = y
        return [omega, -(g / l) * np.cos(theta)]

    sol = solve_ivp(rhs, [0, tmax], [theta0, theta_dot0],
                    rtol=1e-10, atol=1e-12, dense_output=True)
    t = np.linspace(0, tmax, 1200)
    theta = sol.sol(t)[0]
    omega = sol.sol(t)[1]
    theta_dd = -(g / l) * np.cos(theta)

    # Bob kinematics.
    xb = L2 + l * np.cos(theta)
    yb = h + l * np.sin(theta)
    ax = -l * np.sin(theta) * theta_dd - l * np.cos(theta) * omega**2
    ay = l * np.cos(theta) * theta_dd - l * np.sin(theta) * omega**2
    # General ZMP formula x_z = x_bob - y_bob a_x / (a_y + g).
    zmp_general = xb - yb * ax / (ay + g)

    # Geometric line intersection: line through P and the bob at y = 0.
    zmp_line = L2 - h / np.tan(theta)

    return {
        "t": t,
        "theta": theta,
        "omega": omega,
        "xb": xb,
        "yb": yb,
        "zmp_general": zmp_general,
        "zmp_line": zmp_line,
        "zmp_formula": zmp_free_leg(theta),
    }


def main():
    theta0 = np.pi / 2 + 1e-3
    # Linearisation at the upright posture: theta_dd ~ (g/l) (theta - pi/2).
    lam = np.sqrt(g / l)

    theta_H = np.arctan(h / L2)
    theta_T = np.pi - np.arctan(h / L1)

    # Feasible range with an ideal ankle actuator.
    cos_min = -L2 / l
    cos_max = L1 / l
    theta_min = np.arccos(cos_max)
    theta_max = np.arccos(cos_min)

    # Design study: total foot length fixed, move the ankle.
    L_total = L1 + L2
    L1_grid = np.linspace(0.01, L_total - 0.01, 400)
    L2_grid = L_total - L1_grid
    # For each placement, the feasible range of cos(theta) is
    # [-L2/l, L1/l]; require both ends to be inside [-1, 1].
    feasible = (L1_grid <= l) & (L2_grid <= l)
    angle_range = np.where(
        feasible,
        np.arccos(np.clip(-L2_grid / l, -1, 1))
        - np.arccos(np.clip(L1_grid / l, -1, 1)),
        np.nan,
    )

    sim = simulate_free_leg(theta0=theta0, tmax=1.5)

    results = {
        "parameters": {"m": m, "g": g, "l": l, "h": h, "L1": L1, "L2": L2},
        "unstable_rate_sqrt_g_over_l": float(lam),
        "theta_H": float(theta_H),
        "theta_T": float(theta_T),
        "theta_range_no_actuator": [float(theta_H), float(theta_T)],
        "theta_range_actuator": [float(theta_min), float(theta_max)],
        "actuator_range_width": float(theta_max - theta_min),
        "optimal_L1_for_range": float(L1_grid[np.nanargmax(angle_range)]),
        "optimal_angle_range": float(np.nanmax(angle_range)),
        "zmp_formula_error": float(
            np.max(np.abs(sim["zmp_general"] - sim["zmp_formula"]))
        ),
        "line_formula_error": float(
            np.max(np.abs(sim["zmp_line"] - sim["zmp_formula"]))
        ),
    }

    np.savez(
        os.path.join(HERE, "exercise52_data.npz"),
        L1_grid=L1_grid,
        angle_range=angle_range,
        theta_range_actuator=np.array([theta_min, theta_max]),
        theta_H=theta_H,
        theta_T=theta_T,
        simulation_theta=sim["theta"],
        simulation_zmp=sim["zmp_formula"],
        simulation_t=sim["t"],
    )
    with open(os.path.join(HERE, "results_ex52.json"), "w") as f:
        json.dump(results, f, indent=2)

    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
