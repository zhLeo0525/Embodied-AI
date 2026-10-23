"""Example 5.1 -- ZmpPlanner.

Reproduces the notebook example, checks the LIPM/ZMP identity, and studies
the sensitivity to the double-support duration and the feedback policy.

Run with the course virtual environment:

    /Users/leo/venvs/drake/bin/python example51_zmp.py
"""

import json
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from pydrake.all import PiecewisePolynomial, ZmpPlanner

HERE = os.path.dirname(os.path.abspath(__file__))


def desired_zmp_traj(footsteps, double_support_duration, single_support_duration):
    """Piecewise-linear ZMP through the foot centers (notebook 5.1)."""
    time_steps = [0.0]
    zmp_d = [footsteps[0]]
    time = 0.0

    time += single_support_duration
    time_steps.append(time)
    zmp_d.append(footsteps[0])

    for i in range(1, len(footsteps)):
        time += double_support_duration
        time_steps.append(time)
        zmp_d.append(footsteps[i])
        time += single_support_duration
        time_steps.append(time)
        zmp_d.append(footsteps[i])

    return PiecewisePolynomial.FirstOrderHold(time_steps, np.array(zmp_d).T)


def plan(footsteps, double_support, single_support, height=1.0, x0=None):
    if x0 is None:
        x0 = np.zeros(4)
    zmp_traj = desired_zmp_traj(footsteps, double_support, single_support)
    planner = ZmpPlanner()
    planner.Plan(zmp_traj, x0, height)
    return planner, zmp_traj


def sample(planner, zmp_traj, n=400):
    ts = np.linspace(0.0, zmp_traj.end_time(), n)
    com = np.array([planner.get_nominal_com(t) for t in ts])
    comd = np.array([planner.get_nominal_comd(t) for t in ts])
    comdd = np.array([planner.get_nominal_comdd(t) for t in ts])
    zmp = np.array([planner.get_desired_zmp(t) for t in ts])
    return ts, com, comd, comdd, zmp


def simulate_policy(planner, zmp_traj, x0, dt=1e-3):
    """Closed-loop simulation of the linear policy produced by the planner."""
    tf = zmp_traj.end_time()
    ts = np.arange(0.0, tf + dt / 2, dt)
    x = np.array(x0, dtype=float)
    xs = np.zeros((len(ts), 4))
    us = np.zeros((len(ts), 2))
    for i, t in enumerate(ts):
        xs[i] = x
        u = planner.ComputeOptimalCoMdd(t, x)
        us[i] = u
        # RK4 for the double integrator xdd = u
        def f(state):
            return np.concatenate((state[2:], u))

        k1 = f(x)
        k2 = f(x + dt / 2 * k1)
        k3 = f(x + dt / 2 * k2)
        k4 = f(x + dt * k3)
        x = x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    return ts, xs, us


def main():
    footsteps = np.array(
        [[0, -0.05], [0.5, 0.05], [1, -0.05], [1.5, 0.05], [2, -0.05], [2.5, 0.05]]
    )
    height = 1.0
    ds, ss = 0.5, 1.0

    planner, zmp_traj = plan(footsteps, ds, ss, height)
    ts, com, comd, comdd, zmp = sample(planner, zmp_traj)

    # The planner's own CoP map, y = c - z/g * cdd, is an identity.
    cop = np.array(
        [planner.comdd_to_cop(np.concatenate((com[i], comd[i])), comdd[i])
         for i in range(len(ts))]
    )
    identity_err = np.max(np.abs(cop - (com - height / 9.81 * comdd)))
    # Tracking error against the *desired* ZMP.  It is non-zero because the
    # objective trades ZMP tracking (Qy) against CoM acceleration (R).
    tracking_err = np.max(np.abs(zmp - (com - height / 9.81 * comdd)))

    # Sensitivity of the tracking error to the control weight R.
    r_study = {}
    for r in [1e-4, 1e-2, 0.1, 1.0]:
        p_r = ZmpPlanner()
        p_r.Plan(zmp_traj, np.zeros(4), height, 9.81,
                 np.eye(2), r * np.eye(2))
        t_r, c_r, cd_r, cdd_r, z_r = sample(p_r, zmp_traj)
        r_study[str(r)] = {
            "max_tracking_error": float(
                np.max(np.abs(z_r - (c_r - height / 9.81 * cdd_r)))
            ),
            "max_abs_acceleration": float(np.max(np.abs(cdd_r))),
        }

    # Closed loop under a velocity perturbation.
    x0 = np.zeros(4)
    x0_pert = np.array([0.0, 0.0, 0.35, -0.2])
    ts_sim, xs_nom, _ = simulate_policy(planner, zmp_traj, x0)
    _, xs_pert, us_pert = simulate_policy(planner, zmp_traj, x0_pert)
    final_dev_nom = np.linalg.norm(xs_nom[-1, :2] - com[-1])
    final_dev_pert = np.linalg.norm(xs_pert[-1, :2] - com[-1])

    # Timing sensitivity.
    timings = [0.2, 0.5, 0.8]
    sensitivity = {}
    for dsi in timings:
        p_i, z_i = plan(footsteps, dsi, ss, height)
        t_i, c_i, cd_i, _, pz_i = sample(p_i, z_i)
        sensitivity[str(dsi)] = {
            "T": float(z_i.end_time()),
            "max_abs_y": float(np.max(np.abs(c_i[:, 1]))),
            "max_abs_vy": float(np.max(np.abs(cd_i[:, 1]))),
            "max_abs_vx": float(np.max(np.abs(cd_i[:, 0]))),
            "max_zmp_y": float(np.max(np.abs(pz_i[:, 1]))),
            "x_final": float(c_i[-1, 0]),
        }

    results = {
        "identity_error": float(identity_err),
        "max_tracking_error_vs_desired": float(tracking_err),
        "T": float(zmp_traj.end_time()),
        "max_abs_com_y": float(np.max(np.abs(com[:, 1]))),
        "max_abs_com_vx": float(np.max(np.abs(comd[:, 0]))),
        "max_abs_com_vy": float(np.max(np.abs(comd[:, 1]))),
        "max_zmp_x": float(np.max(zmp[:, 0])),
        "feedback_final_error_nominal": float(final_dev_nom),
        "feedback_final_error_perturbed": float(final_dev_pert),
        "feedback_peak_velocity_error": float(
            np.max(np.linalg.norm(xs_pert[:, 2:] - xs_nom[:, 2:], axis=1))
        ),
        "timing_sensitivity": sensitivity,
        "R_sensitivity": r_study,
    }

    np.savez(
        os.path.join(HERE, "zmp_planner_data.npz"),
        footsteps=footsteps,
        t=ts,
        com=com,
        comd=comd,
        comdd=comdd,
        zmp=zmp,
        t_sim=ts_sim,
        x_nom=xs_nom,
        x_pert=xs_pert,
        u_pert=us_pert,
        height=height,
        double_support=ds,
        single_support=ss,
    )
    with open(os.path.join(HERE, "results_zmp.json"), "w") as f:
        json.dump(results, f, indent=2)

    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
