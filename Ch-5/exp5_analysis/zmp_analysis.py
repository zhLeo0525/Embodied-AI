"""Analysis of Example 5.1: Drake's ZmpPlanner.

Produces:
  fig5_1_zmp_plan.png       nominal ZMP / CoM plan
  fig5_2_zmp_timing.png     CoM trajectory vs. footstep timing
  fig5_3_zmp_feedback.png   feedback policy rejects a velocity disturbance
  results_zmp.json          numerical summary
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
from pydrake.all import PiecewisePolynomial, ZmpPlanner

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "figures")
os.makedirs(OUT, exist_ok=True)

HEIGHT = 1.0
FOOTSTEPS = np.array(
    [[0, -0.05], [0.5, 0.05], [1, -0.05], [1.5, 0.05], [2, -0.05], [2.5, 0.05]]
)


def desired_zmp_traj(footsteps, double_support, single_support):
    time_steps, zmp = [], []
    t = 0.0
    time_steps += [t, t + single_support]
    zmp += [footsteps[0], footsteps[0]]
    t += single_support
    for i in range(1, len(footsteps)):
        t += double_support
        time_steps.append(t)
        zmp.append(footsteps[i])
        t += single_support
        time_steps.append(t)
        zmp.append(footsteps[i])
    return PiecewisePolynomial.FirstOrderHold(time_steps, np.array(zmp).T)


def plan(footsteps, double_support, single_support, x0=None):
    if x0 is None:
        x0 = np.zeros(4)
    zmp_traj = desired_zmp_traj(footsteps, double_support, single_support)
    planner = ZmpPlanner()
    planner.Plan(zmp_traj, x0, HEIGHT)
    return planner, zmp_traj


def rollout(planner, zmp_traj, x0, n=800):
    """Simulate the closed-loop LIPM with the planner's feedback policy."""
    ts = np.linspace(0.0, zmp_traj.end_time(), n)
    x = np.array(x0, dtype=float)
    com, zmp, err = [], [], []
    for t in ts:
        cdd = planner.ComputeOptimalCoMdd(t, x)
        com.append(x[:2].copy())
        zmp.append(planner.comdd_to_cop(x, cdd))
        err.append(planner.get_desired_zmp(t) - zmp[-1])
        x[2:] += cdd * (ts[1] - ts[0])
        x[:2] += x[2:] * (ts[1] - ts[0])
    return ts, np.array(com), np.array(zmp), np.array(err)


def metrics(planner, zmp_traj, ts, com, zmp, error):
    return {
        "duration_s": float(zmp_traj.end_time()),
        "peak_abs_com_y_m": float(np.max(np.abs(com[:, 1]))),
        "max_zmp_error_m": float(np.max(np.linalg.norm(error, axis=1))),
        "rms_zmp_error_m": float(np.sqrt(np.mean(np.sum(error**2, axis=1)))),
        "final_com_x_m": float(com[-1, 0]),
    }


def main():
    results = {}

    # ---- nominal plan -------------------------------------------------
    planner, traj = plan(FOOTSTEPS, 0.5, 1.0)
    ts, com, zmp, err = rollout(planner, traj, np.zeros(4))
    zmp_d = np.array([planner.get_desired_zmp(t) for t in ts])
    results["nominal"] = metrics(planner, traj, ts, com, zmp, err)

    fig = plt.figure(figsize=(13, 7))
    gs = fig.add_gridspec(2, 2, height_ratios=[1.0, 1.25])
    ax = fig.add_subplot(gs[1, :])
    axes = [fig.add_subplot(gs[0, 0]), fig.add_subplot(gs[0, 1])]
    ax.plot(FOOTSTEPS[:, 0], FOOTSTEPS[:, 1], "ks--", ms=8, label="footsteps")
    ax.plot(zmp_d[:, 0], zmp_d[:, 1], "-", color="tab:green", lw=2,
            label="desired ZMP")
    ax.plot(zmp[:, 0], zmp[:, 1], "--", color="tab:blue", lw=1.4,
            label="realized ZMP")
    ax.plot(com[:, 0], com[:, 1], "-", color="tab:red", lw=2, label="CoM")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    ax.set_title("x-y plane")

    for k, (ax, label) in enumerate(zip(axes, ["x", "y"])):
        ax.plot(ts, com[:, k], color="tab:red", lw=2, label=f"CoM {label}")
        ax.plot(ts, zmp_d[:, k], color="tab:green", lw=2, label="desired ZMP")
        ax.plot(ts, zmp[:, k], "--", color="tab:blue", lw=1.2,
                label="realized ZMP")
        ax.set_xlabel("time [s]")
        ax.set_ylabel(f"{label} [m]")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
        ax.set_title(f"{label}(t)")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig5_1_zmp_plan.png"), dpi=200)
    plt.close(fig)

    # ---- timing sweep -------------------------------------------------
    timings = [
        (0.2, 1.4, "DS 0.2 / SS 1.4"),
        (0.5, 1.0, "DS 0.5 / SS 1.0 (nominal)"),
        (1.0, 0.5, "DS 1.0 / SS 0.5"),
        (1.0, 1.0, "DS 1.0 / SS 1.0"),
    ]
    timing_rows = []
    fig, ax = plt.subplots(figsize=(9, 4.2))
    for ds, ss, label in timings:
        p, tr = plan(FOOTSTEPS, ds, ss)
        t_i, com_i, zmp_i, err_i = rollout(p, tr, np.zeros(4))
        ax.plot(t_i, com_i[:, 0], lw=2, label=label)
        m = metrics(p, tr, t_i, com_i, zmp_i, err_i)
        timing_rows.append({"double_support_s": ds, "single_support_s": ss,
                            "label": label, **m})
    ax.set_xlabel("time [s]")
    ax.set_ylabel("CoM x [m]")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=9)
    ax.set_title("Effect of footstep timing")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig5_2_zmp_timing.png"), dpi=200)
    plt.close(fig)
    results["timing_sweep"] = timing_rows

    # ---- feedback policy: velocity disturbance ------------------------
    # The nominal plan uses x0 = 0.  Here the robot starts with a forward
    # velocity perturbation; the planner's feedback law should reject it.
    p_nom, tr_nom = plan(FOOTSTEPS, 0.5, 1.0)
    t_nom, com_nom, _, _ = rollout(p_nom, tr_nom, np.zeros(4))
    disturbances = [
        (np.array([0.0, 0.0, 0.0, 0.0]), "nominal"),
        (np.array([0.0, 0.0, 0.30, 0.0]), "push +0.30 m/s in x"),
        (np.array([0.0, 0.0, -0.30, 0.0]), "push -0.30 m/s in x"),
        (np.array([0.05, 0.03, 0.0, 0.0]), "CoM offset (5, 3) cm"),
    ]
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.2))
    feedback_rows = []
    for x0, label in disturbances:
        t_i, com_i, _, _ = rollout(p_nom, tr_nom, x0)
        axes[0].plot(t_i, com_i[:, 0] - com_nom[:, 0], lw=1.8, label=label)
        axes[1].plot(t_i, com_i[:, 1] - com_nom[:, 1], lw=1.8, label=label)
        feedback_rows.append({
            "case": label,
            "max_x_deviation_m": float(np.max(np.abs(com_i[:, 0] - com_nom[:, 0]))),
            "final_x_deviation_m": float(com_i[-1, 0] - com_nom[-1, 0]),
            "final_y_deviation_m": float(com_i[-1, 1] - com_nom[-1, 1]),
        })
    for ax, label in zip(axes, ["x", "y"]):
        ax.set_xlabel("time [s]")
        ax.set_ylabel(f"CoM {label} deviation [m]")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
        ax.set_title(f"{label} deviation from nominal plan")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig5_3_zmp_feedback.png"), dpi=200)
    plt.close(fig)
    results["feedback"] = feedback_rows

    with open(os.path.join(HERE, "results_zmp.json"), "w") as f:
        json.dump(results, f, indent=2)

    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
