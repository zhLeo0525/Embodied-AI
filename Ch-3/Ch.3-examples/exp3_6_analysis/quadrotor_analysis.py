"""
Example 3.6 -- LQR for Quadrotors.

Part 1: planar quadrotor (underactuated/quadrotor2d.py)
    state  x = [x, z, theta, xdot, zdot, thetadot]
    input  u = [u1, u2]  (two propeller thrusts)

Part 2: 3D quadrotor (pydrake.examples.QuadrotorPlant)
    state  x = [x, y, z, roll, pitch, yaw, xdot, ydot, zdot, p, q, r]
    input  u = [u1, u2, u3, u4]

The script runs the LQR controllers, records closed-loop simulations, and
performs sensitivity studies on Q, R, initial attitude and mass.

Outputs (in the current directory):
    figures/fig1_planar_baseline.png
    figures/fig2_planar_sensitivity.png
    figures/fig3_quad3d_baseline.png
    figures/fig4_quad3d_sensitivity.png
    results.json
"""

import json
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from pydrake.all import (
    DiagramBuilder,
    LinearQuadraticRegulator,
    LogVectorOutput,
    Simulator,
)
from pydrake.systems.primitives import ConstantVectorSource
from pydrake.examples import QuadrotorPlant, StabilizingLQRController
from underactuated.quadrotor2d import Quadrotor2D

FIG_DIR = "figures"
os.makedirs(FIG_DIR, exist_ok=True)

PLANAR_BASE_Q = np.diag([10.0, 10.0, 10.0, 1.0, 1.0, 0.25 / 2.0 / np.pi])
PLANAR_BASE_R = np.array([[0.1, 0.05], [0.05, 0.1]])


# ---------------------------------------------------------------------------
# Planar quadrotor
# ---------------------------------------------------------------------------


def build_planar(q_pos_scale=1.0, r_scale=1.0, wind=None):
    plant = Quadrotor2D()

    context = plant.CreateDefaultContext()
    context.SetContinuousState(np.zeros(6))
    u_hover = np.array([plant.mass * plant.gravity / 2.0] * 2)
    plant.get_input_port(0).FixValue(context, u_hover)

    Q = PLANAR_BASE_Q.copy()
    Q[:3, :3] *= q_pos_scale
    R = PLANAR_BASE_R * r_scale

    lqr = LinearQuadraticRegulator(
        plant, context, Q, R, input_port_index=0
    )

    builder = DiagramBuilder()
    plant = builder.AddSystem(plant)
    controller = builder.AddSystem(lqr)
    builder.Connect(controller.get_output_port(0), plant.get_input_port(0))
    builder.Connect(plant.get_output_port(0), controller.get_input_port(0))
    if wind is not None:
        wind_source = builder.AddSystem(ConstantVectorSource(np.asarray(wind, dtype=float)))
        builder.Connect(wind_source.get_output_port(0), plant.get_input_port(1))
    state_log = LogVectorOutput(plant.get_output_port(0), builder)
    input_log = LogVectorOutput(controller.get_output_port(0), builder)
    diagram = builder.Build()

    return diagram, state_log, input_log, plant, lqr


def simulate(diagram, state_log, input_log, x0, t_final):
    simulator = Simulator(diagram)
    simulator.get_mutable_integrator().set_maximum_step_size(0.01)
    context = simulator.get_mutable_context()
    context.SetTime(0.0)
    context.SetContinuousState(np.asarray(x0, dtype=float))
    simulator.Initialize()
    simulator.AdvanceTo(t_final)
    diagram_context = simulator.get_context()
    t = state_log.FindLog(diagram_context).sample_times()
    x = state_log.FindLog(diagram_context).data().T
    u = input_log.FindLog(diagram_context).data().T
    return t, x, u


def planar_metrics(t, x, u):
    pos_err = float(np.hypot(x[-1, 0], x[-1, 1]))
    att_err = float(abs(x[-1, 2]))
    vel_err = float(np.linalg.norm(x[-1, 3:]))
    success = pos_err < 0.05 and att_err < 0.05 and vel_err < 0.2

    settled = np.maximum.reduce(
        [np.abs(x[:, 0]), np.abs(x[:, 1]), np.abs(x[:, 2])]
    )
    above = np.where(settled > 0.05)[0]
    settling_time = float(t[above[-1]]) if len(above) else 0.0

    return {
        "final_position_error": pos_err,
        "final_attitude_error": att_err,
        "final_velocity_error": vel_err,
        "settling_time": settling_time,
        "max_thrust": float(np.max(np.abs(u))),
        "success": bool(success),
        "final_state": x[-1].tolist(),
    }


def planar_sweep():
    results = {}

    # Baseline: several random initial conditions.
    rng = np.random.default_rng(0)
    baseline = []
    for _ in range(10):
        x0 = 0.1 * rng.standard_normal(6)
        diagram, slog, ulog, _, _ = build_planar()
        t, x, u = simulate(diagram, slog, ulog, x0, 4.0)
        m = planar_metrics(t, x, u)
        m["x0"] = x0.tolist()
        baseline.append(m)
    results["baseline_random"] = baseline
    results["baseline_success_rate"] = float(
        np.mean([m["success"] for m in baseline])
    )

    # Reference trajectory for plotting.
    x0_ref = np.array([0.0, 0.0, 0.2, 0.0, 0.0, 0.0])
    diagram, slog, ulog, _, _ = build_planar()
    t, x, u = simulate(diagram, slog, ulog, x0_ref, 4.0)
    results["reference"] = {
        "x0": x0_ref.tolist(),
        "t": t.tolist(),
        "state": x.tolist(),
        "input": u.tolist(),
        "metrics": planar_metrics(t, x, u),
    }

    def sweep(name, values, build_kwargs, x0):
        rows = []
        for value in values:
            kwargs = dict(build_kwargs)
            kwargs[name] = value
            diagram, slog, ulog, _, _ = build_planar(**kwargs)
            t, x, u = simulate(diagram, slog, ulog, x0, 4.0)
            row = {"value": value, **planar_metrics(t, x, u)}
            rows.append(row)
        return rows

    x0 = np.array([0.0, 0.0, 0.2, 0.0, 0.0, 0.0])
    results["q_pos_sweep"] = sweep(
        "q_pos_scale", [0.1, 0.5, 1.0, 2.0, 10.0], {}, x0
    )
    results["r_sweep"] = sweep(
        "r_scale", [0.01, 0.1, 1.0, 10.0], {}, x0
    )
    wind_rows = []
    for wind_x in [0.0, 0.25, 0.5, 1.0, 2.0]:
        diagram, slog, ulog, _, _ = build_planar(wind=[wind_x, 0.0])
        t, x, u = simulate(diagram, slog, ulog, np.zeros(6), 6.0)
        wind_rows.append(
            {"value": wind_x, **planar_metrics(t, x, u)}
        )
    results["wind_sweep"] = wind_rows

    tilt_rows = []
    for theta0 in [0.05, 0.1, 0.2, 0.3, 0.5, 0.7]:
        x0_tilt = np.array([0.0, 0.0, theta0, 0.0, 0.0, 0.0])
        diagram, slog, ulog, _, _ = build_planar()
        t, x, u = simulate(diagram, slog, ulog, x0_tilt, 4.0)
        tilt_rows.append({"value": theta0, **planar_metrics(t, x, u)})
    results["tilt_sweep"] = tilt_rows

    return results


# ---------------------------------------------------------------------------
# 3D quadrotor
# ---------------------------------------------------------------------------


def build_quad3d(q_pos_scale=1.0, r_scale=1.0, use_stabilizing_controller=False):
    plant = QuadrotorPlant()
    mass = plant.m()
    gravity = plant.g()
    x_star = np.zeros(12)
    x_star[2] = 1.0
    u_hover = np.array([mass * gravity / 4.0] * 4)

    builder = DiagramBuilder()
    plant = builder.AddSystem(plant)

    if use_stabilizing_controller:
        controller = builder.AddSystem(
            StabilizingLQRController(plant, [0, 0, 1])
        )
    else:
        context = plant.CreateDefaultContext()
        context.SetContinuousState(x_star)
        plant.get_input_port(0).FixValue(context, u_hover)
        Q = np.diag(np.concatenate(([10.0 * q_pos_scale] * 6, [1.0] * 6)))
        R = np.eye(4) * r_scale
        controller = builder.AddSystem(
            LinearQuadraticRegulator(plant, context, Q, R)
        )

    builder.Connect(controller.get_output_port(0), plant.get_input_port(0))
    builder.Connect(plant.get_output_port(0), controller.get_input_port(0))
    state_log = LogVectorOutput(plant.get_output_port(0), builder)
    input_log = LogVectorOutput(controller.get_output_port(0), builder)
    diagram = builder.Build()
    return diagram, state_log, input_log, plant


def quad3d_metrics(t, x, u):
    pos_err = float(np.linalg.norm(x[-1, :3] - np.array([0.0, 0.0, 1.0])))
    att_err = float(np.max(np.abs(x[-1, 3:6])))
    vel_err = float(np.linalg.norm(x[-1, 6:9]))
    success = pos_err < 0.1 and att_err < 0.1 and vel_err < 0.5

    pos_dev = np.linalg.norm(x[:, :3] - np.array([0.0, 0.0, 1.0]), axis=1)
    att_dev = np.max(np.abs(x[:, 3:6]), axis=1)
    above = np.where(np.maximum(pos_dev, att_dev) > 0.05)[0]
    settling_time = float(t[above[-1]]) if len(above) else 0.0

    return {
        "final_position_error": pos_err,
        "final_attitude_error": att_err,
        "final_velocity_error": vel_err,
        "peak_position_deviation": float(np.max(pos_dev)),
        "peak_attitude_deviation": float(np.max(att_dev)),
        "settling_time": settling_time,
        "max_thrust": float(np.max(np.abs(u))),
        "success": bool(success),
        "final_state": x[-1].tolist(),
    }


def quad3d_sweep():
    results = {}
    x0 = np.array(
        [0.0, 0.0, 1.0, 0.2, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    )

    # Official controller from the textbook example.
    diagram, slog, ulog, _ = build_quad3d(use_stabilizing_controller=True)
    t, x, u = simulate(diagram, slog, ulog, x0, 5.0)
    results["baseline_official"] = {
        "x0": x0.tolist(),
        "t": t.tolist(),
        "state": x.tolist(),
        "input": u.tolist(),
        "metrics": quad3d_metrics(t, x, u),
    }

    # The same LQR with Q/R from the notebook.
    diagram, slog, ulog, _ = build_quad3d()
    t, x, u = simulate(diagram, slog, ulog, x0, 5.0)
    results["reference"] = {
        "x0": x0.tolist(),
        "t": t.tolist(),
        "state": x.tolist(),
        "input": u.tolist(),
        "metrics": quad3d_metrics(t, x, u),
    }

    def sweep(kw_name, values, x0_local):
        rows = []
        for value in values:
            diagram, slog, ulog, _ = build_quad3d(**{kw_name: value})
            t, x, u = simulate(diagram, slog, ulog, x0_local, 5.0)
            rows.append({"value": value, **quad3d_metrics(t, x, u)})
        return rows

    results["q_pos_sweep"] = sweep(
        "q_pos_scale", [0.1, 0.5, 1.0, 2.0, 10.0], x0
    )
    results["r_sweep"] = sweep(
        "r_scale", [0.01, 0.1, 1.0, 10.0], x0
    )

    tilt_rows = []
    for tilt in [0.05, 0.1, 0.2, 0.3, 0.5]:
        x0_tilt = x0.copy()
        x0_tilt[3] = tilt
        x0_tilt[4] = 0.5 * tilt
        diagram, slog, ulog, _ = build_quad3d()
        t, x, u = simulate(diagram, slog, ulog, x0_tilt, 5.0)
        tilt_rows.append({"value": tilt, **quad3d_metrics(t, x, u)})
    results["tilt_sweep"] = tilt_rows

    return results


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------


def plot_planar(results):
    ref = results["planar"]["reference"]
    t = np.array(ref["t"])
    x = np.array(ref["state"])
    u = np.array(ref["input"])

    fig, axes = plt.subplots(2, 2, figsize=(11, 7), sharex=True)
    axes[0, 0].plot(t, x[:, 0], label=r"$x$")
    axes[0, 0].plot(t, x[:, 1], label=r"$z$")
    axes[0, 0].set_ylabel("position [m]")
    axes[0, 0].legend()
    axes[0, 1].plot(t, x[:, 2], color="tab:red")
    axes[0, 1].set_ylabel(r"$\theta$ [rad]")
    axes[1, 0].plot(t, u[:, 0], label=r"$u_1$")
    axes[1, 0].plot(t, u[:, 1], label=r"$u_2$")
    axes[1, 0].set_ylabel("thrust [N]")
    axes[1, 0].set_xlabel("time [s]")
    axes[1, 0].legend()
    axes[1, 1].plot(t, x[:, 3], label=r"$\dot x$")
    axes[1, 1].plot(t, x[:, 4], label=r"$\dot z$")
    axes[1, 1].plot(t, x[:, 5], label=r"$\dot\theta$")
    axes[1, 1].set_ylabel("velocity")
    axes[1, 1].set_xlabel("time [s]")
    axes[1, 1].legend(fontsize=8)
    for ax in axes.flat:
        ax.grid(True, ls=":", lw=0.4)
    fig.suptitle(
        r"Planar quadrotor LQR: $x_0=(0,0,0.2,0,0,0)$", fontsize=13
    )
    fig.tight_layout()
    fig.savefig(os.path.join(FIG_DIR, "fig1_planar_baseline.png"), dpi=200)
    plt.close(fig)


def _sweep_plot(ax, rows, xlabel, title):
    vals = [r["value"] for r in rows]
    settle = [r["settling_time"] for r in rows]
    thrust = [r["max_thrust"] for r in rows]
    ax.plot(vals, settle, "o-", color="tab:blue", label="settling time [s]")
    ax.set_xlabel(xlabel)
    ax.set_ylabel("settling time [s]", color="tab:blue")
    ax.tick_params(axis="y", labelcolor="tab:blue")
    ax2 = ax.twinx()
    ax2.plot(vals, thrust, "s--", color="tab:orange", label="max thrust [N]")
    ax2.set_ylabel("max thrust [N]", color="tab:orange")
    ax2.tick_params(axis="y", labelcolor="tab:orange")
    ax.set_title(title)
    ax.grid(True, ls=":", lw=0.4)
    for r in rows:
        ax.annotate(
            "" if r["success"] else "×",
            (r["value"], r["settling_time"]),
            color="red",
            fontsize=12,
        )


def _wind_plot(ax, rows, title):
    vals = [r["value"] for r in rows]
    err = [r["final_position_error"] for r in rows]
    thrust = [r["max_thrust"] for r in rows]
    ax.plot(vals, err, "o-", color="tab:blue", label="final position error [m]")
    ax.set_xlabel("horizontal wind force [N]")
    ax.set_ylabel("final position error [m]", color="tab:blue")
    ax.tick_params(axis="y", labelcolor="tab:blue")
    ax2 = ax.twinx()
    ax2.plot(vals, thrust, "s--", color="tab:orange", label="max thrust [N]")
    ax2.set_ylabel("max thrust [N]", color="tab:orange")
    ax2.tick_params(axis="y", labelcolor="tab:orange")
    ax.set_title(title)
    ax.grid(True, ls=":", lw=0.4)


def plot_planar_sensitivity(results):
    p = results["planar"]
    fig, axes = plt.subplots(2, 2, figsize=(11, 7))
    _sweep_plot(axes[0, 0], p["q_pos_sweep"], "position weight scale", "(a) Q position weight")
    _sweep_plot(axes[0, 1], p["r_sweep"], "R scale", "(b) R scale")
    _wind_plot(axes[1, 0], p["wind_sweep"], "(c) constant horizontal wind")
    _sweep_plot(axes[1, 1], p["tilt_sweep"], r"initial $\theta_0$ [rad]", "(d) initial tilt")
    fig.suptitle(
        "Planar quadrotor: sensitivity of LQR performance (× = failed)",
        fontsize=13,
    )
    fig.tight_layout()
    fig.savefig(os.path.join(FIG_DIR, "fig2_planar_sensitivity.png"), dpi=200)
    plt.close(fig)


def plot_quad3d(results):
    ref = results["quad3d"]["reference"]
    t = np.array(ref["t"])
    x = np.array(ref["state"])
    u = np.array(ref["input"])

    fig, axes = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True)
    axes[0].plot(t, x[:, 0], label=r"$x$")
    axes[0].plot(t, x[:, 1], label=r"$y$")
    axes[0].plot(t, x[:, 2], label=r"$z$")
    axes[0].axhline(1.0, color="k", ls=":", lw=0.8)
    axes[0].set_ylabel("position [m]")
    axes[0].legend()
    axes[1].plot(t, x[:, 3], label=r"roll")
    axes[1].plot(t, x[:, 4], label=r"pitch")
    axes[1].plot(t, x[:, 5], label=r"yaw")
    axes[1].set_ylabel("attitude [rad]")
    axes[1].set_xlabel("time [s]")
    axes[1].legend()
    for ax in axes:
        ax.grid(True, ls=":", lw=0.4)
    fig.suptitle(
        r"3D quadrotor LQR: $x_0=(0,0,1,0.2,0.1,0,\ldots)$", fontsize=13
    )
    fig.tight_layout()
    fig.savefig(os.path.join(FIG_DIR, "fig3_quad3d_baseline.png"), dpi=200)
    plt.close(fig)


def plot_quad3d_sensitivity(results):
    p = results["quad3d"]
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.2))
    for ax, key, label in [
        (axes[0], "q_pos_sweep", "position/attitude weight scale"),
        (axes[1], "r_sweep", "R scale"),
        (axes[2], "tilt_sweep", r"initial tilt [rad]"),
    ]:
        rows = p[key]
        vals = [r["value"] for r in rows]
        err = [max(r["settling_time"], 1e-3) for r in rows]
        thrust = [r["max_thrust"] for r in rows]
        ax.plot(vals, err, "o-", color="tab:blue", label="settling time [s]")
        ax.set_xlabel(label)
        ax.set_ylabel("settling time [s]", color="tab:blue")
        ax.tick_params(axis="y", labelcolor="tab:blue")
        ax2 = ax.twinx()
        ax2.plot(vals, thrust, "s--", color="tab:orange", label="max thrust [N]")
        ax2.set_ylabel("max thrust [N]", color="tab:orange")
        ax2.tick_params(axis="y", labelcolor="tab:orange")
        ax.grid(True, ls=":", lw=0.4)
    fig.suptitle("3D quadrotor: LQR sensitivity", fontsize=13)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG_DIR, "fig4_quad3d_sensitivity.png"), dpi=200)
    plt.close(fig)


def main():
    planar = planar_sweep()
    quad3d = quad3d_sweep()
    results = {"planar": planar, "quad3d": quad3d}

    with open("results.json", "w") as f:
        json.dump(results, f, indent=2)

    plot_planar(results)
    plot_planar_sensitivity(results)
    plot_quad3d(results)
    plot_quad3d_sensitivity(results)

    print("=== Planar quadrotor ===")
    print("random baseline success rate:", planar["baseline_success_rate"])
    for name in ["q_pos_sweep", "r_sweep", "wind_sweep", "tilt_sweep"]:
        print(f"\n{name}:")
        for row in planar[name]:
            print(
                f"  value={row['value']:.3g}  success={row['success']}  "
                f"settle={row['settling_time']:.3f}s  maxu={row['max_thrust']:.3f}N"
            )

    print("\n=== 3D quadrotor ===")
    print("official controller:", quad3d["baseline_official"]["metrics"])
    for name in ["q_pos_sweep", "r_sweep", "tilt_sweep"]:
        print(f"\n{name}:")
        for row in quad3d[name]:
            print(
                f"  value={row['value']:.3g}  success={row['success']}  "
                f"settle={row['settling_time']:.3f}s  "
                f"maxu={row['max_thrust']:.3f}N"
            )


if __name__ == "__main__":
    main()
