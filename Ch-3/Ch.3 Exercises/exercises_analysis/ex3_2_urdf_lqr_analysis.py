"""
Exercise 3.2 -- extra analysis for the single/double pendulum cart-pole.

The URDF strings and the equilibrium points are read directly from
../cartpoles_urdf.ipynb, so this script always analyses exactly the model
that is submitted with the notebook.

It reports:
  * the LQR gain K and the closed-loop eigenvalues,
  * a 5 s closed-loop simulation from the initial conditions used in the
    notebook,
  * the state/input histories (figures/ex3_2_*_balancing.png),
  * results_ex3_2.json.
"""

import contextlib
import io
import json
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    Linearize,
    LinearQuadraticRegulator,
    Parser,
    Simulator,
)
from pydrake.systems.primitives import LogVectorOutput
from scipy.linalg import solve_continuous_are

FIG_DIR = "figures"
os.makedirs(FIG_DIR, exist_ok=True)


def load_notebook_symbols():
    """Execute the pure-string cells of the URDF notebook."""
    nb = json.load(open("../cartpoles_urdf.ipynb"))
    cells = nb["cells"]
    ns = {"np": np}
    # base_urdf, pendulum_link, base_joint, pendulum_joint, transmission,
    # single_pendulum_urdf, second_pendulum_link/joint + double_pendulum_urdf,
    # x_star_single_pendulum, x_star_double_pendulum.
    with contextlib.redirect_stdout(io.StringIO()):
        for i in [12, 14, 17, 19, 21, 23, 30, 27, 34]:
            src = "".join(cells[i]["source"])
            exec(compile(src, f"<cell {i}>", "exec"), ns)
    return ns


def build_diagram(urdf, x_star, Q, R):
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    Parser(plant).AddModelsFromString(urdf, "urdf")
    plant.Finalize()
    plant.set_name("cartpole")

    context = plant.CreateDefaultContext()
    context.get_mutable_continuous_state_vector().SetFromVector(x_star)
    plant.get_actuation_input_port().FixValue(context, [0])
    input_i = plant.get_actuation_input_port().get_index()

    lqr = LinearQuadraticRegulator(
        plant, context, Q, R, input_port_index=input_i
    )
    lqr = builder.AddSystem(lqr)
    builder.Connect(plant.get_state_output_port(), lqr.get_input_port(0))
    builder.Connect(lqr.get_output_port(0), plant.get_actuation_input_port())

    state_logger = LogVectorOutput(plant.get_state_output_port(), builder)
    ctrl_logger = LogVectorOutput(lqr.get_output_port(0), builder)

    diagram = builder.Build()

    # Continuous-time linearization used by LQR, and the gain from the CARE.
    linear = Linearize(
        plant,
        context,
        input_port_index=input_i,
        output_port_index=plant.get_state_output_port().get_index(),
    )
    A = linear.A()
    B = linear.B()
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.solve(R, B.T @ P)
    eig_cl = np.linalg.eigvals(A - B @ K)

    return (
        diagram,
        state_logger,
        ctrl_logger,
        A,
        B,
        K,
        eig_cl,
        plant.num_continuous_states(),
    )


def simulate(diagram, state_logger, ctrl_logger, x0, sim_time=5.0):
    simulator = Simulator(diagram)
    simulator.get_mutable_integrator().set_fixed_step_mode(True)
    simulator.get_mutable_integrator().set_maximum_step_size(1e-3)

    context = simulator.get_mutable_context()
    context.SetTime(0.0)
    context.SetContinuousState(np.asarray(x0, dtype=float))

    simulator.Initialize()
    simulator.AdvanceTo(sim_time)

    diagram_context = simulator.get_context()
    states = state_logger.FindLog(diagram_context)
    controls = ctrl_logger.FindLog(diagram_context)
    return states.sample_times(), states.data().T, controls.data().T


def plot_result(t, x, u, n, title, filename):
    fig, axes = plt.subplots(n, 1, figsize=(9, 1.9 * n), sharex=True)
    if n == 4:
        labels = [r"$x$", r"$\theta_0$", r"$\dot x$", r"$\dot\theta_0$"]
    else:
        labels = [
            r"$x$",
            r"$\theta_0$",
            r"$\theta_1$",
            r"$\dot x$",
            r"$\dot\theta_0$",
            r"$\dot\theta_1$",
        ]
    for i in range(n):
        axes[i].plot(t, x[:, i], lw=1.4)
        axes[i].set_ylabel(labels[i])
        axes[i].grid(True, ls=":", lw=0.4)
    axes[-1].set_xlabel("time [s]")
    axes[0].set_title(title)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG_DIR, filename), dpi=200)
    plt.close(fig)


def analyse(name, urdf, x_star, Q, R, x0, filename):
    (
        diagram,
        state_logger,
        ctrl_logger,
        A,
        B,
        K,
        eig_cl,
        n,
    ) = build_diagram(urdf, x_star, Q, R)

    t, x, u = simulate(diagram, state_logger, ctrl_logger, x0)

    plot_result(
        t,
        x,
        u,
        n,
        f"{name}: LQR closed loop from x0 = {np.round(x0, 2)}",
        filename,
    )

    final = x[-1]
    angle_err = np.abs(final[1:n // 2] - np.pi)
    result = {
        "name": name,
        "x_star": list(map(float, x_star)),
        "x0": list(map(float, x0)),
        "A": A.tolist(),
        "B": B.tolist(),
        "K": K.tolist(),
        "closed_loop_eigenvalues": [
            [float(z.real), float(z.imag)] for z in eig_cl
        ],
        "final_state": final.tolist(),
        "final_angle_error_to_pi": angle_err.tolist(),
        "max_abs_force": float(np.max(np.abs(u))),
        "final_time": float(t[-1]),
        "num_states": int(n),
    }
    return result


def main():
    ns = load_notebook_symbols()

    x_star_single = np.array(ns["x_star_single_pendulum"], dtype=float)
    x_star_double = np.array(ns["x_star_double_pendulum"], dtype=float)

    Q_single = np.diag((10.0, 10.0, 1.0, 1.0))
    Q_double = np.diag((10.0, 10.0, 10.0, 1.0, 1.0, 1.0))
    R = np.eye(1)

    single = analyse(
        "single pendulum",
        ns["single_pendulum_urdf"],
        x_star_single,
        Q_single,
        R,
        np.array([0.0, 0.9 * np.pi, 0.0, 0.0]),
        "ex3_2_single_balancing.png",
    )
    double = analyse(
        "double pendulum",
        ns["double_pendulum_urdf"],
        x_star_double,
        Q_double,
        R,
        np.array([-2.0, 0.96 * np.pi, 0.93 * np.pi, 0.0, 0.0, 0.0]),
        "ex3_2_double_balancing.png",
    )

    results = {"single": single, "double": double}
    with open("results_ex3_2.json", "w") as f:
        json.dump(results, f, indent=2)

    for r in [single, double]:
        print(f"\n=== {r['name']} ===")
        print("K =", np.round(np.array(r["K"]), 4))
        print(
            "closed-loop eigenvalues:",
            np.round(
                np.array(r["closed_loop_eigenvalues"])[:, 0]
                + 1j * np.array(r["closed_loop_eigenvalues"])[:, 1],
                4,
            ),
        )
        print("final state:", np.round(r["final_state"], 4))
        print("angle error to pi:", np.round(r["final_angle_error_to_pi"], 5))
        print("max |force|:", np.round(r["max_abs_force"], 3), "N")


if __name__ == "__main__":
    main()
