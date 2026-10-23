"""Figures for the Manipulation Chapter 1 report."""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from pydrake.all import *
from pathlib import Path

FIG = "figures"
os.makedirs(FIG, exist_ok=True)


def pendulum_plot():
    g = 9.81

    def f(t, x, tau):
        return [x[1], -g * np.sin(x[0]) + tau]

    cases = [
        ("Verification 1: $x_0=(0.15,0)$, $\\tau=0$", [0.15, 0.0], 0.0, 2.5),
        ("Verification 2: $x_0=(-0.1,0.2)$, $\\tau=0.5$", [-0.1, 0.2], 0.5, 1.8),
    ]
    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    for ax, (title, x0, tau, T) in zip(axes, cases):
        sol = solve_ivp(f, [0, T], x0, args=(tau,), rtol=1e-10, atol=1e-12,
                        max_step=0.001)
        ax.plot(sol.t, sol.y[0], lw=2)
        ax.set_xlabel("time [s]")
        ax.set_ylabel(r"$\theta$ [rad]")
        ax.set_title(title)
        ax.grid(True, ls=":", lw=0.4)
        ax.annotate(f"$\\theta({T})={sol.y[0,-1]:.4f}$",
                    xy=(sol.t[-1], sol.y[0, -1]),
                    xytext=(-80, 20), textcoords="offset points",
                    arrowprops=dict(arrowstyle="->"))
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig_pendulum.png"), dpi=200)
    plt.close(fig)


class SimpleController(LeafSystem):
    def __init__(self, gain, q_desired):
        LeafSystem.__init__(self)
        self.gain = gain
        self.q_desired = np.asarray(q_desired, dtype=float)
        self.input_port = self.DeclareVectorInputPort("state", 14)
        self.DeclareVectorOutputPort("tau", 7, self.ComputeTorque)

    def ComputeTorque(self, context, output):
        q = self.input_port.Eval(context)[:7]
        output.SetFromVector(-self.gain * (q - self.q_desired))


def iiwa_plot():
    model = str((Path.cwd() / "models" / "iiwa14_primitive_collision.urdf").resolve())
    builder = DiagramBuilder()
    plant, sg = AddMultibodyPlantSceneGraph(builder, time_step=1e-4)
    (iiwa,) = Parser(plant).AddModels(model)
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("iiwa_link_0", iiwa))
    plant.Finalize()
    ctrl = builder.AddSystem(SimpleController(120.0, np.zeros(7)))
    builder.Connect(plant.get_state_output_port(iiwa), ctrl.input_port)
    builder.Connect(ctrl.get_output_port(0), plant.get_actuation_input_port(iiwa))
    logger = LogVectorOutput(plant.get_state_output_port(iiwa), builder)
    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyMutableContextFromRoot(context)
    plant.SetPositions(plant_context, iiwa, np.array([0.2, 0.2, 0.2, 0, 0, 0, 0]))
    sim = Simulator(diagram, context)
    sim.set_target_realtime_rate(0.0)
    sim.Initialize()
    sim.AdvanceTo(10.0)
    log = logger.FindLog(context)
    q = log.data()[:7]
    fig, ax = plt.subplots(figsize=(9, 4.5))
    for i in range(7):
        ax.plot(log.sample_times(), q[i], label=f"q{i+1}")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("joint position [rad]")
    ax.set_title("IIWA14 under P control, $q_d=0$, gain 120")
    ax.grid(True, ls=":", lw=0.4)
    ax.legend(ncol=4, fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig_iiwa.png"), dpi=200)
    plt.close(fig)
    print("IIWA final:", np.round(plant.GetPositions(plant_context), 5))


if __name__ == "__main__":
    pendulum_plot()
    iiwa_plot()
