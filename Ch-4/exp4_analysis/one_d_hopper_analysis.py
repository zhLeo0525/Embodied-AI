"""Section 4.6 -- one-dimensional hopper: closed-loop hopping and sensitivity.

The supplied notebook code (parameter helper, Spring, block diagram, energy,
controller) is executed from the notebook; the visualizer recording is
skipped for speed.
"""

import json
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from pydrake.all import Simulator

FIG = "figures"
os.makedirs(FIG, exist_ok=True)

import json as _json
import numpy as _np

NB = "../Exp4'6-one_d_hopper.ipynb"
nb = _json.load(open(NB))
src = ""
# imports, parameters, spring, block diagram, simulate, energy, controller
for i in [2, 4, 8, 10, 16, 28]:
    src += "".join(nb["cells"][i]["source"]) + "\n\n"
ns = {"np": _np, "plt": plt, "matplotlib": matplotlib}
exec(compile(src, "hopper_cells", "exec"), ns)
get_hopper_parameters = ns["get_hopper_parameters"]
build_block_diagram = ns["build_block_diagram"]
PreloadController = ns["PreloadController"]
mechanical_energy = ns["mechanical_energy"]


def simulate_headless(initial_state, duration, h):
    diagram = build_block_diagram(PreloadController(h))
    simulator = Simulator(diagram)
    hopper = diagram.GetSubsystemByName("hopper")
    sim_context = simulator.get_mutable_context()
    hopper_context = diagram.GetMutableSubsystemContext(hopper, sim_context)
    hopper.SetPositionsAndVelocities(hopper_context, initial_state)
    simulator.Initialize()
    simulator.AdvanceTo(duration)
    log = diagram.GetSubsystemByName("logger").FindLog(simulator.get_context())
    return log.sample_times(), log.data()


def metrics(t, data, h):
    zb = data[0]
    delta = data[4]
    tail = zb[t >= t[-1] - 2.0]
    return {
        "final_two_second_max_height": float(np.max(tail)),
        "final_two_second_min_height": float(np.min(tail)),
        "final_two_second_mean_height": float(np.mean(tail)),
        "height_error": float(np.max(tail) - h),
        "max_preload": float(np.max(delta)),
        "max_body_height": float(np.max(zb)),
    }


def main():
    duration = 8.0

    # Target-height sensitivity.
    h_values = [0.5, 0.75, 1.0, 1.25, 1.5]
    init = [0.5, 0, 0, 0]
    h_rows = []
    fig, ax = plt.subplots(figsize=(8, 4.5))
    for h in h_values:
        t, data = simulate_headless(init, duration, h)
        m = metrics(t, data, h)
        h_rows.append({"h": h, "initial_state": init, **m})
        ax.plot(t, data[0], label=f"h={h}")
    ax.plot([0, duration], [h_values[-1]] * 2, "k--", lw=0.6)
    ax.set_xlabel("time [s]")
    ax.set_ylabel(r"$z_b$ [m]")
    ax.set_title("Hopper: body height for different targets")
    ax.grid(True, ls=":", lw=0.4)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig_hopper_height.png"), dpi=200)
    plt.close(fig)

    # Initial-condition sensitivity at h=1.
    z0_values = [0.1, 0.3, 0.5, 0.8, 1.0]
    z0_rows = []
    fig, ax = plt.subplots(figsize=(8, 4.5))
    for z0 in z0_values:
        x0 = [z0, 0, 0, 0]
        t, data = simulate_headless(x0, duration, 1.0)
        m = metrics(t, data, 1.0)
        z0_rows.append({"z0": z0, "initial_state": x0, **m})
        ax.plot(t, data[0], label=f"z0={z0}")
    ax.set_xlabel("time [s]")
    ax.set_ylabel(r"$z_b$ [m]")
    ax.set_title(r"Hopper: convergence from different initial heights ($h=1$)")
    ax.grid(True, ls=":", lw=0.4)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig_hopper_initial.png"), dpi=200)
    plt.close(fig)

    # Energy and preload for the nominal case.
    t, data = simulate_headless([0.1, 0, 0, 0], duration, 1.0)
    E = mechanical_energy(data[:4], data[4])
    fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    axes[0].plot(t, E)
    axes[0].set_ylabel("mechanical energy [J]")
    axes[0].set_title("Hopper: energy and preload (h=1)")
    axes[1].plot(t, data[4])
    axes[1].set_ylabel(r"preload $\delta$ [m]")
    axes[1].set_xlabel("time [s]")
    for ax in axes:
        ax.grid(True, ls=":", lw=0.4)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig_hopper_energy.png"), dpi=200)
    plt.close(fig)

    results = {
        "target_height_sweep": h_rows,
        "initial_height_sweep": z0_rows,
        "nominal_metrics": metrics(t, data, 1.0),
    }
    with open("results_hopper.json", "w") as fh:
        json.dump(results, fh, indent=2)

    print("target height sweep:")
    for r in h_rows:
        print(
            f"  h={r['h']:.2f}  max z_b (last 2 s)={r['final_two_second_max_height']:.4f}  "
            f"error={r['height_error']:+.4f}  max delta={r['max_preload']:.4f}"
        )
    print("initial height sweep:")
    for r in z0_rows:
        print(
            f"  z0={r['z0']:.2f}  max z_b (last 2 s)={r['final_two_second_max_height']:.4f}  "
            f"error={r['height_error']:+.4f}"
        )


if __name__ == "__main__":
    main()
