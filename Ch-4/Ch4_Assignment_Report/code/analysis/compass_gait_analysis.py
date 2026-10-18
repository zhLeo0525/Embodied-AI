"""Example 4.4 -- passive compass gait: limit cycle and sensitivity.

State: [stance, swing, stance_dot, swing_dot].
The plant walks downhill; the slope is a numeric parameter of CompassGait.
Steps are detected by a change of the abstract "left support" state.
"""

import json
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from pydrake.all import DiagramBuilder, LogVectorOutput, Simulator
from pydrake.examples import CompassGait

FIG = "figures"
os.makedirs(FIG, exist_ok=True)


def simulate(x0, slope=0.0525, duration=10.0, dt=0.002):
    plant = CompassGait()
    builder = DiagramBuilder()
    plant = builder.AddSystem(plant)
    logger = LogVectorOutput(plant.get_output_port(0), builder)
    diagram = builder.Build()

    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()
    context.SetAccuracy(1e-4)
    pctx = diagram.GetMutableSubsystemContext(plant, context)
    pctx.get_mutable_numeric_parameter(0).set_slope(slope)
    context.SetContinuousState(np.asarray(x0, dtype=float))

    simulator.Initialize()
    pctx = diagram.GetMutableSubsystemContext(plant, simulator.get_mutable_context())
    last_support = bool(pctx.get_abstract_state(0).get_value())

    events = []
    t = 0.0
    while t < duration:
        t += dt
        simulator.AdvanceTo(t)
        pctx = diagram.GetMutableSubsystemContext(
            plant, simulator.get_mutable_context()
        )
        support = bool(pctx.get_abstract_state(0).get_value())
        if support != last_support:
            state = pctx.get_continuous_state_vector().CopyToVector().copy()
            events.append((t, state))
            last_support = support

    log = logger.FindLog(simulator.get_context())
    return log.sample_times(), log.data(), events


def is_sustained(t, data, events, duration):
    """A simple limit-cycle detector: enough steps, bounded state, periodic."""
    if len(events) < 10:
        return False
    et = np.array([e[0] for e in events])
    periods = np.diff(et)
    if len(periods) < 5:
        return False
    last = periods[-5:]
    if np.std(last) / max(np.mean(last), 1e-9) > 0.05:
        return False
    tail = data[:, t >= duration - 3.0]
    if np.max(np.abs(tail)) > 3.0:
        return False
    return True


def main():
    x0_default = np.array([0.0, 0.0, 0.4, -2.0])
    t, data, events = simulate(x0_default)

    # Phase portrait and step events.
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    axes[0].plot(data[0], data[2], "b", lw=1.0)
    ev = np.array([e[1] for e in events])
    if len(ev):
        axes[0].plot(ev[:, 0], ev[:, 2], "ro", ms=3, label="step events")
    axes[0].set_xlabel("stance angle [rad]")
    axes[0].set_ylabel("stance angular velocity [rad/s]")
    axes[0].set_title("Compass gait: phase portrait")
    axes[0].grid(True, ls=":", lw=0.4)
    axes[0].legend()

    if len(ev) > 2:
        axes[1].plot(ev[:-1, 0], ev[1:, 0], "o-", ms=3)
        lim = [min(ev[:, 0]), max(ev[:, 0])]
        axes[1].plot(lim, lim, "k--", lw=0.8)
    axes[1].set_xlabel(r"stance angle at step $n$ [rad]")
    axes[1].set_ylabel(r"stance angle at step $n+1$ [rad]")
    axes[1].set_title("Step-to-step map (stance angle)")
    axes[1].grid(True, ls=":", lw=0.4)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig_cg_phase.png"), dpi=200)
    plt.close(fig)

    # Initial-condition sensitivity.
    initials = [
        [0.0, 0.0, 0.4, -2.0],
        [0.0, 0.0, 0.2, -1.5],
        [0.0, 0.0, 0.6, -2.5],
        [0.0, 0.0, 0.4, -1.0],
    ]
    fig, ax = plt.subplots(figsize=(7, 5))
    init_rows = []
    for x0_init in initials:
        ti, di, ei = simulate(np.array(x0_init))
        ax.plot(di[0], di[2], lw=1.0, label=str(x0_init))
        init_rows.append(
            {
                "x0": x0_init,
                "n_steps": len(ei),
                "final_state": di[:, -1].tolist(),
                "mean_step_period": float(np.mean(np.diff([e[0] for e in ei])))
                if len(ei) > 1
                else None,
            }
        )
    ax.set_xlabel("stance angle [rad]")
    ax.set_ylabel("stance angular velocity [rad/s]")
    ax.set_title("Compass gait: convergence from different initial states")
    ax.grid(True, ls=":", lw=0.4)
    ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig_cg_initial.png"), dpi=200)
    plt.close(fig)

    # Slope sensitivity: try several initial conditions for each slope.
    slopes = [0.02, 0.035, 0.0525, 0.08, 0.12]
    init_stance_dot = [0.2, 0.4, 0.6]
    init_swing_dot = [-2.5, -2.0, -1.5]
    slope_rows = []
    for gamma in slopes:
        runs = []
        for sd in init_stance_dot:
            for swd in init_swing_dot:
                x0_try = np.array([0.0, 0.0, sd, swd])
                try:
                    ts, ds, es = simulate(x0_try, slope=gamma, duration=12.0)
                    ok = is_sustained(ts, ds, es, 12.0)
                    periods = np.diff([e[0] for e in es])
                    runs.append(
                        {
                            "x0": x0_try.tolist(),
                            "n_steps": len(es),
                            "success": bool(ok),
                            "mean_step_period": float(np.mean(periods))
                            if len(periods)
                            else None,
                            "final_state": ds[:, -1].tolist(),
                        }
                    )
                except Exception as exc:  # noqa: BLE001
                    runs.append(
                        {
                            "x0": x0_try.tolist(),
                            "n_steps": 0,
                            "success": False,
                            "mean_step_period": None,
                            "final_state": None,
                            "error": str(exc),
                        }
                    )
        good = [r for r in runs if r["success"]]
        slope_rows.append(
            {
                "slope": gamma,
                "success_fraction": len(good) / len(runs),
                "n_success": len(good),
                "n_trials": len(runs),
                "mean_step_period": float(
                    np.mean([r["mean_step_period"] for r in good])
                )
                if good
                else None,
                "runs": runs,
            }
        )

    fig, ax = plt.subplots(figsize=(7, 4.5))
    gs = [r["slope"] for r in slope_rows]
    frac = [r["success_fraction"] for r in slope_rows]
    ax.plot(gs, frac, "o-")
    ax.set_xlabel(r"slope $\gamma$ [rad]")
    ax.set_ylabel("fraction of initial states with a sustained gait")
    ax.set_ylim(-0.02, 1.02)
    ax.set_title("Compass gait: basin of attraction vs slope")
    ax.grid(True, ls=":", lw=0.4)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig_cg_slope.png"), dpi=200)
    plt.close(fig)

    results = {
        "default_initial_state": x0_default.tolist(),
        "default_n_steps": len(events),
        "default_event_times": [e[0] for e in events],
        "default_event_states": [e[1].tolist() for e in events],
        "initial_conditions": init_rows,
        "slopes": slope_rows,
    }
    with open("results_cg.json", "w") as fh:
        json.dump(results, fh, indent=2)

    print("default steps:", len(events))
    print("slope sensitivity:")
    for r in slope_rows:
        print(
            f"  gamma={r['slope']:.4f}  success {r['n_success']}/{r['n_trials']}  "
            f"period={r['mean_step_period']}"
        )


if __name__ == "__main__":
    main()
