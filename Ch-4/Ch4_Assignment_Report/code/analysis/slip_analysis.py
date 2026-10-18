"""Examples 4.5/4.6 -- SLIP apex-to-apex map and sensitivity.

The plant class is taken verbatim from the supplied notebook
(Exp4'5-6-slip.ipynb) by executing its definition cells.  The map is computed
for several touchdown angles and dimensionless energies.
"""

import json
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from pydrake.all import (
    BasicVector,
    LeafSystem,
    PublishEvent,
    Simulator,
    UnrestrictedUpdateEvent,
    WitnessFunctionDirection,
    namedview,
)

FIG = "figures"
os.makedirs(FIG, exist_ok=True)

import json as _json

NB = "../notebooks/Exp4'5-6-slip.ipynb"
nb = _json.load(open(NB))
plant_src = "".join(nb["cells"][5]["source"])
# Keep only the state view and the plant class (the visualizer is not needed).
plant_src = plant_src.split("class SLIPVisualizer")[0]
ns = {
    "np": np,
    "BasicVector": BasicVector,
    "LeafSystem": LeafSystem,
    "namedview": namedview,
    "WitnessFunctionDirection": WitnessFunctionDirection,
    "UnrestrictedUpdateEvent": UnrestrictedUpdateEvent,
    "PublishEvent": PublishEvent,
}
exec(plant_src, ns)
SLIPState = ns["SLIPState"]
SpringLoadedInvertedPendulum = ns["SpringLoadedInvertedPendulum"]


def apex_map(touchdown_angle, Etilde, n=30, zmax=1.2):
    plant = SpringLoadedInvertedPendulum()
    s = SLIPState(np.zeros(8))
    s.theta = touchdown_angle
    s.r = 1.0

    simulator = Simulator(plant)
    context = simulator.get_mutable_context()
    plant.get_input_port(0).FixValue(context, [touchdown_angle])
    context.SetAccuracy(1e-5)

    zmin = np.cos(touchdown_angle) + 0.001
    zs = np.linspace(zmin, zmax, n)
    zns = np.full(n, np.nan)
    for i, z in enumerate(zs):
        s.z = z
        try:
            s.xdot = plant.apex_velocity_from_dimensionless_system_energy(
                Etilde, z
            )
        except Exception:
            continue
        context.SetTime(0.0)
        context.get_mutable_continuous_state_vector().SetFromVector(s[:])
        simulator.Initialize()
        simulator.AdvanceTo(0.6)
        if plant.last_apex is not None:
            zns[i] = plant.last_apex
        plant.last_apex = None
    return zs, zns


def fixed_points(zs, zns):
    mask = np.isfinite(zns)
    z = zs[mask]
    p = zns[mask]
    g = p - z
    roots = []
    for i in range(len(z) - 1):
        if g[i] == 0:
            roots.append((z[i], np.nan))
        elif g[i] * g[i + 1] < 0:
            # linear interpolation
            zr = z[i] - g[i] * (z[i + 1] - z[i]) / (g[i + 1] - g[i])
            # derivative by finite difference on the original map
            j = min(max(i, 1), len(z) - 2)
            dp = (p[j + 1] - p[j - 1]) / (z[j + 1] - z[j - 1])
            roots.append((float(zr), float(dp)))
    return roots


def main():
    # (a) fix the touchdown angle at 30 deg and vary the energy
    # (b) fix the energy at 1.61 and vary the touchdown angle
    angle_nominal = np.deg2rad(30)
    energy_nominal = 1.61
    energies = [1.55, 1.60, 1.65]
    angles = [np.deg2rad(28), np.deg2rad(30), np.deg2rad(32)]

    results = []
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8))

    for E in energies:
        zs, zns = apex_map(angle_nominal, E)
        roots = fixed_points(zs, zns)
        axes[0].plot(zs, zns, ".-", ms=3, label=fr"$\tilde E={E}$")
        for zr, dp in roots:
            stable = np.isfinite(dp) and abs(dp) < 1.0
            axes[0].plot(
                zr, zr,
                "o" if stable else "o",
                ms=5,
                mfc="tab:green" if stable else "white",
                mec="k",
            )
        results.append(
            {
                "touchdown_angle_deg": 30.0,
                "Etilde": E,
                "zs": zs.tolist(),
                "zns": zns.tolist(),
                "fixed_points": [
                    {"z": zr, "P_prime": dp} for zr, dp in roots
                ],
            }
        )

    for ang in angles:
        zs, zns = apex_map(ang, energy_nominal)
        roots = fixed_points(zs, zns)
        axes[1].plot(
            zs, zns, ".-", ms=3,
            label=fr"$\theta_{{td}}={np.rad2deg(ang):.0f}^\circ$",
        )
        for zr, dp in roots:
            stable = np.isfinite(dp) and abs(dp) < 1.0
            axes[1].plot(
                zr, zr, "o", ms=5,
                mfc="tab:green" if stable else "white", mec="k",
            )
        results.append(
            {
                "touchdown_angle_deg": float(np.rad2deg(ang)),
                "Etilde": energy_nominal,
                "zs": zs.tolist(),
                "zns": zns.tolist(),
                "fixed_points": [
                    {"z": zr, "P_prime": dp} for zr, dp in roots
                ],
            }
        )

    for ax, title in zip(
        axes,
        [
            fr"Fixed $\theta_{{td}}=30^\circ$: energy sensitivity",
            r"Fixed $\tilde E=1.61$: touchdown-angle sensitivity",
        ],
    ):
        ax.plot([0.8, 1.1], [0.8, 1.1], "k--", lw=0.8)
        ax.set_xlim(0.84, 1.05)
        ax.set_ylim(0.84, 1.05)
        ax.set_xlabel(r"apex height $z[n]$ [m]")
        ax.set_ylabel(r"apex height $z[n+1]$ [m]")
        ax.set_title(title)
        ax.grid(True, ls=":", lw=0.4)
        ax.legend(fontsize=8)
    fig.suptitle("SLIP: apex-to-apex return map (filled = stable fixed point)")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig_slip_map.png"), dpi=200)
    plt.close(fig)

    summary = []
    for r in results:
        for fp in r["fixed_points"]:
            summary.append(
                {
                    "touchdown_angle_deg": r["touchdown_angle_deg"],
                    "Etilde": r["Etilde"],
                    "z": fp["z"],
                    "P_prime": fp["P_prime"],
                    "stable": bool(np.isfinite(fp["P_prime"]) and abs(fp["P_prime"]) < 1.0),
                }
            )

    with open("results_slip.json", "w") as fh:
        _json.dump({"maps": results, "summary": summary}, fh, indent=2)

    print("touchdown  Etilde  z*       P'(z*)  stable")
    for r in summary:
        print(
            f"{r['touchdown_angle_deg']:8.0f}  {r['Etilde']:.2f}   "
            f"{r['z']:.4f}  {r['P_prime']:6.3f}  {r['stable']}"
        )


if __name__ == "__main__":
    main()
