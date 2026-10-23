"""Analysis of Example 5.2: LittleDog gait optimization.

Loads the solution saved by the executed notebook and produces:
  fig5_8_littledog_gait.png        stance schedule and foot clearance
  fig5_9_littledog_state.png       CoM, contact forces, and foot motion
  fig5_10_littledog_snapshots.png  side-view stick figures over one half stride
  results_littledog.json           numerical summary
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
from matplotlib.patches import Rectangle

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "figures")
DATA = os.path.join(HERE, "..", "Ch.5-examples", "littledog_walking_trot.npz")
os.makedirs(OUT, exist_ok=True)

FEET = ["FL", "FR", "BL", "BR"]
FOOT_FRAMES = {
    "FL": "front_left_foot_center",
    "FR": "front_right_foot_center",
    "BL": "back_left_foot_center",
    "BR": "back_right_foot_center",
}
LEG_FRAMES = {
    "FL": ["front_left_hip", "front_left_upper_leg", "front_left_lower_leg",
           "front_left_foot_center"],
    "FR": ["front_right_hip", "front_right_upper_leg", "front_right_lower_leg",
           "front_right_foot_center"],
    "BL": ["back_left_hip", "back_left_upper_leg", "back_left_lower_leg",
           "back_left_foot_center"],
    "BR": ["back_right_hip", "back_right_upper_leg", "back_right_lower_leg",
           "back_right_foot_center"],
}


def quaternion_to_rpy(qw, qx, qy, qz):
    roll = np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx**2 + qy**2))
    pitch = np.arcsin(np.clip(2 * (qw * qy - qz * qx), -1.0, 1.0))
    yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))
    return roll, pitch, yaw


def main():
    d = np.load(DATA, allow_pickle=True)
    h = d["h_sol"]
    t = np.concatenate(([0.0], np.cumsum(h)))
    in_stance = d["in_stance"]
    com = d["com_sol"]
    comdot = d["comdot_sol"]
    forces = d["contact_force_sol"]  # (4, 3, N-1)
    q = d["q_sol"]
    _, pitch, _ = quaternion_to_rpy(q[0], q[1], q[2], q[3])

    half_stride = t[-1]
    full_stride = 2 * half_stride
    effective_speed = d["stride_length"] / full_stride

    results = {
        "gait": str(d["gait"]),
        "success": bool(d["success"]),
        "cost": float(d["cost"]),
        "solve_time_s": float(d["solve_time_s"]),
        "half_stride_duration_s": float(half_stride),
        "full_stride_duration_s": float(full_stride),
        "stride_length_m": float(d["stride_length"]),
        "effective_speed_m_s": float(effective_speed),
        "total_mass_kg": float(d["total_mass"]),
        "com_height_min_m": float(np.min(com[2])),
        "com_height_max_m": float(np.max(com[2])),
        "max_body_pitch_rad": float(np.max(np.abs(pitch))),
        "max_foot_clearance_m": float(np.max([
            np.max(d["frame_" + FOOT_FRAMES[f]][:, 2]) for f in FEET
        ])),
        "max_normal_force_per_foot_N": {
            f: float(np.max(forces[i, 2, :])) for i, f in enumerate(FEET)
        },
    }

    # ---- gait diagram and foot clearance ------------------------------
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True,
                             gridspec_kw={"height_ratios": [1.2, 1]})
    ax = axes[0]
    for i, foot in enumerate(FEET):
        for n in range(len(t) - 1):
            if in_stance[i, n]:
                ax.barh(len(FEET) - 1 - i, t[n + 1] - t[n], left=t[n],
                        height=0.6, color="tab:blue", alpha=0.75)
    ax.set_yticks(range(len(FEET)))
    ax.set_yticklabels(FEET[::-1])
    ax.set_ylabel("foot")
    ax.set_title("Walking-trot stance schedule (blue = stance)")
    ax.grid(True, axis="x", alpha=0.3)

    ax = axes[1]
    for foot in FEET:
        z = d["frame_" + FOOT_FRAMES[foot]][:, 2]
        # subtract the standing height of the foot frame so stance is at zero
        ax.plot(t, 1e3 * (z - np.min(z)), "o-", ms=3, label=foot)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("foot height [mm]")
    ax.grid(True, alpha=0.3)
    ax.legend(ncol=4, fontsize=8)
    ax.set_title("Swing-foot clearance")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig5_8_littledog_gait.png"), dpi=200)
    plt.close(fig)

    # ---- CoM, velocity, contact forces --------------------------------
    fig, axes = plt.subplots(2, 2, figsize=(12, 7))
    ax = axes[0, 0]
    ax.plot(t, com[0], "o-", ms=3, label="CoM x")
    ax.plot(t, com[2], "s-", ms=3, label="CoM z")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("position [m]")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    ax.set_title("Center of mass")

    ax = axes[0, 1]
    ax.plot(t, comdot[0], "o-", ms=3, label=r"$\dot x$")
    ax.plot(t, comdot[2], "s-", ms=3, label=r"$\dot z$")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("velocity [m/s]")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    ax.set_title("CoM velocity")

    ax = axes[1, 0]
    for i, foot in enumerate(FEET):
        ax.plot(t[:-1], forces[i, 2, :], "o-", ms=3, label=foot)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("normal force [N]")
    ax.grid(True, alpha=0.3)
    ax.legend(ncol=2, fontsize=8)
    ax.set_title("Contact normal forces")

    ax = axes[1, 1]
    for i, (foot, frame) in enumerate(FOOT_FRAMES.items()):
        pos = d["frame_" + frame]
        ax.plot(t, pos[:, 0], "o-", ms=3, label=f"{foot} x")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("foot x [m]")
    ax.grid(True, alpha=0.3)
    ax.legend(ncol=2, fontsize=8)
    ax.set_title("Foot progression")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig5_9_littledog_state.png"), dpi=200)
    plt.close(fig)

    # ---- side-view stick figures --------------------------------------
    snapshots = [0, 3, 6, 9, 12, 15, 18, 20]
    fig, axes = plt.subplots(2, 4, figsize=(13, 5.6))
    for ax, n in zip(axes.ravel(), snapshots):
        body = d["frame_body"][n]
        # Body as a short segment along the pitch direction.
        L = 0.22
        ax.plot([body[0] - L / 2 * np.cos(pitch[n]),
                 body[0] + L / 2 * np.cos(pitch[n])],
                [body[2] + L / 2 * np.sin(pitch[n]),
                 body[2] - L / 2 * np.sin(pitch[n])],
                color="k", lw=3)
        for foot in FEET:
            frames = LEG_FRAMES[foot]
            pts = np.array([d["frame_" + f][n][[0, 2]] for f in frames])
            ax.plot(pts[:, 0], pts[:, 1], "-o", ms=2.5, lw=1.4,
                    color="tab:red" if foot.startswith("F") else "tab:blue",
                    ls="-" if foot.endswith("L") else "--",
                    alpha=0.9)
        ax.axhline(0, color="0.4", lw=1)
        ax.set_xlim(body[0] - 0.30, body[0] + 0.30)
        ax.set_ylim(-0.02, 0.30)
        ax.set_aspect("equal")
        ax.set_title(f"t = {t[n]:.3f} s", fontsize=11)
        ax.grid(True, alpha=0.2)
        ax.set_xlabel("x [m]", fontsize=10)
        if n == snapshots[0]:
            ax.set_ylabel("z [m]", fontsize=10)
    fig.suptitle("LittleDog walking trot: side-view snapshots over one half stride", fontsize=13)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig5_10_littledog_snapshots.png"), dpi=200)
    plt.close(fig)

    with open(os.path.join(HERE, "results_littledog.json"), "w") as f:
        json.dump(results, f, indent=2)
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
