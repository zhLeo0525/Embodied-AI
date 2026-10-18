"""
Exercise 3.3 -- Controllability of discrete and continuous LTI systems.

Part (a): discrete double integrator on an integer grid.
    x1[n+1] = x1[n] + x2[n]
    x2[n+1] = x2[n] + u[n],      u[n] in Z

Part (b): controllability of four continuous-time LTI pairs (Ai, Bi).

Running this script produces
    figures/ex3_3_grid_graph.png
    results_ex3_3.json
in the current directory.
"""

import json
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle

FIG_DIR = "figures"
os.makedirs(FIG_DIR, exist_ok=True)


# ---------------------------------------------------------------------------
# Part (a): the discrete double integrator as a graph on Z^2
# ---------------------------------------------------------------------------


def step(state, u):
    """One step of the discrete double integrator."""
    x1, x2 = state
    return (x1 + x2, x2 + u)


def simulate(x0, controls):
    """Apply a control sequence and return the full state trajectory."""
    states = [tuple(x0)]
    for u in controls:
        states.append(step(states[-1], u))
    return states


def two_step_control(start, goal):
    """Find (u0, u1) taking `start` to `goal` in exactly two steps.

    After two steps from (i1, j1):
        x1[2] = i1 + 2*j1 + u0
        x2[2] = j1 + u0 + u1
    Both equations can be solved in integers because the controls are
    unconstrained integers.
    """
    i1, j1 = start
    i2, j2 = goal
    u0 = i2 - i1 - 2 * j1
    u1 = j2 - j1 - u0
    return (u0, u1)


def draw_grid_graph():
    """Draw the edges of the graph for u = -1, 0, +1 (nine source nodes)."""
    nodes = [(i, j) for i in range(3) for j in range(3)]
    colors = {-1: "tab:blue", 0: "tab:gray", 1: "tab:orange"}
    labels = {-1: r"$u=-1$", 0: r"$u=0$", 1: r"$u=+1$"}

    fig, axes = plt.subplots(1, 3, figsize=(17, 6))

    for ax, u in zip(axes, [-1, 0, 1]):
        destinations = set()
        for s in nodes:
            d = step(s, u)
            destinations.add(d)
            if d == s:
                # Self-loop (only possible for rare states): draw a small loop
                # above the node so that it is visible.
                ax.annotate(
                    "",
                    xy=(s[0] + 0.12, s[1] + 0.10),
                    xytext=(s[0] - 0.12, s[1] + 0.10),
                    arrowprops=dict(
                        arrowstyle="-|>",
                        color=colors[u],
                        lw=1.5,
                        connectionstyle="arc3,rad=2.2",
                        mutation_scale=14,
                    ),
                )
            else:
                ax.annotate(
                    "",
                    xy=d,
                    xytext=s,
                    arrowprops=dict(
                        arrowstyle="-|>",
                        color=colors[u],
                        lw=1.5,
                        shrinkA=17,
                        shrinkB=15,
                        mutation_scale=14,
                    ),
                )

        for p in sorted(set(nodes) | destinations):
            inside = 0 <= p[0] <= 2 and 0 <= p[1] <= 2
            ax.add_patch(
                Circle(
                    p,
                    0.17,
                    facecolor="white" if inside else "none",
                    edgecolor="black" if inside else colors[u],
                    linestyle="-" if inside else "--",
                    lw=1.3,
                    zorder=3,
                )
            )
            ax.text(
                p[0],
                p[1],
                rf"$s_{{{p[0]},{p[1]}}}$",
                ha="center",
                va="center",
                fontsize=11,
                zorder=4,
            )

        ax.set_xlim(-0.8, 4.8)
        ax.set_ylim(-1.8, 3.8)
        ax.set_aspect("equal")
        ax.set_xlabel(r"$x_1$")
        ax.set_ylabel(r"$x_2$")
        ax.set_title(f"Edges for {labels[u]}")
        ax.grid(True, ls=":", lw=0.5, alpha=0.6)

    fig.suptitle(
        r"Discrete double integrator: $(x_1,x_2)\mapsto(x_1+x_2,\;x_2+u)$",
        fontsize=14,
    )
    fig.tight_layout()
    fig.savefig(os.path.join(FIG_DIR, "ex3_3_grid_graph.png"), dpi=200)
    plt.close(fig)


def discrete_experiments():
    """Verify the two-step construction for several start/target pairs."""
    tests = []

    # From s_{0,0} to s_{i,j} with j > i > 0 (part a.2).
    for i, j in [(1, 2), (2, 3), (2, 5), (3, 7)]:
        controls = two_step_control((0, 0), (i, j))
        traj = simulate((0, 0), controls)
        tests.append(
            {
                "start": [0, 0],
                "goal": [i, j],
                "controls": list(controls),
                "trajectory": traj,
                "reached_goal_in_2_steps": traj[-1] == (i, j),
            }
        )

    # Arbitrary start -> arbitrary goal (part a.3).
    for start, goal in [((2, -3), (-4, 5)), ((1, 2), (3, 0)), ((-1, 4), (2, -2))]:
        controls = two_step_control(start, goal)
        traj = simulate(start, controls)
        tests.append(
            {
                "start": list(start),
                "goal": list(goal),
                "controls": list(controls),
                "trajectory": traj,
                "reached_goal_in_2_steps": traj[-1] == goal,
            }
        )

    # One step from s_{0,0} can only reach (0, u), so s_{i,j}, i>0, is not
    # reachable in one step.  This is the lower bound that makes 2 minimal.
    assert all(t["reached_goal_in_2_steps"] for t in tests)
    return tests


# ---------------------------------------------------------------------------
# Part (b): continuous-time controllability
# ---------------------------------------------------------------------------


def controllability_matrix(A, B):
    n = A.shape[0]
    cols = [B]
    for _ in range(1, n):
        cols.append(A @ cols[-1])
    return np.hstack(cols)


def pbh_ranks(A, B):
    """rank([A - lambda I, B]) for each distinct eigenvalue."""
    result = []
    for lam in np.unique(np.linalg.eigvals(A)):
        M = np.hstack([A - lam * np.eye(A.shape[0]), B])
        result.append(
            {
                "eigenvalue": [float(lam.real), float(lam.imag)],
                "pbh_rank": int(np.linalg.matrix_rank(M)),
                "full_rank": bool(np.linalg.matrix_rank(M) == A.shape[0]),
            }
        )
    return result


def analyse_pair(name, A, B):
    C = controllability_matrix(A, B)
    rank = int(np.linalg.matrix_rank(C))
    n = A.shape[0]
    sv = np.linalg.svd(C, compute_uv=False)
    return {
        "name": name,
        "A": A.tolist(),
        "B": B.tolist(),
        "controllability_matrix": C.tolist(),
        "rank": rank,
        "n": n,
        "controllable": rank == n,
        "singular_values": sv.tolist(),
        "condition_number": float(sv[0] / sv[-1]) if sv[-1] > 1e-12 else None,
        "eigenvalues_A": [
            [float(z.real), float(z.imag)] for z in np.linalg.eigvals(A)
        ],
        "pbh": pbh_ranks(A, B),
    }


def continuous_experiments():
    A1 = np.array([[1.0, 0.0], [0.0, 1.0]])
    B1 = np.array([[0.0], [1.0]])

    A2 = np.array([[1.0, 0.0], [0.0, 1.0]])
    B2 = np.array([[1.0], [1.0]])

    A3 = np.array([[0.0, 1.0], [0.0, 0.0]])
    B3 = np.array([[0.0], [1.0]])

    A4 = np.array(
        [
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 2.0, 0.0, 0.0],
        ]
    )
    B4 = np.array([[0.0], [0.0], [1.0], [1.0]])

    pairs = [
        analyse_pair("(A1, B1)", A1, B1),
        analyse_pair("(A2, B2)", A2, B2),
        analyse_pair("(A3, B3)", A3, B3),
        analyse_pair("(A4, B4)", A4, B4),
    ]

    # Underactuation requires a second-order interpretation x = [q, qdot].
    # (A3, B3): q is 1-dimensional and B3 enters qddot -> fully actuated.
    # (A4, B4): q = [x, theta] is 2-dimensional, and B4 only enters xddot
    #           -> one actuator for two degrees of freedom -> underactuated.
    underactuation = {
        "(A3, B3)": {
            "n_dof": 1,
            "n_actuators": 1,
            "underactuated": False,
            "reason": "q is scalar; B3 drives qddot directly.",
        },
        "(A4, B4)": {
            "n_dof": 2,
            "n_actuators": 1,
            "underactuated": True,
            "reason": "q = [x, theta]; only x is actuated, theta is not.",
        },
        "remark": (
            "(A1, B1) and (A2, B2) are not naturally written as second-order "
            "systems with A = [[0, I], [*, *]], so the underactuation "
            "definition does not apply to them."
        ),
    }

    return pairs, underactuation


def main():
    draw_grid_graph()
    discrete_tests = discrete_experiments()
    pairs, underactuation = continuous_experiments()

    results = {
        "part_a": {
            "minimum_steps_from_s00": 2,
            "minimum_steps_arbitrary_start_goal": 2,
            "two_step_tests": discrete_tests,
        },
        "part_b": {
            "pairs": pairs,
            "underactuation": underactuation,
        },
    }

    with open("results_ex3_3.json", "w") as f:
        json.dump(results, f, indent=2)

    print("=== Part (a): discrete double integrator ===")
    for t in discrete_tests:
        print(
            f"  {t['start']} --(u={t['controls']})--> {t['trajectory']}"
            f"  goal {t['goal']}: {t['reached_goal_in_2_steps']}"
        )

    print("\n=== Part (b): continuous-time pairs ===")
    for p in pairs:
        print(f"\n{p['name']}")
        print("  A =\n", np.array(p["A"]))
        print("  B =", np.array(p["B"]).ravel())
        print("  C = [B, AB, ...] =\n", np.array(p["controllability_matrix"]))
        print(f"  rank(C) = {p['rank']} / {p['n']}  -> controllable: {p['controllable']}")
        print(f"  singular values of C: {np.round(p['singular_values'], 4)}")
        print(f"  eigenvalues(A): {np.round(np.array(p['eigenvalues_A']), 4).tolist()}")
        print(
            "  PBH ranks:",
            [(np.round(q["eigenvalue"], 4).tolist(), q["pbh_rank"]) for q in p["pbh"]],
        )

    print("\n=== Underactuation ===")
    for key in ["(A3, B3)", "(A4, B4)"]:
        print(f"  {key}: {underactuation[key]}")


if __name__ == "__main__":
    main()
