"""Analysis of Section 5.8: footstep planning via MIQP and via GCS.

Produces:
  fig5_4_miqp_plans.png       optimal MIQP plans on terrains A and B
  fig5_5_gcs_plans.png        optimal GCS plans on the longer terrains
  fig5_6_span_sensitivity.png feasible step span vs. cost / number of steps
  fig5_7_runtime_scaling.png  MIQP vs GCS solve time
  results_footstep.json       numerical summary
"""

import json
import logging
import os
import time

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
from pydrake.all import GraphOfConvexSets, Point, HPolyhedron
from pydrake.all import MathematicalProgram, OsqpSolver, le, ge, eq
from pydrake.solvers import MixedIntegerBranchAndBound, SnoptSolver

logging.getLogger("drake").setLevel(logging.WARNING)

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "figures")
os.makedirs(OUT, exist_ok=True)


# ---------------------------------------------------------------------------
# Terrain
# ---------------------------------------------------------------------------
class SteppingStone:
    def __init__(self, center, width, height, name=None):
        self.center = np.array(center, dtype=float)
        self.width = width
        self.height = height
        self.name = name
        c2tr = np.array([width, height]) / 2
        c2br = np.array([width, -height]) / 2
        self.top_right = self.center + c2tr
        self.bottom_right = self.center + c2br
        self.top_left = self.center - c2br
        self.bottom_left = self.center - c2tr
        self.A = np.array([[1, 0], [0, 1], [-1, 0], [0, -1]], dtype=float)
        self.b = np.concatenate([c2tr] * 2) + self.A.dot(self.center)


class Terrain:
    def __init__(self, bool_bridge):
        self.stepping_stones = []
        initial = self.add_stone([0, 0], 1, 1, "initial")
        width_bridge = 0.2
        center = initial.bottom_right + np.array(
            [width_bridge * 1.5, initial.height / 4]
        )
        centers = [
            center + np.array([i * 2 * width_bridge, 0])
            for i in np.where(bool_bridge)[0]
        ]
        self.add_stones(
            centers,
            [width_bridge] * int(sum(bool_bridge)),
            [initial.height / 2] * int(sum(bool_bridge)),
            "bridge",
        )
        center = initial.center + np.array(
            [initial.width + (len(bool_bridge) * 2 + 1) * width_bridge, 0]
        )
        self.add_stone(center, initial.width, initial.height, "goal")
        height = 0.4
        clearance = 0.1
        c2g = self.stepping_stones[-1].center - initial.center
        width = initial.width + c2g[0]
        center = (
            initial.center
            + c2g / 2
            + np.array([0, (initial.height + height) / 2 + clearance])
        )
        self.add_stone(center, width, height, "lateral")

    def add_stone(self, center, width, height, name=None):
        stone = SteppingStone(center, width, height, name=name)
        self.stepping_stones.append(stone)
        return stone

    def add_stones(self, centers, widths, heights, name=None):
        stones = []
        for i in range(len(centers)):
            stone_name = name if name is None else name + "_" + str(i)
            stones.append(self.add_stone(centers[i], widths[i], heights[i],
                                         name=stone_name))
        return stones

    def get_stone_by_name(self, name):
        for stone in self.stepping_stones:
            if stone.name == name:
                return stone
        raise ValueError(name)


def plot_terrain(ax, terrain, title=None):
    for i, stone in enumerate(terrain.stepping_stones):
        ax.add_patch(Rectangle(stone.bottom_left, stone.width, stone.height,
                               facecolor="0.85", edgecolor="k", lw=1.0))
        ax.text(stone.center[0], stone.center[1], stone.name, fontsize=6,
                ha="center", va="center")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.2)
    ax.set_title(title)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")


# ---------------------------------------------------------------------------
# MIQP planner (Section 5.8a)
# ---------------------------------------------------------------------------
def add_decision_variables(prog, terrain, n_steps):
    n_stones = len(terrain.stepping_stones)
    position_left = prog.NewContinuousVariables(rows=n_steps + 1, cols=2)
    position_right = prog.NewContinuousVariables(rows=n_steps + 1, cols=2)
    stone_left = prog.NewBinaryVariables(rows=n_steps + 1, cols=n_stones)
    stone_right = prog.NewBinaryVariables(rows=n_steps + 1, cols=n_stones)
    first_left = prog.NewBinaryVariables(1)[0]
    return position_left, position_right, stone_left, stone_right, first_left


def set_initial_and_goal_position(prog, terrain, dv):
    position_left, position_right = dv[:2]
    foot_offset = np.array([0, 0.2])
    center = terrain.get_stone_by_name("initial").center
    prog.AddLinearConstraint(eq(position_left[0], center))
    prog.AddLinearConstraint(eq(position_right[0], center - foot_offset))
    center = terrain.get_stone_by_name("goal").center
    prog.AddLinearConstraint(eq(position_left[-1], center))
    prog.AddLinearConstraint(eq(position_right[-1], center - foot_offset))


def relative_position_limits(prog, n_steps, step_span, dv):
    position_left, position_right = dv[:2]
    half = step_span / 2.0
    for t in range(n_steps + 1):
        rel = position_left[t] - position_right[t]
        prog.AddLinearConstraint(le(rel, half))
        prog.AddLinearConstraint(ge(rel, -half))


def step_sequence(prog, n_steps, step_span, dv):
    position_left, position_right = dv[:2]
    first_left = dv[-1]
    first_right = 1 - first_left
    limit = np.ones(2) * step_span
    for t in range(n_steps):
        step_left = position_left[t + 1] - position_left[t]
        step_right = position_right[t + 1] - position_right[t]
        if t % 2 == 0:
            lim_left, lim_right = limit * first_left, limit * first_right
        else:
            lim_left, lim_right = limit * first_right, limit * first_left
        prog.AddLinearConstraint(le(step_left, lim_left))
        prog.AddLinearConstraint(ge(step_left, -lim_left))
        prog.AddLinearConstraint(le(step_right, lim_right))
        prog.AddLinearConstraint(ge(step_right, -lim_right))


def one_stone_per_foot(prog, n_steps, dv):
    stone_left, stone_right = dv[2:4]
    for t in range(n_steps + 1):
        prog.AddLinearConstraint(stone_left[t].sum() == 1)
        prog.AddLinearConstraint(stone_right[t].sum() == 1)


def get_big_M(terrain):
    initial = terrain.get_stone_by_name("initial")
    goal = terrain.get_stone_by_name("goal")
    lateral = terrain.get_stone_by_name("lateral")
    M = [goal.center[0] - initial.center[0],
         lateral.top_right[1] - initial.center[1]]
    return np.array(M * 2)


def foot_in_stepping_stone(prog, terrain, n_steps, dv):
    position_left, position_right, stone_left, stone_right = dv[:4]
    M = get_big_M(terrain)
    for t in range(n_steps + 1):
        for i, stone in enumerate(terrain.stepping_stones):
            prog.AddLinearConstraint(
                le(stone.A.dot(position_left[t]),
                   stone.b + (1 - stone_left[t, i]) * M))
            prog.AddLinearConstraint(
                le(stone.A.dot(position_right[t]),
                   stone.b + (1 - stone_right[t, i]) * M))


def minimize_step_length(prog, n_steps, dv):
    position_left, position_right = dv[:2]
    for t in range(n_steps):
        dl = position_left[t + 1] - position_left[t]
        dr = position_right[t + 1] - position_right[t]
        prog.AddQuadraticCost(dl.dot(dl))
        prog.AddQuadraticCost(dr.dot(dr))


def footstep_planner(terrain, n_steps, step_span):
    prog = MathematicalProgram()
    dv = add_decision_variables(prog, terrain, n_steps)
    set_initial_and_goal_position(prog, terrain, dv)
    relative_position_limits(prog, n_steps, step_span, dv)
    step_sequence(prog, n_steps, step_span, dv)
    one_stone_per_foot(prog, n_steps, dv)
    foot_in_stepping_stone(prog, terrain, n_steps, dv)
    minimize_step_length(prog, n_steps, dv)
    t0 = time.perf_counter()
    bb = MixedIntegerBranchAndBound(prog, OsqpSolver().solver_id())
    result = bb.Solve()
    solve_time = time.perf_counter() - t0
    if result != result.kSolutionFound:
        raise ValueError("Infeasible optimization problem.")
    dv_opt = [bb.GetSolution(v) for v in dv]
    return dv_opt, bb.GetOptimalCost(), solve_time


def summarize_miqp(terrain, position_left, position_right):
    moves = []
    for t in range(len(position_left) - 1):
        dl = np.linalg.norm(position_left[t + 1] - position_left[t])
        dr = np.linalg.norm(position_right[t + 1] - position_right[t])
        if max(dl, dr) > 1e-6:
            moves.append(("L" if dl > dr else "R", t))
    used_lateral = False
    for pos in (position_left, position_right):
        for p in pos:
            stone = terrain.get_stone_by_name("lateral")
            if np.all(stone.A.dot(p) <= stone.b + 1e-6):
                used_lateral = True
    return len(moves), used_lateral, moves


# ---------------------------------------------------------------------------
# GCS planner (Section 5.8c)
# ---------------------------------------------------------------------------
def gcs_vertices_and_copy_edges(G, T, step_span):
    start_position = tuple(T.get_stone_by_name("initial").center)
    end_position = tuple(T.get_stone_by_name("goal").center)
    V, E_list = [], []
    V.append(G.AddVertex(Point(start_position), "source"))
    V.append(G.AddVertex(Point(end_position), "target"))

    def num_copies(stone):
        return int(np.floor(stone.width / step_span)) + 2

    n_lateral = num_copies(T.get_stone_by_name("lateral"))
    n_start_goal = num_copies(T.get_stone_by_name("initial"))
    bridges = [s for s in T.stepping_stones if s.name.startswith("bridge")]
    n_bridge = num_copies(bridges[0]) if bridges else 0

    for i, stone in enumerate(T.stepping_stones):
        H_i = HPolyhedron(stone.A, stone.b)
        if stone.name == "lateral":
            prefix, count = "l", n_lateral
        elif stone.name == "initial":
            prefix, count = "s", n_start_goal
        elif stone.name == "goal":
            prefix, count = "t", n_start_goal
        else:
            prefix, count = "b", n_bridge
        for ii in range(count):
            V.append(G.AddVertex(H_i, f"{prefix}{i}_{ii}"))
            if ii > 0:
                E_list.append([V[-2], V[-1]])
    return V, E_list


def gcs_add_edges(G, V, E_list):
    def split(name):
        if "_" not in name:
            return name, None
        stem, idx = name.rsplit("_", 1)
        return stem, int(idx)

    for u in V:
        if u.name() == "target":
            continue
        u_stem, _ = split(u.name())
        for v in V:
            if v.name() == "source":
                continue
            v_stem, v_idx = split(v.name())
            if u.name() == "source":
                if v.name() != "target" and v_idx == 0:
                    E_list.append([u, v])
            elif v.name() == "target":
                E_list.append([u, v])
            elif v_stem != u_stem and v_idx == 0:
                E_list.append([u, v])
    return E_list


def gcs_planner(T, step_span, convex_relaxation=True, solver=None):
    G = GraphOfConvexSets()
    V, E_list = gcs_vertices_and_copy_edges(G, T, step_span)
    E_list = gcs_add_edges(G, V, E_list)
    E = []
    for u, v in E_list:
        e = G.AddEdge(u, v)
        for k in range(2):
            e.AddConstraint(e.xu()[k] - e.xv()[k] <= step_span / 2.0)
            e.AddConstraint(e.xv()[k] - e.xu()[k] <= step_span / 2.0)
        e.AddCost(1.0)
        E.append(e)

    options = GraphOfConvexSetsOptions() if False else None
    from pydrake.geometry.optimization import GraphOfConvexSetsOptions
    options = GraphOfConvexSetsOptions()
    options.preprocessing = True
    options.max_rounded_paths = 10
    options.convex_relaxation = convex_relaxation
    options.solver = solver if solver is not None else SnoptSolver()
    t0 = time.perf_counter()
    result = G.SolveShortestPath(V[0], V[1], options)
    solve_time = time.perf_counter() - t0

    # Extract the path: follow the outgoing edge with flow 1.
    V_dict = {v.name(): i for i, v in enumerate(V)}
    V_adj = np.zeros((len(V), len(V)))
    for e in E:
        if result.GetSolution(e.phi()):
            V_adj[V_dict[e.u().name()], V_dict[e.v().name()]] = 1
    path = ["source"]
    while path[-1] != "target":
        u_idx = V_dict[path[-1]]
        nxt = np.where(V_adj[u_idx, :] == 1)[0]
        if len(nxt) == 0:
            raise RuntimeError("No outgoing edge in path.")
        path.append(V[nxt[0]].name())
    path_position = np.array([result.GetSolution(V[V_dict[p]].x())
                              for p in path])

    # Reconstruct left/right foot positions exactly as in the notebook.
    n_steps = path_position.shape[0]
    position_left = np.zeros((n_steps, 2))
    position_right = np.zeros((n_steps, 2))
    position_right[0, :] = np.array([0.0, -0.2])
    for i in range(n_steps):
        if i % 2 == 0:
            position_left[i, :] = path_position[i, :]
            if i > 0:
                position_right[i, :] = position_right[i - 1, :]
        else:
            position_right[i, :] = path_position[i, :]
            position_left[i, :] = position_left[i - 1, :]
    path_cost = len(path) - 1
    return position_left, position_right, path_cost, solve_time


# ---------------------------------------------------------------------------
# Plotting helpers
# ---------------------------------------------------------------------------
def plot_plan(ax, terrain, position_left, position_right, title):
    plot_terrain(ax, terrain, title)
    ax.plot(position_left[:, 0], position_left[:, 1], "o-", color="tab:red",
            ms=4, lw=1.2, label="left foot")
    ax.plot(position_right[:, 0], position_right[:, 1], "s-", color="tab:blue",
            ms=4, lw=1.2, label="right foot")
    ax.legend(fontsize=8, loc="upper left")


def main():
    results = {}

    # ---- MIQP on the two benchmark terrains ---------------------------
    n_steps, step_span = 8, 0.8
    terrain_A = Terrain([1, 1, 1, 1])
    terrain_B = Terrain([1, 1, 1, 0])
    miqp = {}
    plans = {}
    for name, terrain in [("A", terrain_A), ("B", terrain_B)]:
        dv, cost, dt = footstep_planner(terrain, n_steps, step_span)
        n_used, lateral, moves = summarize_miqp(terrain, dv[0], dv[1])
        miqp[name] = {
            "cost": float(cost),
            "solve_time_s": float(dt),
            "steps_used": n_used,
            "uses_lateral": bool(lateral),
            "first_foot": "left" if dv[4] > 0.5 else "right",
        }
        plans[name] = (dv[0], dv[1])
    results["miqp_benchmark"] = miqp

    fig, axes = plt.subplots(1, 2, figsize=(14, 5.2))
    for ax, name, terrain in zip(axes, ["A", "B"], [terrain_A, terrain_B]):
        plot_plan(ax, terrain, *plans[name],
                  f"MIQP terrain {name}: cost {miqp[name]['cost']:.3f}, "
                  f"{miqp[name]['steps_used']} steps")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig5_4_miqp_plans.png"), dpi=200)
    plt.close(fig)

    # ---- GCS on the notebook terrains ---------------------------------
    terrain_gcs_A = Terrain([1, 1, 1, 1, 1, 1])
    terrain_gcs_B = Terrain([1, 1, 1, 0, 1, 1])
    gcs = {}
    gcs_plans = {}
    for name, terrain in [("A", terrain_gcs_A), ("B", terrain_gcs_B)]:
        pl, pr, cost, dt = gcs_planner(terrain, step_span)
        gcs_plans[name] = (pl, pr)
        gcs[name] = {"path_cost": int(cost), "solve_time_s": float(dt),
                     "n_positions": int(pl.shape[0])}
    results["gcs_benchmark"] = gcs

    fig, axes = plt.subplots(1, 2, figsize=(14, 5.2))
    for ax, name, terrain in zip(axes, ["A", "B"],
                                  [terrain_gcs_A, terrain_gcs_B]):
        plot_plan(ax, terrain, *gcs_plans[name],
                  f"GCS terrain {name}: {gcs[name]['path_cost']} edges, "
                  f"{gcs[name]['solve_time_s']:.2f} s")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig5_5_gcs_plans.png"), dpi=200)
    plt.close(fig)

    # ---- step-span sensitivity (MIQP, terrain B) ----------------------
    span_rows = []
    for span in [0.60, 0.70, 0.75, 0.80, 0.90, 1.00, 1.10, 1.20, 1.30, 1.40, 1.50]:
        row = {"step_span": span}
        try:
            dv, cost, dt = footstep_planner(terrain_B, n_steps, span)
            n_used, lateral, _ = summarize_miqp(terrain_B, dv[0], dv[1])
            row.update({"feasible": True, "cost": float(cost),
                        "steps_used": n_used, "uses_lateral": bool(lateral),
                        "solve_time_s": float(dt)})
        except ValueError:
            row.update({"feasible": False})
        span_rows.append(row)
    results["span_sensitivity"] = span_rows

    feasible = [r for r in span_rows if r["feasible"]]
    fig, ax = plt.subplots(figsize=(8, 4.2))
    for route, color, marker in [("lateral", "tab:red", "o"),
                                 ("bridge", "tab:blue", "s")]:
        pts = [r for r in feasible
               if r["uses_lateral"] == (route == "lateral")]
        if pts:
            ax.plot([r["step_span"] for r in pts], [r["cost"] for r in pts],
                    marker + "-", color=color, label=f"route: {route}")
            for r in pts:
                ax.annotate(f"{r['steps_used']} steps",
                            (r["step_span"], r["cost"]),
                            textcoords="offset points", xytext=(4, 6),
                            fontsize=8)
    ax.axvline(0.75, color="0.5", ls=":", label="lower feasibility limit")
    ax.set_xlabel("step span [m]")
    ax.set_ylabel("MIQP cost")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    ax.set_title("Terrain B: effect of the step-span limit")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig5_6_span_sensitivity.png"), dpi=200)
    plt.close(fig)

    # ---- runtime scaling ----------------------------------------------
    scaling = []
    for n_bridge in [4, 6, 8]:
        terrain = Terrain([1] * n_bridge)
        n_steps_i = n_bridge + 5
        row = {"n_bridge_stones": n_bridge, "n_steps_limit": n_steps_i}
        try:
            _, miqp_cost, miqp_dt = footstep_planner(terrain, n_steps_i, step_span)
            row.update({"miqp_time_s": float(miqp_dt),
                        "miqp_cost": float(miqp_cost)})
        except Exception as exc:  # noqa: BLE001
            row.update({"miqp_time_s": None, "miqp_error": str(exc)[:80]})
        try:
            _, _, gcs_cost, gcs_dt = gcs_planner(terrain, step_span)
            row.update({"gcs_time_s": float(gcs_dt), "gcs_edges": int(gcs_cost)})
        except Exception as exc:  # noqa: BLE001
            row.update({"gcs_time_s": None, "gcs_error": str(exc)[:80]})
        scaling.append(row)
    results["scaling"] = scaling

    fig, ax = plt.subplots(figsize=(8, 4.2))
    xs = [r["n_bridge_stones"] for r in scaling]
    ax.semilogy(xs, [r.get("miqp_time_s") or np.nan for r in scaling],
                "o-", label="MIQP (branch and bound)")
    ax.semilogy(xs, [r.get("gcs_time_s") or np.nan for r in scaling],
                "s-", label="GCS (convex relaxation)")
    ax.set_xlabel("number of bridge stones")
    ax.set_ylabel("solve time [s]")
    ax.grid(True, alpha=0.3, which="both")
    ax.legend()
    ax.set_xticks(xs)
    ax.set_title("Scaling of footstep planning")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig5_7_runtime_scaling.png"), dpi=200)
    plt.close(fig)

    with open(os.path.join(HERE, "results_footstep.json"), "w") as f:
        json.dump(results, f, indent=2)
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
