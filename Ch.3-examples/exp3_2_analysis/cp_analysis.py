"""Exp3'2-cartpole: reproduce the notebook results + sensitivity study.

Run with: /Users/leo/venvs/drake/bin/python cp_analysis.py
"""
import json
import os
import numpy as np
import scipy.linalg as sla
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from pydrake.all import (
    AddMultibodyPlantSceneGraph, ControllabilityMatrix, DiagramBuilder,
    Linearize, LinearQuadraticRegulator, Parser, Saturation, Simulator,
)
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import RotationalInertia, SpatialInertia
from pydrake.systems.primitives import LogVectorOutput
from underactuated import ConfigureParser

plt.rcParams.update({
    "font.sans-serif": ["Arial Unicode MS", "Hiragino Sans GB"],
    "axes.unicode_minus": False,
    "figure.dpi": 130,
})

HERE = os.path.dirname(os.path.abspath(__file__))
FIG = os.path.join(HERE, "figures")
os.makedirs(FIG, exist_ok=True)
np.set_printoptions(precision=4, suppress=True, linewidth=180)

X_STAR = np.array([0.0, np.pi, 0.0, 0.0])
BASE_Q = np.diag([10.0, 10.0, 1.0, 1.0])
BASE_R = np.array([1.0])
MC, MP, LC = 10.0, 1.0, 0.5      # URDF defaults
T = 5.0
ANGLE_TOL, VEL_TOL = 0.05, 0.2   # pole: rad, rad/s
CART_TOL, CART_VEL_TOL = 0.5, 0.5  # cart stays bounded (m, m/s)
FORCE_LIMIT = 1e6                # notebook has no saturation

MODEL_URL = "package://underactuated/models/cartpole.urdf"


def wrap(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def make_plant(builder=None, gravity=9.81):
    if builder is None:
        plant = MultibodyPlant(0.0)
        sg = None
    else:
        plant, sg = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    parser = Parser(plant)
    ConfigureParser(parser)
    parser.AddModelsFromUrl(MODEL_URL)
    plant.Finalize()
    plant.mutable_gravity_field().set_gravity_vector(np.array([0.0, 0.0, -gravity]))
    return plant, sg


def set_model_params(plant, ctx, mc=MC, mp=MP, lc=LC):
    plant.GetBodyByName("cart").SetMass(ctx, mc)
    M = SpatialInertia.MakeFromCentralInertia(
        mp, np.array([0.0, 0.0, -lc]), RotationalInertia(0, 0, 0))
    plant.GetBodyByName("pole").SetSpatialInertiaInBodyFrame(ctx, M)


def make_ctx(plant, mc=MC, mp=MP, lc=LC):
    ctx = plant.CreateDefaultContext()
    plant.get_actuation_input_port().FixValue(ctx, [0.0])
    plant.SetPositionsAndVelocities(ctx, X_STAR)
    set_model_params(plant, ctx, mc, mp, lc)
    return ctx


def linear_data(mc=MC, mp=MP, lc=LC, gravity=9.81):
    plant, _ = make_plant(gravity=gravity)
    ctx = make_ctx(plant, mc, mp, lc)
    lin = Linearize(
        plant, ctx,
        input_port_index=plant.get_actuation_input_port().get_index(),
        output_port_index=plant.get_state_output_port().get_index())
    A, B = lin.A(), lin.B()
    Cm = ControllabilityMatrix(lin).astype(float)
    sv = np.linalg.svd(Cm, compute_uv=False)
    lam = np.linalg.eigvals(A)
    pbh = [int(np.linalg.matrix_rank(np.hstack([li * np.eye(4) - A, B])))
           for li in sorted_complex(np.unique(np.round(lam, 10)))]
    zeros = {}
    E = np.block([[np.eye(4), np.zeros((4, 1))],
                  [np.zeros((1, 4)), np.zeros((1, 1))]])
    for name, cout in (("u->x", np.array([[1.0, 0, 0, 0]])),
                       ("u->θ", np.array([[0.0, 1, 0, 0]]))):
        M = np.block([[A, B], [cout, np.zeros((1, 1))]])
        z = sla.eigvals(M, E)
        zeros[name] = sorted(float(np.real(v)) for v in z if np.isfinite(v))
    # finite-horizon controllability Gramian (normalized only by time)
    W = sla.solve_continuous_lyapunov(A, -B @ B.T) if np.all(np.real(lam) < 0) else None
    return dict(A=A, B=B, Cm=Cm, sv=sv, eig=lam, rank=int(np.linalg.matrix_rank(Cm)),
                pbh=pbh, zeros=zeros, cond=float(sv[0] / sv[-1]))


def design(Q=BASE_Q, R=BASE_R, mc=MC, mp=MP, lc=LC, gravity=9.81):
    plant, _ = make_plant(gravity=gravity)
    ctx = make_ctx(plant, mc, mp, lc)
    return LinearQuadraticRegulator(
        plant, ctx, Q, R,
        input_port_index=plant.get_actuation_input_port().get_index())


def build_sim(controller, mc=MC, mp=MP, lc=LC, lim=FORCE_LIMIT):
    builder = DiagramBuilder()
    plant, scene_graph = make_plant(builder)
    ctrl = builder.AddSystem(controller)
    sat = builder.AddSystem(Saturation([-lim], [lim]))
    builder.Connect(plant.get_state_output_port(), ctrl.get_input_port(0))
    builder.Connect(ctrl.get_output_port(0), sat.get_input_port(0))
    builder.Connect(sat.get_output_port(0), plant.get_actuation_input_port())
    log_x = LogVectorOutput(plant.get_state_output_port(), builder, 0.01)
    log_u = LogVectorOutput(sat.get_output_port(), builder, 0.01)
    log_ucmd = LogVectorOutput(ctrl.get_output_port(0), builder, 0.01)
    diagram = builder.Build()
    return dict(diagram=diagram, plant=plant, log_x=log_x, log_u=log_u,
                log_ucmd=log_ucmd, D=controller.D().ravel())


def simulate(diag, x0, mc=MC, mp=MP, lc=LC, T=T):
    sim = Simulator(diag["diagram"])
    ctx = sim.get_mutable_context()
    plant = diag["plant"]
    pctx = plant.GetMyMutableContextFromRoot(ctx)
    set_model_params(plant, pctx, mc, mp, lc)
    plant.SetPositionsAndVelocities(pctx, np.asarray(x0, float))
    sim.Initialize()
    sim.AdvanceTo(T)
    return (diag["log_x"].FindLog(ctx).sample_times().copy(),
            diag["log_x"].FindLog(ctx).data().copy(),
            diag["log_u"].FindLog(ctx).data().copy(),
            diag["log_ucmd"].FindLog(ctx).data().copy())


def metrics(ts, X, U, Uc):
    x, th, xd, thd = X[0], wrap(X[1] - np.pi), X[2], X[3]
    pole_ok = bool(abs(th[-1]) < ANGLE_TOL and abs(thd[-1]) < VEL_TOL)
    cart_ok = bool(abs(x[-1]) < CART_TOL and abs(xd[-1]) < CART_VEL_TOL)
    success = pole_ok and cart_ok
    bad = (np.abs(th) > ANGLE_TOL) | (np.abs(thd) > VEL_TOL)
    if pole_ok:
        settle = float(ts[np.where(bad)[0][-1] + 1]) if bad.any() else 0.0
    else:
        settle = np.nan
    early = ts < 0.5
    return dict(success=success, pole_ok=pole_ok, cart_ok=cart_ok, settle=settle,
                final_x=float(abs(x[-1])), final_ang=float(abs(th[-1])),
                max_x=float(np.max(np.abs(x))),
                peak_ucmd_early=float(np.max(np.abs(Uc[0][early]))),
                max_force=float(np.max(np.abs(U[0]))),
                sat_frac=float(np.mean(np.abs(Uc[0]) > 1e5)) if np.max(np.abs(Uc[0])) > 1e5 else 0.0)


def run_condition(mc=MC, mp=MP, lc=LC, Q=BASE_Q, R=BASE_R, lim=FORCE_LIMIT, x0s=None):
    ctrl = design(Q, R, mc, mp, lc)
    diag = build_sim(ctrl, mc, mp, lc, lim)
    rows = [metrics(*simulate(diag, x0, mc, mp, lc)) for x0 in x0s]
    return rows, diag


def summarize(rows):
    ok = [r for r in rows if r["success"]]
    return dict(n=len(rows), success=int(sum(r["success"] for r in rows)),
                rate=float(np.mean([r["success"] for r in rows])),
                pole_rate=float(np.mean([r["pole_ok"] for r in rows])),
                cart_rate=float(np.mean([r["cart_ok"] for r in rows])),
                peak=float(np.median([r["peak_ucmd_early"] for r in rows])),
                peak90=float(np.percentile([r["peak_ucmd_early"] for r in rows], 90)),
                max_x=float(np.median([r["max_x"] for r in rows])),
                final_x=float(np.median([r["final_x"] for r in rows])),
                settle=(float(np.nanmedian([r["settle"] for r in ok])) if ok else float("nan")))


def dof(vals):
    return [[float(v.real), float(v.imag)] for v in vals]


def sorted_complex(vals):
    return sorted(vals, key=lambda z: (round(z.real, 6), round(z.imag, 6)))


def main():
    R = {}

    # ------------------------------------------------------------- baseline
    ld = linear_data()
    ctrl = design()
    K = ctrl.D().ravel()
    A, B = ld["A"], ld["B"]
    cl = np.linalg.eigvals(A + B @ K.reshape(1, -1))  # u = D(x-x*)
    R["linear_default"] = dict(A=A.tolist(), B=B.ravel().tolist(),
                               eig=dof(ld["eig"]), sv=ld["sv"].tolist(),
                               cond=ld["cond"], rank=ld["rank"], pbh=ld["pbh"],
                               zeros=ld["zeros"], D=K.tolist(),
                               y0=float(ctrl.y0()[0]), eig_closed=dof(cl))
    print("=== cart-pole defaults: mc=10, mp=1, lc=0.5, g=9.81 ===")
    print("A =\n", A)
    print("B =", B.ravel())
    print("open-loop eig:", np.sort_complex(ld["eig"]))
    print("rank(C) =", ld["rank"], " PBH ranks:", ld["pbh"])
    print("σ(C) =", ld["sv"], " cond =", ld["cond"])
    print("zeros:", ld["zeros"])
    print("D =", K, " y0 =", float(ctrl.y0()[0]))
    print("closed-loop eig:", np.sort_complex(cl))

    # note parameters (g=1) to validate the lecture notes
    ldn = linear_data(mc=1.0, mp=1.0, lc=1.0, gravity=1.0)
    ctrln = design(mc=1.0, mp=1.0, lc=1.0, gravity=1.0)
    Kn = ctrln.D().ravel()
    cln = np.linalg.eigvals(ldn["A"] + ldn["B"] @ Kn.reshape(1, -1))
    R["linear_notes"] = dict(A=ldn["A"].tolist(), B=ldn["B"].ravel().tolist(),
                             eig=dof(ldn["eig"]), sv=ldn["sv"].tolist(),
                             cond=ldn["cond"], rank=ldn["rank"], pbh=ldn["pbh"],
                             zeros=ldn["zeros"], D=Kn.tolist(),
                             y0=float(ctrln.y0()[0]), eig_closed=dof(cln))
    print("\n=== lecture-note model: mc=mp=1, lc=1, g=1 ===")
    print("A =\n", ldn["A"])
    print("B =", ldn["B"].ravel())
    print("open-loop eig:", np.sort_complex(ldn["eig"]))
    print("σ(C) =", ldn["sv"])
    print("zeros:", ldn["zeros"])
    print("D =", Kn, " y0 =", float(ctrln.y0()[0]))

    # common perturbations
    rng = np.random.default_rng(42)
    draws100 = [rng.standard_normal(4) for _ in range(100)]
    x0s_01 = [X_STAR + 0.1 * w for w in draws100]

    # notebook cell 7 replica: 5 random trials
    rng5 = np.random.default_rng(7)
    rows5, _ = run_condition(x0s=[X_STAR + 0.1 * rng5.standard_normal(4) for _ in range(5)])
    R["notebook_5trials"] = summarize(rows5)
    print("\nnotebook 5-trial replica:", R["notebook_5trials"])

    # 100 random trials, no force limit (as in the notebook)
    rows_base, _ = run_condition(x0s=x0s_01)
    R["baseline_no_limit"] = summarize(rows_base)
    print("baseline 0.1*randn, no force limit:", R["baseline_no_limit"])
    print("  per-trial peak force: median %.1f, p90 %.1f, max %.1f N" % (
        np.median([r["max_force"] for r in rows_base]),
        np.percentile([r["max_force"] for r in rows_base], 90),
        np.max([r["max_force"] for r in rows_base])))

    # ------------------------------------------------------ delta sweep
    deltas = [0.01, 0.02, 0.05, 0.1, 0.15, 0.2, 0.3, 0.5]
    rng_d = np.random.default_rng(43)
    draws50 = [rng_d.standard_normal(4) for _ in range(50)]
    sweep = []
    for d in deltas:
        x0s = [X_STAR + d * w for w in draws50]
        s_free = summarize(run_condition(x0s=x0s)[0])
        s_lim = summarize(run_condition(x0s=x0s, lim=5.0)[0])
        sweep.append(dict(delta=d, free=s_free, lim5=s_lim))
        print(f"delta={d:<5}: no limit {s_free['rate']:.2f} (pole {s_free['pole_rate']:.2f}) |  |u|<=5 N {s_lim['rate']:.2f} (pole {s_lim['pole_rate']:.2f})"
              f"  peak|u| med {s_free['peak']:.1f} N")
    R["delta_sweep"] = sweep

    # ------------------------------------------------------ force limit sweep
    limits = [0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 1e6]
    fl = []
    for lim in limits:
        s = summarize(run_condition(x0s=x0s_01, lim=lim)[0])
        fl.append(dict(limit=lim, summary=s))
        print(f"force limit {lim:>10.1f}: success {s['rate']:.2f} (pole {s['pole_rate']:.2f})  peak|u| {s['peak']:.1f} N"
              f"  max|x| {s['max_x']:.2f} m")
    R["force_limit_sweep"] = fl

    # ------------------------------------------------------ R and Q sweeps
    rr = []
    for Rv in [0.01, 0.1, 1.0, 10.0, 100.0]:
        c = design(R=np.array([Rv]))
        Dv = c.D().ravel()
        s = summarize(run_condition(R=np.array([Rv]), x0s=x0s_01)[0])
        rr.append(dict(R=Rv, D=Dv.tolist(), normD=float(np.linalg.norm(Dv)), summary=s))
        print(f"R={Rv:<6}: success {s['rate']:.2f}  |D|={np.linalg.norm(Dv):.2f}"
              f"  peak|u| med {s['peak']:.1f} N  final|x| {s['final_x']:.2f} m")
    R["R_sweep"] = rr

    qr = []
    for w in [1.0, 10.0, 100.0]:
        Q = np.diag([10.0, w, 1.0, 1.0])
        c = design(Q=Q)
        Dv = c.D().ravel()
        s = summarize(run_condition(Q=Q, x0s=x0s_01)[0])
        qr.append(dict(w=w, D=Dv.tolist(), normD=float(np.linalg.norm(Dv)), summary=s))
        print(f"Q_theta={w:<5}: success {s['rate']:.2f}  |D|={np.linalg.norm(Dv):.2f}"
              f"  peak|u| med {s['peak']:.1f} N")
    R["Q_sweep"] = qr

    # ------------------------------------------------------ physical sweeps
    def param_sweep(name, values, key):
        out = []
        for v in values:
            mods = dict(mc=MC, mp=MP, lc=LC); mods[key] = v
            ldv = linear_data(**mods)
            c = design(**mods)
            Dv = c.D().ravel()
            s = summarize(run_condition(**mods, x0s=x0s_01)[0])
            out.append(dict(value=v, sv=ldv["sv"].tolist(), cond=ldv["cond"],
                            eig=dof(ldv["eig"]), D=Dv.tolist(),
                            normD=float(np.linalg.norm(Dv)), summary=s))
            print(f"{name}={v:<6}: σmin={ldv['sv'][-1]:.4f}  cond={ldv['cond']:.0f}"
                  f"  success={s['rate']:.2f}  peak|u|={s['peak']:.1f} N"
                  f"  max|x|={s['max_x']:.2f} m")
        return out

    R["mc_sweep"] = param_sweep("mc", [2.0, 5.0, 10.0, 20.0], "mc")
    R["mp_sweep"] = param_sweep("mp", [0.1, 0.5, 1.0, 2.0], "mp")
    R["lc_sweep"] = param_sweep("lc", [0.25, 0.5, 1.0], "lc")

    # ------------------------------------------------------ trajectories
    # Same initial condition: no limit (success) vs 2 N limit (failure)
    x0_case = X_STAR + 0.1 * np.array([0.5, 1.0, 0.2, 1.0])
    traj = {}
    for key, lim in (("no_limit", FORCE_LIMIT), ("lim2", 2.0)):
        ctrl = design()
        diag = build_sim(ctrl, lim=lim)
        ts, X, U, Uc = simulate(diag, x0_case)
        m = metrics(ts, X, U, Uc)
        traj[key] = dict(t=list(ts), x=X.tolist(), u=U[0].tolist(),
                         ucmd=Uc[0].tolist(), success=m["success"])
        print(f"trajectory {key}: success={m['success']} peak|ucmd|={m['peak_ucmd_early']:.1f} N")
    R["trajectories"] = traj
    R["trajectory_x0"] = x0_case.tolist()

    # ------------------------------------------------------ figures
    fig, axs = plt.subplots(2, 2, figsize=(9, 6), sharex=True)
    for key, color, label in (("no_limit", "tab:green", "无输入限幅"),
                              ("lim2", "tab:red", "|u| ≤ 2 N")):
        tr = traj[key]
        ts = np.array(tr["t"]); X = np.array(tr["x"])
        axs[0, 0].plot(ts, X[0], color=color, label=label)
        axs[0, 1].plot(ts, X[1] - np.pi, color=color, label=label)
        axs[1, 0].plot(ts, X[3], color=color, label=label)
        axs[1, 1].plot(ts, np.array(tr["u"]), color=color, label=label)
    for ax, ti in zip(axs.ravel(), ["小车位置 x [m]", "θ - π [rad]",
                                    "θ̇ [rad/s]", "小车推力 u [N]"]):
        ax.set_title(ti); ax.grid(alpha=0.3); ax.legend(fontsize=8); ax.set_xlabel("t [s]")
    axs[0, 1].set_ylim(-2.0, 10.0)
    axs[0, 1].set_title("θ - π（原始值，未 wrap）[rad]")
    fig.suptitle("Cart-Pole LQR：同一初始状态，有无推力限幅的对比")
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "fig1_success_vs_limit.png")); plt.close(fig)

    fig, ax = plt.subplots(figsize=(6, 3.8))
    ds = [s["delta"] for s in sweep]
    ax.plot(ds, [s["free"]["pole_rate"] for s in sweep], "o-", label="无输入限幅")
    ax.plot(ds, [s["lim5"]["pole_rate"] for s in sweep], "s--", label="|u| ≤ 5 N")
    ax.set_xlabel("扰动尺度 δ（x0 = x* + δ·N(0,I)）"); ax.set_ylabel("摆杆平衡成功率（N=50）")
    ax.set_ylim(-0.02, 1.05); ax.grid(alpha=0.3); ax.legend()
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "fig2_delta_sweep.png")); plt.close(fig)

    fig, ax = plt.subplots(figsize=(6, 3.8))
    xs = np.arange(len(fl))
    ax.plot(xs, [f["summary"]["pole_rate"] for f in fl], "o-", label="摆杆平衡")
    ax.plot(xs, [f["summary"]["rate"] for f in fl], "s--", label="摆杆+小车回到容差")
    ax.set_xticks(xs)
    ax.set_xticklabels(["0.5", "1", "2", "5", "10", "20", "50", "∞"])
    ax.set_xlabel("推力限幅 [N]（∞ = 无饱和）"); ax.set_ylabel("成功率（δ=0.1, N=100）")
    ax.set_ylim(-0.02, 1.05); ax.grid(alpha=0.3); ax.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "fig3_force_limit.png")); plt.close(fig)

    fig, axs = plt.subplots(1, 2, figsize=(9, 3.6))
    mc_vals = [r["value"] for r in R["mc_sweep"]]
    axs[0].plot(mc_vals, [r["sv"][-1] for r in R["mc_sweep"]], "o-")
    axs[0].set_xlabel("mc [kg]"); axs[0].set_ylabel("σ_min(C)")
    axs[0].set_yscale("log"); axs[0].grid(alpha=0.3)
    axs[0].set_title("小车质量增大 → 最弱可控方向变差")
    axs[1].plot(mc_vals, [r["summary"]["peak"] for r in R["mc_sweep"]], "s-", color="tab:red")
    axs[1].set_xlabel("mc [kg]"); axs[1].set_ylabel("前 0.5 s 峰值推力中位数 [N]")
    axs[1].grid(alpha=0.3)
    axs[1].set_title("小车质量增大 → 所需控制力增大")
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "fig4_physical.png")); plt.close(fig)

    def enc(o):
        if isinstance(o, complex):
            return [o.real, o.imag]
        if isinstance(o, np.ndarray):
            return o.tolist()
        if isinstance(o, (np.floating, np.integer)):
            return o.item()
        raise TypeError
    with open(os.path.join(HERE, "results.json"), "w") as f:
        json.dump(R, f, indent=1, default=enc)
    print("\nsaved:", os.path.join(HERE, "results.json"))
    print("figures in", FIG)


if __name__ == "__main__":
    main()
