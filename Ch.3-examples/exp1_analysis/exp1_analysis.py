"""Exp1-acrobot: reproduce the notebook results + sensitivity study.

Run with: /Users/leo/venvs/drake/bin/python exp1_analysis.py
"""
import json
import os
import numpy as np
from pydrake.all import (
    ControllabilityMatrix, DiagramBuilder, Linearize, LinearQuadraticRegulator,
    Saturation, WrapToSystem, Simulator,
)
from pydrake.examples import AcrobotPlant, AcrobotInput
from pydrake.systems.primitives import LogVectorOutput
import scipy.linalg as sla
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams.update({
    "font.sans-serif": ["Hiragino Sans GB", "Arial Unicode MS"],
    "axes.unicode_minus": False,
    "figure.dpi": 130,
})

HERE = os.path.dirname(os.path.abspath(__file__))
FIG = os.path.join(HERE, "figures")
os.makedirs(FIG, exist_ok=True)
np.set_printoptions(precision=4, suppress=True, linewidth=180)

X_STAR = np.array([np.pi, 0.0, 0.0, 0.0])
BASE_Q = np.diag([10.0, 10.0, 1.0, 1.0])
BASE_R = np.array([[1.0]])
SAT = 10.0
T = 4.0
ANGLE_TOL = 0.05   # rad
VEL_TOL = 0.2      # rad/s

PARAM_NAMES = ["m1", "m2", "l1", "l2", "lc1", "lc2", "Ic1", "Ic2", "b1", "b2", "gravity"]
# Parameters reproducing the lecture notes exactly (I about pivot -> Ic = I - m*lc^2)
NOTES_PARAMS = dict(m1=1.0, m2=1.0, l1=1.0, lc1=0.5, lc2=0.5,
                    Ic1=0.0, Ic2=0.0, b1=0.0, b2=0.0, gravity=9.81)


def wrap(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def make_plant_ctx(params_mod=None, with_input=True):
    plant = AcrobotPlant()
    ctx = plant.CreateDefaultContext()
    if params_mod:
        mp = plant.get_mutable_parameters(ctx)
        for k, v in params_mod.items():
            getattr(mp, f"set_{k}")(v)
    if with_input:
        inp = AcrobotInput()
        inp.set_tau(0.0)
        plant.get_input_port(0).FixValue(ctx, inp)
    ctx.get_mutable_continuous_state_vector().SetFromVector(X_STAR)
    return plant, ctx


def design_controller(params_mod=None, Q=BASE_Q, R=BASE_R):
    plant, ctx = make_plant_ctx(params_mod)
    return LinearQuadraticRegulator(plant, ctx, Q, R)


def linear_data(params_mod=None):
    plant, ctx = make_plant_ctx(params_mod)
    lin = Linearize(plant, ctx)
    A, B = lin.A(), lin.B()
    Cm = ControllabilityMatrix(lin).astype(float)
    sv = np.linalg.svd(Cm, compute_uv=False)
    lam, V = np.linalg.eig(A)
    beta = np.linalg.inv(V) @ B
    zeros = {}
    for name, cout in (("theta1", np.array([[1.0, 0, 0, 0]])),
                       ("theta2", np.array([[0.0, 1, 0, 0]]))):
        E = np.block([[np.eye(4), np.zeros((4, 1))],
                      [np.zeros((1, 4)), np.zeros((1, 1))]])
        M = np.block([[A, B], [cout, np.zeros((1, 1))]])
        z = sla.eigvals(M, E)
        zeros[name] = sorted([float(np.real(v)) for v in z if np.isfinite(v)])
    return dict(A=A, B=B, eig=lam, sv=sv, beta=beta.ravel(), zeros=zeros,
                cond=float(sv[0] / sv[-1]))


def build_diagram(params_mod=None, Q=BASE_Q, R=BASE_R, sat_limit=SAT):
    controller = design_controller(params_mod, Q, R)
    builder = DiagramBuilder()
    acrobot = builder.AddSystem(AcrobotPlant())
    saturation = builder.AddSystem(Saturation(min_value=[-sat_limit], max_value=[sat_limit]))
    builder.Connect(saturation.get_output_port(0), acrobot.get_input_port(0))
    wrapangles = WrapToSystem(4)
    wrapangles.set_interval(0, 0.0, 2.0 * np.pi)
    wrapangles.set_interval(1, -np.pi, np.pi)
    wrapto = builder.AddSystem(wrapangles)
    builder.Connect(acrobot.get_output_port(0), wrapto.get_input_port(0))
    ctrl = builder.AddSystem(controller)
    builder.Connect(wrapto.get_output_port(0), ctrl.get_input_port(0))
    builder.Connect(ctrl.get_output_port(0), saturation.get_input_port(0))
    log_x = LogVectorOutput(acrobot.get_output_port(0), builder, 0.01)
    log_u = LogVectorOutput(saturation.get_output_port(0), builder, 0.01)
    log_ucmd = LogVectorOutput(ctrl.get_output_port(0), builder, 0.01)
    diagram = builder.Build()
    return dict(diagram=diagram, acrobot=acrobot, log_x=log_x, log_u=log_u,
                log_ucmd=log_ucmd, K=-controller.D(), y0=float(controller.y0()[0]))


def simulate(diag, params_mod, x0, T=T):
    sim = Simulator(diag["diagram"])
    ctx = sim.get_mutable_context()
    if params_mod:
        pctx = diag["diagram"].GetMutableSubsystemContext(diag["acrobot"], ctx)
        mp = diag["acrobot"].get_mutable_parameters(pctx)
        for k, v in params_mod.items():
            getattr(mp, f"set_{k}")(v)
    ctx.SetContinuousState(np.asarray(x0, float))
    sim.Initialize()
    sim.AdvanceTo(T)
    ts = diag["log_x"].FindLog(ctx).sample_times().copy()
    X = diag["log_x"].FindLog(ctx).data().copy()
    U = diag["log_u"].FindLog(ctx).data().copy()
    Uc = diag["log_ucmd"].FindLog(ctx).data().copy()
    return ts, X, U, Uc


def metrics(ts, X, U, Uc):
    th1 = wrap(X[0] - np.pi)
    th2 = wrap(X[1])
    w1, w2 = X[2], X[3]
    e1, e2 = th1[-1], th2[-1]
    success = bool(abs(e1) < ANGLE_TOL and abs(e2) < ANGLE_TOL
                   and abs(w1[-1]) < VEL_TOL and abs(w2[-1]) < VEL_TOL)
    bad = (np.abs(th1) > ANGLE_TOL) | (np.abs(th2) > ANGLE_TOL) | \
          (np.abs(w1) > VEL_TOL) | (np.abs(w2) > VEL_TOL)
    if success:
        settle = float(ts[np.where(bad)[0][-1] + 1]) if bad.any() else 0.0
    else:
        settle = np.nan
    early = ts < 0.5
    return dict(success=success,
                final_abs_err=[float(abs(e1)), float(abs(e2)), float(abs(w1[-1])), float(abs(w2[-1]))],
                max_angle_excursion=float(np.max(np.hypot(th1, th2))),
                max_ucmd=float(np.max(np.abs(Uc[0]))),
                max_ucmd_early=float(np.max(np.abs(Uc[0][early]))),
                max_tau=float(np.max(np.abs(U[0]))),
                sat_frac_early=float(np.mean(np.abs(Uc[0][early]) > SAT)),
                settle_time=settle)


def run_condition(params_mod=None, Q=BASE_Q, R=BASE_R, sat_limit=SAT, x0s=None):
    diag = build_diagram(params_mod, Q, R, sat_limit)
    rows = []
    for x0 in x0s:
        ts, X, U, Uc = simulate(diag, params_mod, x0)
        rows.append(metrics(ts, X, U, Uc))
    return rows, diag


def summarize(rows):
    succ = np.array([r["success"] for r in rows])
    ok = [r for r in rows if r["success"]]
    return dict(
        n=len(rows), success=int(succ.sum()), success_rate=float(succ.mean()),
        med_max_ucmd_early=float(np.median([r["max_ucmd_early"] for r in rows])),
        p90_max_ucmd_early=float(np.percentile([r["max_ucmd_early"] for r in rows], 90)),
        med_excursion=float(np.median([r["max_angle_excursion"] for r in rows])),
        med_settle=float(np.nanmedian([r["settle_time"] for r in ok])) if ok else float("nan"),
        med_sat_frac_early=float(np.median([r["sat_frac_early"] for r in rows])),
    )


def dof(complex_list):
    return [[float(v.real), float(v.imag)] for v in complex_list]


def main():
    results = {}

    plant, ctx = make_plant_ctx()
    results["params_default"] = dict(zip(PARAM_NAMES, plant.get_parameters(ctx).CopyToVector().tolist()))
    print("Drake AcrobotPlant defaults:", results["params_default"])

    # ------------------------------------------------------------------ linear
    ld = linear_data()
    ctrl = design_controller()
    K = -ctrl.D().ravel()
    A, B = ld["A"], ld["B"]
    cl_eig = np.linalg.eigvals(A - B @ K.reshape(1, -1))
    results["linear_default"] = dict(
        A=A.tolist(), B=B.ravel().tolist(), eig_open=dof(ld["eig"]),
        sv=ld["sv"].tolist(), cond=ld["cond"], beta=ld["beta"].tolist(),
        zeros=ld["zeros"], K=K.tolist(), y0=float(ctrl.y0()[0]),
        eig_closed=dof(cl_eig))
    print("\n--- Drake defaults ---")
    print("A =\n", A)
    print("B =", B.ravel())
    print("open-loop eig:", np.sort_complex(ld["eig"]))
    print("controllability sv:", ld["sv"], " cond:", ld["cond"])
    print("modal beta:", ld["beta"])
    print("zeros:", ld["zeros"])
    print("K (u=-Kx):", K, " y0:", float(ctrl.y0()[0]))
    print("closed-loop eig:", np.sort_complex(cl_eig))

    ld_notes = linear_data(NOTES_PARAMS)
    ctrl_notes = design_controller(NOTES_PARAMS)
    K_notes = -ctrl_notes.D().ravel()
    cl_notes = np.linalg.eigvals(ld_notes["A"] - ld_notes["B"] @ K_notes.reshape(1, -1))
    results["linear_notes"] = dict(
        A=ld_notes["A"].tolist(), B=ld_notes["B"].ravel().tolist(),
        eig_open=dof(ld_notes["eig"]), sv=ld_notes["sv"].tolist(), cond=ld_notes["cond"],
        beta=ld_notes["beta"].tolist(), zeros=ld_notes["zeros"],
        K=K_notes.tolist(), y0=float(ctrl_notes.y0()[0]), eig_closed=dof(cl_notes))
    print("\n--- lecture-note parameters ---")
    print("A =\n", ld_notes["A"])
    print("open-loop eig:", np.sort_complex(ld_notes["eig"]))
    print("controllability sv:", ld_notes["sv"], " cond:", ld_notes["cond"])
    print("zeros:", ld_notes["zeros"])
    print("K (u=-Kx):", K_notes)
    print("closed-loop eig:", np.sort_complex(cl_notes))

    # ----------------------------------------------------- common perturbations
    rng = np.random.default_rng(42)
    draws100 = [rng.standard_normal(4) for _ in range(100)]
    x0s_005 = [X_STAR + 0.05 * w for w in draws100]

    # notebook cell 9 replica: 5 random trials (as in the notebook)
    rng_nb = np.random.default_rng(7)
    x0s_nb = [X_STAR + 0.05 * rng_nb.standard_normal(4) for _ in range(5)]
    rows_nb, _ = run_condition(x0s=x0s_nb, sat_limit=SAT)
    results["notebook_5trials"] = dict(rows=rows_nb, summary=summarize(rows_nb))
    print("\nnotebook cell-9 replica (5 trials):", summarize(rows_nb))

    # baseline: 100 trials with |tau| <= 10 and with no saturation
    rows_base, _ = run_condition(x0s=x0s_005, sat_limit=SAT)
    results["baseline_sat10"] = summarize(rows_base)
    rows_unb, _ = run_condition(x0s=x0s_005, sat_limit=1e6)
    results["baseline_unbounded"] = summarize(rows_unb)
    print("baseline 0.05*randn, sat=10:", results["baseline_sat10"])
    print("baseline 0.05*randn, unbounded:", results["baseline_unbounded"])

    # -------------------------------------------------- perturbation-size sweep
    deltas = [0.005, 0.01, 0.02, 0.03, 0.05, 0.075, 0.1, 0.15, 0.2]
    rng = np.random.default_rng(43)
    draws50 = [rng.standard_normal(4) for _ in range(50)]
    sweep = []
    for d in deltas:
        x0s = [X_STAR + d * w for w in draws50]
        r10, _ = run_condition(x0s=x0s, sat_limit=SAT)
        ru, _ = run_condition(x0s=x0s, sat_limit=1e6)
        s10, su = summarize(r10), summarize(ru)
        sweep.append(dict(delta=d, sat10=s10, unbounded=su))
        print(f"delta={d:<6}: sat10 {s10['success_rate']:.2f} (med|u|0.5s={s10['med_max_ucmd_early']:.1f})"
              f"  unbounded {su['success_rate']:.2f}")
    results["delta_sweep"] = sweep

    # ------------------------------------------------------------------ R sweep
    Rs = [0.1, 1.0, 10.0, 100.0]
    r_rows = []
    for R in Rs:
        rows, _ = run_condition(Q=BASE_Q, R=np.array([[R]]), x0s=x0s_005, sat_limit=SAT)
        c = design_controller(Q=BASE_Q, R=np.array([[R]]))
        KR = -c.D().ravel()
        eigs = np.linalg.eigvals(ld["A"] - ld["B"] @ KR.reshape(1, -1))
        row = dict(R=R, K=KR.tolist(), K_norm=float(np.linalg.norm(KR)),
                   eig_closed=dof(eigs), summary=summarize(rows))
        r_rows.append(row)
        print(f"R={R:<6}: success {row['summary']['success_rate']:.2f}  |K|={row['K_norm']:.1f}"
              f"  med|u|0.5s={row['summary']['med_max_ucmd_early']:.1f}  eigs={np.sort_complex(eigs)}")
    results["R_sweep"] = r_rows

    # ------------------------------------------------------------ Q weight sweep
    q_rows = []
    for w in [1.0, 10.0, 100.0]:
        Q = np.diag([w, w, 1.0, 1.0])
        rows, _ = run_condition(Q=Q, R=BASE_R, x0s=x0s_005, sat_limit=SAT)
        c = design_controller(Q=Q, R=BASE_R)
        KQ = -c.D().ravel()
        eigs = np.linalg.eigvals(ld["A"] - ld["B"] @ KQ.reshape(1, -1))
        q_rows.append(dict(w=w, K=KQ.tolist(), K_norm=float(np.linalg.norm(KQ)),
                           eig_closed=dof(eigs), summary=summarize(rows)))
        print(f"Qangle={w:<5}: success {q_rows[-1]['summary']['success_rate']:.2f}"
              f"  |K|={q_rows[-1]['K_norm']:.1f}  med|u|0.5s={q_rows[-1]['summary']['med_max_ucmd_early']:.1f}")
    results["Q_sweep"] = q_rows

    # ----------------------------------------------------- physical parameter sweeps
    def param_sweep(name, values):
        out = []
        for v in values:
            mods = {name: v}
            rows, _ = run_condition(params_mod=mods, x0s=x0s_005, sat_limit=SAT)
            ldp = linear_data(mods)
            c = design_controller(mods)
            Kp = -c.D().ravel()
            eigs = np.linalg.eigvals(ldp["A"] - ldp["B"] @ Kp.reshape(1, -1))
            out.append(dict(name=name, value=v, sv=ldp["sv"].tolist(), cond=ldp["cond"],
                            eig_open=dof(ldp["eig"]), K=Kp.tolist(),
                            eig_closed=dof(eigs), summary=summarize(rows)))
            print(f"{name}={v}: svmin={ldp['sv'][-1]:.4f}  cond={ldp['cond']:.0f}"
                  f"  success={out[-1]['summary']['success_rate']:.2f}"
                  f"  med|u|0.5s={out[-1]['summary']['med_max_ucmd_early']:.1f}"
                  f"  |K|={np.linalg.norm(Kp):.1f}")
        return out

    results["m2_sweep"] = param_sweep("m2", [0.5, 1.0, 2.0, 4.0])
    results["lc2_sweep"] = param_sweep("lc2", [0.5, 1.0, 1.5])

    # -------------------------------------------------- representative trajectories
    diag = build_diagram()
    cases = {
        "success_theta1": X_STAR + np.array([0.05, 0.0, 0.0, 0.0]),
        "failure_mixed": X_STAR + np.array([0.05, 0.03, 0.0, 0.0]),
    }
    traj = {}
    for key, x0 in cases.items():
        ts, X, U, Uc = simulate(diag, None, x0)
        m = metrics(ts, X, U, Uc)
        traj[key] = dict(t=ts.tolist(), x=X.tolist(), u=U[0].tolist(), ucmd=Uc[0].tolist(),
                         x0=x0.tolist(), success=m["success"])
        print(f"trajectory {key}: success={m['success']}  max|ucmd|={m['max_ucmd']:.1f}")
    results["trajectories"] = traj

    # ------------------------------------------------------------------ figures
    def plot_case(axs, tr, color, label):
        ts = np.array(tr["t"]); X = np.array(tr["x"])
        axs[0].plot(ts, X[0] - np.pi, color=color, label=label)
        axs[1].plot(ts, X[1], color=color, label=label)
        axs[2].plot(ts, X[2], color=color, label=label)
        axs[3].plot(ts, np.clip(np.array(tr["ucmd"]), -60, 60), color=color, lw=0.8, alpha=0.5)
        axs[3].plot(ts, np.array(tr["u"]), color=color, label=label)

    fig, axs = plt.subplots(2, 2, figsize=(9, 6), sharex=True)
    plot_case([axs[0, 0], axs[0, 1], axs[1, 0], axs[1, 1]], traj["success_theta1"], "tab:green",
              "成功: 仅 θ1 偏差 0.05")
    plot_case([axs[0, 0], axs[0, 1], axs[1, 0], axs[1, 1]], traj["failure_mixed"], "tab:red",
              "失败: θ1=0.05, θ2=0.03")
    titles = ["θ1 - π（原始值，未 wrap）[rad]", "θ2（原始值，未 wrap）[rad]", "角速度 [rad/s]", "肘关节力矩 [N·m]"]
    axs[0, 0].set_ylim(-1.0, 5.5); axs[0, 1].set_ylim(-10.5, 2.5); axs[1, 0].set_ylim(-20, 20)
    for ax, ti in zip(axs.ravel(), titles):
        ax.set_title(ti); ax.grid(alpha=0.3); ax.legend(fontsize=8); ax.set_xlabel("t [s]")
    for ax in axs[1, :]:
        ax.axhline(10, color="k", ls=":", lw=0.8)
        ax.axhline(-10, color="k", ls=":", lw=0.8)
    axs[1, 1].set_ylim(-15, 15)
    axs[1, 1].set_title("肘关节力矩 (细线=未饱和指令, 粗线=实际)")
    fig.suptitle("LQR 闭环：初始扰动的方向决定是否触发饱和失效")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig1_success_vs_failure.png"))
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(6, 3.8))
    ds = [s["delta"] for s in sweep]
    ax.plot(ds, [s["sat10"]["success_rate"] for s in sweep], "o-", label="|τ| ≤ 10 N·m")
    ax.plot(ds, [s["unbounded"]["success_rate"] for s in sweep], "s--", label="无输入饱和")
    ax.set_xlabel("扰动尺度 δ（x0 = x* + δ·N(0,I)）"); ax.set_ylabel("成功率（N=50）")
    ax.set_ylim(-0.02, 1.05); ax.grid(alpha=0.3); ax.legend()
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "fig2_delta_sweep.png")); plt.close(fig)

    fig, axs = plt.subplots(1, 2, figsize=(9, 3.6))
    m2 = [r["value"] for r in results["m2_sweep"]]
    axs[0].plot(m2, [r["sv"][-1] for r in results["m2_sweep"]], "o-")
    axs[0].set_xlabel("m2 [kg]"); axs[0].set_ylabel("σ_min(C)"); axs[0].set_yscale("log")
    axs[0].grid(alpha=0.3); axs[0].set_title("可控性最弱方向 σ_min")
    axs[0].set_yticks([0.2, 0.1, 0.05, 0.02, 0.01, 0.006])
    axs[0].set_yticklabels(["0.2", "0.1", "0.05", "0.02", "0.01", "0.006"])
    axs[1].plot(m2, [r["summary"]["success_rate"] for r in results["m2_sweep"]], "s-", color="tab:red")
    axs[1].set_xlabel("m2 [kg]"); axs[1].set_ylabel("成功率（δ=0.05, N=100）")
    axs[1].set_ylim(-0.02, 1.05); axs[1].grid(alpha=0.3); axs[1].set_title("饱和 LQR 成功率")
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "fig3_m2_sweep.png")); plt.close(fig)

    fig, ax = plt.subplots(figsize=(6, 3.8))
    ax.plot(Rs, [r["K_norm"] for r in r_rows], "o-", label="‖K‖")
    ax.set_xscale("log"); ax.set_xlabel("R"); ax.set_ylabel("‖K‖")
    ax.set_xticks(Rs); ax.set_xticklabels(["0.1", "1", "10", "100"])
    ax.grid(alpha=0.3); ax.legend(loc="upper right")
    ax2 = ax.twinx()
    ax2.plot(Rs, [r["summary"]["success_rate"] for r in r_rows], "s--", color="tab:red", label="成功率")
    ax2.set_ylabel("成功率（δ=0.05, N=100）", color="tab:red"); ax2.set_ylim(-0.02, 1.05)
    ax.set_title("增大 R 时增益趋于有限值（昂贵控制极限）")
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "fig4_R_sweep.png")); plt.close(fig)

    def enc(o):
        if isinstance(o, complex):
            return [o.real, o.imag]
        if isinstance(o, np.ndarray):
            return o.tolist()
        if isinstance(o, (np.floating, np.integer)):
            return o.item()
        raise TypeError
    with open(os.path.join(HERE, "results.json"), "w") as f:
        json.dump(results, f, indent=1, default=enc)
    print("\nsaved:", os.path.join(HERE, "results.json"))
    print("figures in", FIG)


if __name__ == "__main__":
    main()
