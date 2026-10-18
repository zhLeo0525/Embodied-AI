"""Example 3.5 (LQR for the Acrobot and Cart-Pole): comparison + sensitivity study.

Reuses the verified helpers from the previous two analyses.
Run with: /Users/leo/venvs/drake/bin/python lqr_comparison.py
"""
import json
import os
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import NullFormatter

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import exp1_analysis as acro   # noqa: E402
import cp_analysis as cp       # noqa: E402

FIG = os.path.join(HERE, "figures")
os.makedirs(FIG, exist_ok=True)

plt.rcParams.update({
    "font.sans-serif": ["Arial Unicode MS", "Hiragino Sans GB"],
    "axes.unicode_minus": False,
    "figure.dpi": 130,
})
np.set_printoptions(precision=4, suppress=True, linewidth=180)


def enc(o):
    if isinstance(o, complex):
        return [o.real, o.imag]
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    return o


def eig(A, B, D):
    return np.linalg.eigvals(A + B @ D.reshape(1, -1))


def slowest(eigs):
    return float(np.min(np.abs(eigs)))


def threshold90(ds, rates):
    good = [d for d, r in zip(ds, rates) if r >= 0.9]
    return max(good) if good else None


def main():
    R = {}

    # ---------------------------------------------------------- linear analysis
    aL = acro.linear_data()
    cL = cp.linear_data()
    aD = acro.design_controller().D().ravel()
    cD = cp.design().D().ravel()
    a_cl = eig(aL["A"], aL["B"], aD)
    c_cl = eig(cL["A"], cL["B"], cD)

    R["acrobot_linear"] = dict(
        A=aL["A"].tolist(), B=aL["B"].ravel().tolist(), eig=aL["eig"],
        sv=aL["sv"].tolist(), cond=aL["cond"], zeros=aL["zeros"],
        D=aD.tolist(), eig_closed=a_cl)
    R["cartpole_linear"] = dict(
        A=cL["A"].tolist(), B=cL["B"].ravel().tolist(), eig=cL["eig"],
        sv=cL["sv"].tolist(), cond=cL["cond"], zeros=cL["zeros"],
        D=cD.tolist(), eig_closed=c_cl)

    print("=== Example 3.5: linear analysis ===")
    print("[Acrobot] eigs:", np.sort_complex(aL["eig"]))
    print("[Acrobot] sv:", aL["sv"], " cond:", aL["cond"])
    print("[Acrobot] zeros:", aL["zeros"])
    print("[Acrobot] D:", aD, " closed-loop:", np.sort_complex(a_cl))
    print("[CartPole] eigs:", np.sort_complex(cL["eig"]))
    print("[CartPole] sv:", cL["sv"], " cond:", cL["cond"])
    print("[CartPole] zeros:", cL["zeros"])
    print("[CartPole] D:", cD, " closed-loop:", np.sort_complex(c_cl))

    # ------------------------------------------------------------- baselines
    rng = np.random.default_rng(42)
    a_draws = [rng.standard_normal(4) for _ in range(100)]
    c_draws = [rng.standard_normal(4) for _ in range(100)]
    a_x0 = [acro.X_STAR + 0.05 * w for w in a_draws]
    c_x0 = [cp.X_STAR + 0.1 * w for w in c_draws]

    a_sat = acro.summarize(acro.run_condition(x0s=a_x0, sat_limit=10.0)[0])
    a_free = acro.summarize(acro.run_condition(x0s=a_x0, sat_limit=1e6)[0])
    c_free = cp.summarize(cp.run_condition(x0s=c_x0, lim=1e6)[0])
    c_20 = cp.summarize(cp.run_condition(x0s=c_x0, lim=20.0)[0])
    c_5 = cp.summarize(cp.run_condition(x0s=c_x0, lim=5.0)[0])
    R["acrobot_baseline"] = {"sat10": a_sat, "unbounded": a_free}
    R["cartpole_baseline"] = {"unbounded": c_free, "lim20": c_20, "lim5": c_5}
    print("\n=== baselines ===")
    print("[Acrobot] 0.05*randn, sat10:", a_sat)
    print("[Acrobot] 0.05*randn, unbounded:", a_free)
    print("[CartPole] 0.1*randn, unbounded:", c_free)
    print("[CartPole] 0.1*randn, 20N:", c_20)
    print("[CartPole] 0.1*randn, 5N:", c_5)

    # ------------------------------------------------------- robustness sweep
    a_deltas = [0.005, 0.01, 0.02, 0.03, 0.05, 0.075, 0.1, 0.15, 0.2]
    rng_a = np.random.default_rng(43)
    a_draws50 = [rng_a.standard_normal(4) for _ in range(50)]
    a_rob = []
    for d in a_deltas:
        x0s = [acro.X_STAR + d * w for w in a_draws50]
        s10 = acro.summarize(acro.run_condition(x0s=x0s, sat_limit=10.0)[0])
        sfree = acro.summarize(acro.run_condition(x0s=x0s, sat_limit=1e6)[0])
        a_rob.append(dict(delta=d, sat10=s10, free=sfree))
        print(f"[Acrobot] delta={d:<6}: sat10 {s10['success_rate']:.2f} | free {sfree['success_rate']:.2f}")

    c_deltas = [0.01, 0.02, 0.05, 0.1, 0.15, 0.2, 0.3, 0.5]
    rng_c = np.random.default_rng(44)
    c_draws50 = [rng_c.standard_normal(4) for _ in range(50)]
    c_rob = []
    for d in c_deltas:
        x0s = [cp.X_STAR + d * w for w in c_draws50]
        sf = cp.summarize(cp.run_condition(x0s=x0s, lim=1e6)[0])
        s20 = cp.summarize(cp.run_condition(x0s=x0s, lim=20.0)[0])
        s5 = cp.summarize(cp.run_condition(x0s=x0s, lim=5.0)[0])
        c_rob.append(dict(delta=d, free=sf, lim20=s20, lim5=s5))
        print(f"[CartPole] delta={d:<5}: free {sf['pole_rate']:.2f} | 20N {s20['pole_rate']:.2f} | 5N {s5['pole_rate']:.2f}")
    R["acrobot_robustness"] = a_rob
    R["cartpole_robustness"] = c_rob

    # ------------------------------------------------------------- R sweeps
    a_Rs = [0.1, 1.0, 10.0, 100.0]
    a_R = []
    for Rv in a_Rs:
        ctrl = acro.design_controller(R=np.array([[Rv]]))
        Dv = ctrl.D().ravel()
        eigs = eig(aL["A"], aL["B"], Dv)
        s = acro.summarize(acro.run_condition(R=np.array([[Rv]]), x0s=a_x0, sat_limit=10.0)[0])
        a_R.append(dict(R=Rv, D=Dv.tolist(), normD=float(np.linalg.norm(Dv)),
                        eig_closed=eigs, slowest=slowest(eigs), summary=s))
        print(f"[Acrobot] R={Rv:<6}: |D|={np.linalg.norm(Dv):.2f} slow={slowest(eigs):.3f} "
              f"success={s['success_rate']:.2f} peak={s['med_max_ucmd_early']:.1f} N·m")

    c_Rs = [0.01, 0.1, 1.0, 10.0, 100.0]
    c_R = []
    for Rv in c_Rs:
        ctrl = cp.design(R=np.array([Rv]))
        Dv = ctrl.D().ravel()
        eigs = eig(cL["A"], cL["B"], Dv)
        s = cp.summarize(cp.run_condition(R=np.array([Rv]), x0s=c_x0, lim=1e6)[0])
        c_R.append(dict(R=Rv, D=Dv.tolist(), normD=float(np.linalg.norm(Dv)),
                        eig_closed=eigs, slowest=slowest(eigs), summary=s))
        print(f"[CartPole] R={Rv:<6}: |D|={np.linalg.norm(Dv):.2f} slow={slowest(eigs):.3f} "
              f"pole={s['pole_rate']:.2f} full={s['rate']:.2f} final|x|={s['final_x']:.2f}")
    R["acrobot_R"] = a_R
    R["cartpole_R"] = c_R

    # ------------------------------------------------------- physical sweeps
    a_m2 = []
    for v in [0.5, 1.0, 2.0, 4.0]:
        mods = {"m2": v}
        lv = acro.linear_data(mods)
        ctrl = acro.design_controller(mods)
        s = acro.summarize(acro.run_condition(params_mod=mods, x0s=a_x0, sat_limit=10.0)[0])
        a_m2.append(dict(value=v, sv=lv["sv"].tolist(), cond=lv["cond"],
                         normD=float(np.linalg.norm(ctrl.D())), summary=s))
        print(f"[Acrobot] m2={v}: svmin={lv['sv'][-1]:.4f} success={s['success_rate']:.2f} "
              f"peak={s['med_max_ucmd_early']:.1f} N·m")

    c_mc = []
    for v in [2.0, 5.0, 10.0, 20.0]:
        lv = cp.linear_data(mc=v)
        ctrl = cp.design(mc=v)
        s = cp.summarize(cp.run_condition(mc=v, x0s=c_x0, lim=1e6)[0])
        c_mc.append(dict(value=v, sv=lv["sv"].tolist(), cond=lv["cond"],
                         normD=float(np.linalg.norm(ctrl.D())), summary=s))
        print(f"[CartPole] mc={v}: svmin={lv['sv'][-1]:.4f} pole={s['pole_rate']:.2f} "
              f"peak={s['peak']:.1f} N final|x|={s['final_x']:.2f}")
    R["acrobot_m2"] = a_m2
    R["cartpole_mc"] = c_mc

    # ------------------------------------------------------------ comparison
    comp = {
        "acrobot": {
            "sv": aL["sv"].tolist(), "cond": aL["cond"], "zeros": aL["zeros"],
            "rank": int(np.sum(np.array(aL["sv"]) > 1e-12)), "normD": float(np.linalg.norm(aD)),
            "slowest_cl": slowest(a_cl),
            "rhp_zeros": [z for z in aL["zeros"]["theta1"] + aL["zeros"]["theta2"] if z > 0],
            "baseline_unbounded": a_free["success_rate"],
            "baseline_sat10": a_sat["success_rate"],
            "th90_sat10": threshold90([r["delta"] for r in a_rob],
                                      [r["sat10"]["success_rate"] for r in a_rob]),
            "th90_free": threshold90([r["delta"] for r in a_rob],
                                     [r["free"]["success_rate"] for r in a_rob]),
        },
        "cartpole": {
            "sv": cL["sv"].tolist(), "cond": cL["cond"], "zeros": cL["zeros"],
            "rank": cL["rank"], "normD": float(np.linalg.norm(cD)),
            "slowest_cl": slowest(c_cl),
            "rhp_zeros": [z for z in cL["zeros"]["u->x"] if z > 0],
            "baseline_unbounded": c_free["pole_rate"],
            "baseline_lim20": c_20["pole_rate"],
            "baseline_lim5": c_5["pole_rate"],
            "th90_free": threshold90([r["delta"] for r in c_rob],
                                     [r["free"]["pole_rate"] for r in c_rob]),
            "th90_lim20": threshold90([r["delta"] for r in c_rob],
                                      [r["lim20"]["pole_rate"] for r in c_rob]),
            "th90_lim5": threshold90([r["delta"] for r in c_rob],
                                     [r["lim5"]["pole_rate"] for r in c_rob]),
        },
    }
    R["comparison"] = comp
    print("\n=== comparison ===")
    print(json.dumps(comp, indent=1, default=enc))

    # --------------------------------------------------------------- figures
    # Fig 1: normalized singular values + zeros
    fig, axs = plt.subplots(1, 2, figsize=(9, 3.6))
    a_n = np.array(aL["sv"]) / aL["sv"][0]
    c_n = np.array(cL["sv"]) / cL["sv"][0]
    axs[0].semilogy(np.arange(1, 5), a_n, "o-", label="Acrobot")
    axs[0].semilogy(np.arange(1, 5), c_n, "s--", label="Cart-Pole")
    axs[0].set_xlabel("奇异值序号 i"); axs[0].set_ylabel(r"$\sigma_i / \sigma_1$")
    axs[0].set_xticks([1, 2, 3, 4]); axs[0].grid(alpha=0.3, which="both")
    axs[0].legend(); axs[0].set_title("归一化可控性奇异值")
    a_z = sorted(set([float(z) for z in aL["zeros"]["theta1"] + aL["zeros"]["theta2"]]))
    c_z = sorted(set([float(z) for z in cL["zeros"]["u->x"] + cL["zeros"]["u->θ"]]))
    axs[1].scatter(a_z, [1] * len(a_z), marker="o", color="tab:blue", label="Acrobot")
    axs[1].scatter(c_z, [0] * len(c_z), marker="s", color="tab:orange", label="Cart-Pole")
    axs[1].axvline(0, color="k", lw=0.8)
    axs[1].axvspan(0, 6, color="red", alpha=0.06)
    axs[1].text(4.6, 1.25, "RHP", color="tab:red", fontsize=8)
    axs[1].set_ylim(-0.5, 1.5); axs[1].set_yticks([0, 1]); axs[1].set_yticklabels(["Cart-Pole", "Acrobot"])
    axs[1].set_xlabel("零点实部 Re(z)"); axs[1].grid(alpha=0.3)
    axs[1].legend(fontsize=8); axs[1].set_title("系统零点（右半平面 = 非最小相位）")
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "fig1_linear.png")); plt.close(fig)

    # Fig 2: robustness
    fig, axs = plt.subplots(1, 2, figsize=(9, 3.6))
    ds = [r["delta"] for r in a_rob]
    axs[0].plot(ds, [r["free"]["success_rate"] for r in a_rob], "o-", label="无饱和")
    axs[0].plot(ds, [r["sat10"]["success_rate"] for r in a_rob], "s--", label="|τ| ≤ 10 N·m")
    axs[0].set_xlabel("扰动尺度 δ"); axs[0].set_ylabel("完整状态成功率（N=50）")
    axs[0].set_ylim(-0.02, 1.05); axs[0].grid(alpha=0.3); axs[0].legend(fontsize=8)
    axs[0].set_title("Acrobot")
    ds = [r["delta"] for r in c_rob]
    axs[1].plot(ds, [r["free"]["pole_rate"] for r in c_rob], "o-", label="无饱和")
    axs[1].plot(ds, [r["lim20"]["pole_rate"] for r in c_rob], "^-", label="|u| ≤ 20 N")
    axs[1].plot(ds, [r["lim5"]["pole_rate"] for r in c_rob], "s--", label="|u| ≤ 5 N")
    axs[1].set_xlabel("扰动尺度 δ"); axs[1].set_ylabel("摆杆平衡成功率（N=50）")
    axs[1].set_ylim(-0.02, 1.05); axs[1].grid(alpha=0.3); axs[1].legend(fontsize=8)
    axs[1].set_title("Cart-Pole")
    fig.suptitle("初始扰动鲁棒性：无约束 vs 执行器饱和")
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "fig2_robustness.png")); plt.close(fig)

    # Fig 3: weight trade-off
    fig, axs = plt.subplots(1, 2, figsize=(9, 3.6))
    axs[0].plot([r["R"] for r in a_R], [r["normD"] / a_R[1]["normD"] for r in a_R], "o-", label="Acrobot")
    axs[0].plot([r["R"] for r in c_R], [r["normD"] / c_R[2]["normD"] for r in c_R], "s--", label="Cart-Pole")
    axs[0].set_xscale("log"); axs[0].set_xlabel("R"); axs[0].set_ylabel("‖D(R)‖ / ‖D(R=1)‖")
    axs[0].grid(alpha=0.3, which="both"); axs[0].legend(); axs[0].set_title("相对反馈增益")
    axs[1].plot([r["R"] for r in a_R], [r["slowest"] for r in a_R], "o-", label="Acrobot")
    axs[1].plot([r["R"] for r in c_R], [r["slowest"] for r in c_R], "s--", label="Cart-Pole")
    axs[1].set_xscale("log"); axs[1].set_yscale("log")
    axs[1].set_xlabel("R"); axs[1].set_ylabel("最慢闭环极点 |λ| [rad/s]")
    axs[1].grid(alpha=0.3, which="both"); axs[1].legend(); axs[1].set_title("最慢闭环模态")
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "fig3_weights.png")); plt.close(fig)

    # Fig 4: physical parameter sensitivity
    fig, axs = plt.subplots(1, 2, figsize=(9, 3.6))
    axs[0].plot([r["value"] / 1.0 for r in a_m2], [r["sv"][-1] / a_m2[1]["sv"][-1] for r in a_m2],
                "o-", label="Acrobot: m2")
    axs[0].plot([r["value"] / 10.0 for r in c_mc], [r["sv"][-1] / c_mc[1]["sv"][-1] for r in c_mc],
                "s--", label="Cart-Pole: mc")
    axs[0].set_xscale("log")
    axs[0].set_xticks([0.2, 0.5, 1, 2, 4])
    axs[0].set_xticklabels(["0.2", "0.5", "1", "2", "4"])
    axs[0].xaxis.set_minor_formatter(NullFormatter())
    axs[0].set_xlabel("参数 / 名义值"); axs[0].set_ylabel(r"$\sigma_{\min}$ / 名义值")
    axs[0].grid(alpha=0.3, which="both"); axs[0].legend(fontsize=8)
    axs[0].set_title("最弱可控方向随物理参数变化")
    axs[1].plot([r["value"] / 1.0 for r in a_m2], [r["summary"]["success_rate"] for r in a_m2],
                "o-", label="Acrobot（饱和 ±10 N·m）")
    axs[1].plot([r["value"] / 10.0 for r in c_mc], [r["summary"]["pole_rate"] for r in c_mc],
                "s--", label="Cart-Pole（无饱和）")
    axs[1].set_xscale("log")
    axs[1].set_xticks([0.2, 0.5, 1, 2, 4])
    axs[1].set_xticklabels(["0.2", "0.5", "1", "2", "4"])
    axs[1].xaxis.set_minor_formatter(NullFormatter())
    axs[1].set_xlabel("参数 / 名义值"); axs[1].set_ylabel("成功率")
    axs[1].set_ylim(-0.02, 1.05); axs[1].grid(alpha=0.3, which="both"); axs[1].legend(fontsize=8)
    axs[1].set_title("参数变化时的成功率")
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "fig4_physical.png")); plt.close(fig)

    with open(os.path.join(HERE, "results.json"), "w") as f:
        json.dump(R, f, indent=1, default=enc)
    print("\nsaved:", os.path.join(HERE, "results.json"))
    print("figures in", FIG)


if __name__ == "__main__":
    main()
