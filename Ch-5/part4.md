---

### 5.8.3 Exercise 5.3：凸集图（GCS）落脚点规划

#### (a) 从 MIQP 到 GCS

MIQP 的 0-1 变量让最坏复杂度指数增长。GCS 的思路是：把每个"凸集"（石头）作为一个**顶点**，把"从集合 $u$ 到集合 $v$ 的可行转移"作为**边**，于是落脚点规划变成一个**凸集图上的最短路问题（SPP in GCS）**：

$$
\min_{\mathbf x,\boldsymbol\phi}\ \sum_{e\in E}c_e\phi_e
\quad\text{s.t.}\quad
\mathbf x_v\in\mathcal C_v,\ 
(\mathbf x_u,\mathbf x_v)\in\mathcal E_e,\ 
\boldsymbol\phi\ \text{满足流守恒}.
\tag{5.88}
$$

如果边约束和集合都是凸的，**凸松弛**（把 $\phi_e\in\{0,1\}$ 松弛为 $[0,1]$）就是一个凸优化问题，通常可以高效求解，而且松弛往往很紧（解出来接近 0/1）。

#### (b) 地形与凸集

与 MIQP 相同，每块石头是矩形（凸集）

$$
S_i=\{\mathbf x:\mathbf A_i\mathbf x\le\mathbf b_i\}.
\tag{5.89}
$$

此外还有两个**单点集**：

- `source`：初始位置 $\mathbf x_s$（初始石头的中心）；
- `target`：目标位置 $\mathbf x_t$（目标石头的中心）。

#### (c) 顶点：为什么要复制石头

GCS 的一个顶点对应**一个**连续变量 $\mathbf x_v$。如果机器人需要在同一块石头上连续放两次脚（例如很宽的 `lateral` 石头），就需要**两个**变量同时属于同一个集合；但 GCS 不允许两个变量共享同一个顶点。因此要为每块石头创建 $m$ 个**副本**：

$$
m=\left\lfloor\frac{\text{stone width}}{d}\right\rfloor+2 .
\tag{5.90}
$$

- $+2$ 中的第一个 $+1$：窄石头也要能放两只脚（至少 2 个位置）；
- 第二个 $+1$：宽石头需要多一个"过渡"副本。

命名约定：`xi_ii`，其中 `x` 是石头类型（`l` lateral、`s` start、`t` final、`b` bridge），`i` 是凸集索引，`ii` 是副本索引。

#### (d) 边：三类

1. **同一石头的副本之间**：固定顺序

$$
x_{i,0}\to x_{i,1}\to\cdots\to x_{i,m-1}.
\tag{5.91}
$$

不允许 $x_{i,0}\to x_{i,2}$ 这样的跳序，否则会引入冗余边。

2. **不同石头之间**：只连到目标石头的**父副本**（索引 0）：

$$
x_{j,jj}\to x_{i,0}\qquad(i\ne j).
\tag{5.92}
$$

这保证"进入一块石头"总是从它的第一个副本开始；离开时可以来自任意副本。

3. **source/target**：
   - `source` 只有出边，连到每块石头的父副本；
   - `target` 只有入边，来自每块石头的任意副本。

#### (e) 边约束与代价

**运动学约束**：相邻两步（移动的脚）在 $L_\infty$ 意义下不能超过 $d/2$：

$$
|x_u-x_v|\le\frac d2,\qquad
|y_u-y_v|\le\frac d2 .
\tag{5.93}
$$

在 Drake 中：

```python
for i in range(2):
    e.AddConstraint(xu[i] - xv[i] <= step_span / 2)
    e.AddConstraint(xv[i] - xu[i] <= step_span / 2)
```

**代价**：最小步数 ⇒ 每条边代价 1：

```python
e.AddCost(1.0)
```

#### (f) 凸松弛与流变量

对每条边 $e=(u,v)$ 引入流量 $\phi_e\in[0,1]$。流守恒：

$$
\sum_{e\in\delta^+(v)}\phi_e-\sum_{e\in\delta^-(v)}\phi_e
=
\begin{cases}
1,&v=\text{source},\\
-1,&v=\text{target},\\
0,&\text{otherwise}.
\end{cases}
\tag{5.94}
$$

当 $\phi_e\in\{0,1\}$ 时，这是一条从 source 到 target 的路径；松弛到 $[0,1]$ 后，它允许"分数条路径"。

**透视（perspective）形式**：边约束同时涉及 $\mathbf x_u,\mathbf x_v$ 和 $\phi_e$。直接写 $\phi_e(\mathbf x_u,\mathbf x_v)\in\mathcal E_e$ 是非凸的双线性。GCS 用**透视函数**把变量缩放：

$$
\mathbf y_{e,u}=\phi_e\mathbf x_u,\qquad
\mathbf y_{e,v}=\phi_e\mathbf x_v,
\tag{5.95}
$$

并在 $(\phi_e,\mathbf y_{e,u},\mathbf y_{e,v})$ 上写凸约束（集合的透视是凸的）。这就是 GCS 的凸松弛。

如果最优解中所有 $\phi_e\in\{0,1\}$，松弛是**精确**的，直接得到一条路径；否则需要**舍入**（rounding）：在图上搜索接近松弛解的整数路径。Drake 的 `GraphOfConvexSetsOptions` 提供 `max_rounded_paths` 控制舍入次数。

#### (g) 代码要点

```python
# 顶点和同集副本边
num_copy_lateral = int(np.floor(lateral.width / step_span)) + 2
num_copy_s_t = int(np.floor(start.width / step_span)) + 2
num_copy_bridge = int(np.floor(bridge_width / step_span)) + 2

# 不同集合之间只连到父副本
if v.name().endswith("_0") and stone_key(v) != stone_key(u):
    E_list.append([u, v])

# 边约束与代价
for i in range(2):
    e.AddConstraint(xu[i] - xv[i] <= step_span / 2)
    e.AddConstraint(xv[i] - xu[i] <= step_span / 2)
e.AddCost(1.0)
```

#### (h) 结果

在 `terrain_A = [1,1,1,1,1,1]` 和 `terrain_B = [1,1,1,0,1,1]` 上：

| 地形 | 图上的路径代价 | 自评分 |
|---|---|---|
| A（完整桥） | 10 条边 | 16/16 |
| B（缺一块） | 10 条边 | 16/16 |

自评分检查两项：

1. 两脚相对位置不超过 $d/2$（`test_relative_position_limits`，8 分）；
2. 总位移代价达到目标值（`test_minimize_step_length`，8 分）。

**一个值得注意的现象**：完整桥地形 A 上，GCS 的解也可能绕到上方的 `lateral` 石头。原因不是约束错误，而是**最优解不唯一**：所有边的代价都是 1，从桥直走和从 lateral 绕行的**步数相同**，SNOPT 选择了其中一个。MIQP 的二次目标（步长平方）会打破这种平局，所以 MIP 在 A 上直走。这说明：

- 常数边代价只编码"步数"，不编码"步长/能量"；
- 如果希望直走，需要加一个微小的长度代价或 tie-breaker；
- 凸松弛的舍入也可能在等价路径中任选一条。

#### (i) MIQP vs GCS

| 维度 | MIQP（Exercise 5.1） | GCS（Exercise 5.3） |
|---|---|---|
| 组合变量 | 0-1 指派 | 边流量 $\phi_e\in[0,1]$ |
| 求解 | 分支定界，最坏指数 | 凸松弛 + 舍入，通常多项式级 |
| 全局最优性 | 有（分支定界证明） | 松弛精确时有；否则舍入可能次优 |
| 代价 | 二次（步长平方） | 线性（步数） |
| 扩展性 | 石头/步数增多时迅速变难 | 更好，适合大规模地形 |
| 实现 | big-M，调 $M$ | 图构造、副本数、透视约束 |
| 适用 | 中小规模、需要精确最优 | 大规模、需要快速求解 |

> **理解提示：GCS 不是"免费的午餐"**。它把组合搜索藏进了凸松弛和舍入；如果松弛不紧（分数流量很多），舍入可能找不到全局最优。它的优势在于：大多数实际地形上，松弛非常紧，求解速度远快于分支定界。

> **自检 5.10**：
> 1. 为什么同一块石头需要多个副本？如果只用一个副本，会丢失哪类解？
> 2. 流守恒式 (5.94) 中 source 的净流出为什么是 1？
> 3. 常数边代价 1 与步长平方代价，对"绕路"和"直走"的选择有什么不同？

---

## 本章小结

| 主题 | 核心公式/结论 | 适用条件 | 失效场景 |
|---|---|---|---|
| ZMP/CoP | $\ddot x=\frac{g}{h}(x-x_{cp})$ | 平地、恒高、小角动量 | 大角动量、崎岖地形、点足 |
| LIPM | $\ddot x=\omega^2(x-p)$ | 恒质心高度 | 质心高度大幅变化 |
| DCM | $\xi=x+\dot x/\omega$，$\dot\xi=\omega(\xi-p)$ | LIPM | 非 LIPM 动力学 |
| 捕获点 | $p=\xi$ 可让质心停住 | 单步、恒定 $p$ | 步长受限、多步 |
| 质心动力学 | $\mathbf A\ddot{\mathbf q}+\dot{\mathbf A}\dot{\mathbf q}=\sum\mathbf F$ | 精确 | 无（但约束需完整） |
| 全身 QP | 式 (5.49) | 实时控制 | 非凸接触、强冲击 |
| MIQP 落脚点 | 式 (5.66) | 中小规模、全局最优 | 大规模（指数爆炸） |
| GCS 落脚点 | 式 (5.88) | 大规模、快速 | 松弛不紧、需要精确最优 |

**一句话总结**：第 5 章把"高自由度"拆成三层——**质心层用 ZMP/LIPM 做连续规划，接触层用 MIQP/GCS 做离散规划，关节层用质心动力学/全身 QP 做执行**。ZMP 是连接连续与离散的桥梁；捕获点是连接当前步与下一步的桥梁。

---

## 附录 A：数值验证脚本

完整脚本在：

- `exp5_analysis/example51_zmp.py`：Example 5.1 的复现、CoP 恒等式、跟踪误差、反馈、步时/权重敏感性；
- `exp5_analysis/exercise52_zmp.py`：Exercise 5.2 的三种 ZMP 计算、自由摆仿真、设计研究；
- `exp5_analysis/make_ch5_figures.py`：所有图；
- `examples/Exp5.1-zmp-planner.ipynb`、`examples/Exp5.2-littledog.ipynb`：已执行的示例 notebook；
- `exercises/Exp5.8.1-footstep-mip.ipynb`、`exercises/Exp5.8.3-footstep-gcs.ipynb`：已执行的习题 notebook（自评分 16/16）。

关键验证代码（可直接运行）：

```python
import numpy as np
from scipy.integrate import solve_ivp
from pydrake.all import PiecewisePolynomial, ZmpPlanner

# ------------------------------------------------------------
# 1. LIPM 闭式解 vs 数值积分
# ------------------------------------------------------------
g, h = 9.81, 1.0
w = np.sqrt(g / h)
p, x0, v0 = 0.3, 0.0, 0.4
T = 1.2
x_cf = lambda t: p + (x0 - p) * np.cosh(w * t) + v0 / w * np.sinh(w * t)
sol = solve_ivp(lambda t, y: [y[1], w**2 * (y[0] - p)], [0, T], [x0, v0],
                rtol=1e-12, atol=1e-14, dense_output=True)
err = max(abs(sol.sol(t)[0] - x_cf(t)) for t in np.linspace(0, T, 100))
print("LIPM closed-form error:", err)          # ~1e-14

# ------------------------------------------------------------
# 2. DCM 守恒（常数 ZMP）
# ------------------------------------------------------------
t = np.linspace(0, T, 100)
x = x_cf(t)
v = np.array([sol.sol(s)[1] for s in t])
xi = x + v / w
print("DCM spread:", xi.max() - xi.min())      # ~1e-14

# ------------------------------------------------------------
# 3. ZmpPlanner 的 CoP 恒等式
# ------------------------------------------------------------
footsteps = np.array([[0, -0.05], [0.5, 0.05], [1, -0.05], [1.5, 0.05]])
zmp_traj = PiecewisePolynomial.FirstOrderHold(
    [0, 1, 1.5, 2.5, 3, 4], np.array(
        [[0, 0.5, 0.5, 1.0, 1.0, 1.5],
         [-0.05, -0.05, 0.05, 0.05, -0.05, -0.05]]))
planner = ZmpPlanner()
planner.Plan(zmp_traj, np.zeros(4), h)
err = 0.0
for s in np.linspace(0, zmp_traj.end_time(), 100):
    c = planner.get_nominal_com(s)
    cd = planner.get_nominal_comd(s)
    cdd = planner.get_nominal_comdd(s)
    cop = planner.comdd_to_cop(np.concatenate((c, cd)), cdd)
    err = max(err, np.max(np.abs(cop - (c - h / g * cdd))))
print("CoP identity error:", err)              # 0

# ------------------------------------------------------------
# 4. Exercise 5.2：ZMP 三种算法
# ------------------------------------------------------------
h_f, L1, L2, l, m = 0.08, 0.12, 0.08, 0.5, 1.0
for th in np.linspace(0.5, 2.6, 7):
    # 几何：线 P-b 与地面交点
    xz_geo = L2 - h_f / np.tan(th)
    # 动力学：自由摆 + 一般 ZMP 公式
    thdd = -(9.81 / l) * np.cos(th)
    thd = 0.7
    xb = L2 + l * np.cos(th)
    zb = h_f + l * np.sin(th)
    ax = -l * np.sin(th) * thdd - l * np.cos(th) * thd**2
    ay = l * np.cos(th) * thdd - l * np.sin(th) * thd**2
    xz_dyn = xb - zb * ax / (ay + 9.81)
    assert abs(xz_geo - xz_dyn) < 1e-12
print("Exercise 5.2 ZMP checks passed.")

# ------------------------------------------------------------
# 5. 习题数值结果
# ------------------------------------------------------------
import json
with open("exp5_analysis/results_zmp.json") as f:
    zmp_res = json.load(f)
print("ZMP planner tracking error:", zmp_res["max_tracking_error_vs_desired"])
print("Timing study:", zmp_res["timing_sensitivity"])
with open("exercises/results.json") as f:
    print("Exercise autograder:", json.load(f)["score"])  # 16
```

**LittleDog 质检**：对保存的最优轨迹，用 Drake 的
`CalcSpatialMomentumInWorldAboutPoint` 计算每个节点的空间动量，并与接触力 + 重力的空间力做有限差分对拍。中位残差约 $2.2\times10^{-2}$；在接触切换节点上最大残差约 $10$，这是**速度级后向欧拉 + 接触冲量**造成的预期现象，不是建模错误。

---

## 附录 B：公式速查

| 内容 | 公式 |
|---|---|
| 刚体平动 | $m\ddot x=\sum f_x$，$m\ddot z=\sum f_z-mg$ |
| 刚体转动（$y$ 分量） | $I\ddot\theta=\sum(p_zf_x-p_xf_z)$ |
| 单边接触 | $f_z\ge0\Rightarrow\ddot z\ge-g$ |
| 摩擦 | $|f_x|\le\mu f_z\Rightarrow|\ddot x|\le\mu(\ddot z+g)$ |
| CoP | $x_{cp}=\frac{\sum x_if_{i,z}}{\sum f_{i,z}}$ |
| ZMP 方程 | $(m\ddot z+mg)(x_{cp}-x)=-zm\ddot x-I\ddot\theta$ |
| LIPM | $\ddot x=\omega^2(x-x_{cp})$，$\omega=\sqrt{g/h}$ |
| LIPM 闭式解 | $x=p+(x_0-p)\cosh\omega t+\frac{\dot x_0}{\omega}\sinh\omega t$ |
| DCM | $\xi=x+\dot x/\omega$，$\dot\xi=\omega(\xi-p)$ |
| 捕获点 | $p=\xi$ 时质心渐近停在 $p$ |
| 轨道能量 | $E=\frac12\dot x^2-\frac12\omega^2(x-p)^2$ |
| 质心 | $\mathbf p_{cm}=\frac{\sum m_i\mathbf p_i}{\sum m_i}$ |
| 质心动量 | $\mathbf L=\mathbf A(\mathbf q)\dot{\mathbf q}$ |
| 质心动力学 | $\mathbf A\ddot{\mathbf q}+\dot{\mathbf A}\dot{\mathbf q}=\sum\mathbf F$ |
| 质心雅可比 | $\mathbf J_{cm}=\frac{1}{M}\sum m_i\mathbf J_i$ |
| MIQP 相对约束 | $\|\mathbf x^L_t-\mathbf x^R_t\|_\infty\le d/2$ |
| big-M | $\mathbf A_i\mathbf x_t\le\mathbf b_i+(1-\delta_{t,i})\mathbf M$ |
| 每脚一石 | $\sum_i\delta_{t,i}=1$ |
| MIQP 目标 | $\sum_t\|\Delta\mathbf x^L_t\|^2+\|\Delta\mathbf x^R_t\|^2$ |
| GCS 流守恒 | $\sum_{e\in\delta^+(v)}\phi_e-\sum_{e\in\delta^-(v)}\phi_e=1_v^{src}-1_v^{tgt}$ |
| GCS 边约束 | $\|\mathbf x_u-\mathbf x_v\|_\infty\le d/2$ |
| 副本数 | $m=\lfloor w/d\rfloor+2$ |
| 自由腿动力学 | $\ddot\theta=-(g/l)\cos\theta$ |
| 无驱动踝 ZMP | $x_{zmp}=L_2-h\cot\theta$ |
| 有驱动踝 ZMP | $x_{zmp}=L_2+l\cos\theta$ |
| 脚尖极限 | $\theta_T=\pi-\arctan(h/L_1)$ |
| 脚跟极限 | $\theta_H=\arctan(h/L_2)$ |
| 有驱动范围 | $\theta\in[\arccos(L_1/l),\arccos(-L_2/l)]$ |
| 抬脚跟条件 | $\cos\theta_{lift}=L_1/l$ |

---

## 附录 C：本章最容易错的 12 个点

1. **把 CoP 当成质心投影**。只有静态/零加速度时才相等；动态行走时 ZMP 必须超前/滞后。
2. **把 ZMP 当成稳定性证明**。ZMP 在支撑多边形内只是"当前接触可行"，不保证速度收敛或下一步有脚可踩。
3. **叉乘符号搞错**。本章用 $y=z\times x$ 约定，$[\mathbf p\times\mathbf f]_y=p_zf_x-p_xf_z$；与常见二维标量叉乘差一个符号。
4. **忽略 $I\ddot\theta$**。上身快速转动、摆臂、大角动量时，ZMP 方程 (5.8) 中的角动量项不能丢。
5. **认为 ZMP 可以任意放在支撑多边形外**。ZMP 是接触力的结果；跑到外面意味着机器人绕脚边缘翻倒，平脚模型失效。
6. **把 LIPM 的 $\omega$ 写成 $\sqrt{h/g}$**。正确是 $\omega=\sqrt{g/h}$；腿越高，倒得越慢。
7. **DCM 与质心位置混淆**。$\xi=x+\dot x/\omega$；捕获点是"脚该踩哪"，不是质心现在在哪。
8. **认为 ZMP 规划能处理所有接触**。点足、脚尖/脚跟滚动、非平地接触需要更一般的模型；ZMP 规划只适用于平脚/面接触。
9. **big-M 取太大**。分支定界的 LP 松弛变差，求解时间爆炸；应取"刚好覆盖地形"的最小值。
10. **MIQP 目标用步数而不是步长平方**。步数是整数、非凸，不能直接放进二次目标；步长平方是凸的代理。
11. **GCS 的副本数取错**。$m=\lfloor w/d\rfloor+2$；少一个副本会导致窄石头无法放两只脚，或宽石头无法多步通过。
12. **Exercise 5.2 忽略无质量腿的二力杆性质**。腿力沿 $P$–$b$ 连线，ZMP 是这条线与地面的交点；如果按"质心投影"算，会得到完全错误的 $L_2+l\cos\theta$（那是**有驱动踝**的答案，不是无驱动踝的答案）。

> **两个附加陷阱**：
> - 有驱动踝与无驱动踝的 ZMP 公式**不同**：$L_2+l\cos\theta$ vs $L_2-h\cot\theta$；答题时先确认踝是否驱动。
> - "最大化姿态范围"的踝位置严格解是**脚的一端**，不是中间；中间是**最小**范围。若题目期望"对称裕度"，需要说明优化指标。

---

## 附录 D：与 Ch. 3 / Ch. 4 的衔接

| Ch. 3（Acrobot/Cart-Pole/Quadrotor） | Ch. 4（简单行走/奔跑） | Ch. 5（高自由度足式） |
|---|---|---|
| 固定点稳定性 | 周期轨/极限环 | 质心轨迹 + 支撑多边形 |
| 线性化 $A,B$ | 返回映射 $DP$ | 质心动量矩阵 $\mathbf A(\mathbf q)$ |
| LQR | 离散 LQR/deadbeat | ZMP 跟踪 + WBC QP |
| 欠驱动 = 输入少 | 欠驱动 = 单边接触 | 欠驱动 = 接触切换/点足 |
| 连续控制 | 混合动力学/碰撞 | 分层：离散落脚 + 连续质心 |
| 能量整形 | 能量平衡 | 质心/角动量守恒 |

**一句话总结**：第 3 章稳定一个**点**；第 4 章稳定一条**轨道**；第 5 章规划一个**接触序列 + 质心轨迹**，再用全身控制执行。工具从"连续线性化"→"庞加莱映射"→"ZMP/LIPM + 混合整数/凸优化"，但核心思想始终是：**在低维抽象上做可解的优化，在高维模型上做反馈修正**。
