# 第 4 章 行走与奔跑的简单模型 —— 深度推导笔记

> **来源**：Russ Tedrake, *Underactuated Robotics*, Ch. 4 "Simple Models of Walking and Running"（underactuated.csail.mit.edu/simple_legs.html，2024 版）。
> 公式编号 `(n)` 为本笔记自编，按出现顺序排列；`[笔记]` 标记的是原书省略、本笔记补全的推导。
> 所有数值结果均由附录 A 的脚本验证。

---

## 0. 本章地图与符号

### 0.1 本章要解决的问题

前几章的系统（Acrobot、Cart-Pole、四旋翼）虽然有欠驱动，但**始终没有接触**。本章第一次引入：

1. **接触（contact）**：接触力只在两物体接触时非零。什么时候落脚、哪只脚支撑、是否打滑，都是离散事件，给连续动力学叠加了一层组合结构。
2. **周期轨（periodic orbit）**：走路不是稳定到一个固定点，而是稳定到一条**周期轨道**——稳定的步态是一个极限环。
3. **混合动力学（hybrid dynamics）**：连续相（支撑相、飞行相）+ 瞬时离散事件（脚跟着地、膝盖锁定、起飞）共同构成一个 hybrid system。

行走的欠驱动性还有一层特殊含义：脚不能"抓住"地面，因此**无论执行器多强，都无法让质心比重力更快地压向地面**。这是行走区别于一般机械臂的根本约束。

### 0.2 统一符号

| 符号 | 含义 |
|---|---|
| $\theta$ | 关节/腿的绝对角（各节会重新定义零位与正方向） |
| $\alpha$ | 无轮缘轮：相邻辐条夹角的一半；罗盘步态：腿部几何参数 |
| $\gamma$ | 斜坡角（斜面与水平面的夹角） |
| $l$ | 腿长/轮半径 |
| $m,\ m_h$ | 集中质量、髋部质量 |
| $a,\ b$ | 质心到髋/脚的距离 |
| $\omega=\dot\theta$ | 角速度 |
| $\mathbf{x}=[\mathbf{q}^\top,\dot{\mathbf{q}}^\top]^\top$ | 连续状态 |
| $S$ | 庞加莱截面（surface of section） |
| $\mathbf{P}(\cdot)$ | 庞加莱返回映射（return map） |
| $\lambda_i$ | 返回映射线性化的特征值（Floquet 乘子） |
| $E=T+U$ | 总机械能 |
| $k,\ l_0$ | SLIP 弹簧刚度、原长 |

### 0.3 全章技术路线

```text
接触 + 欠驱动 + 周期运动
            │
            ▼
   极限环 / 轨道稳定性（§4.1）
            │
            ▼
   庞加莱截面：连续流 → 离散映射（§4.2）
            │
            ▼
   返回映射的固定点 ↔ 周期步态
            │
            ▼
   在固定点线性化：|λ|<1 ⇒ 步态稳定
            │
   ┌────────┴────────┐
   ▼                 ▼
行走模型            奔跑模型
无轮缘轮 → 罗盘步态  SLIP → apex-to-apex map
→ 膝盖步行者         → 无差拍控制 / 盲跑
   └────────┬────────┘
            ▼
   同一套工具：杂耍（§4.10）
```

### 0.4 五个模型一句话对照

| 模型 | 自由度/状态 | 能量来源与耗散 | 分析工具 | 核心结论 |
|---|---|---|---|---|
| Van der Pol | 2 | 非线性阻尼注入/耗散 | 1D 返回映射 | 稳定极限环 |
| 无轮缘轮 | 2 | 重力输入 vs 碰撞耗散 | 1D 步到步映射 | 滚动不动点 $\omega^*_{roll}$，导数 $\cos^2 2\alpha<1$ |
| 罗盘步态 | 4 | 重力输入 vs 脚跟着地耗散 | 高维返回映射 | 稳定极限环，但吸引域极窄 |
| SLIP | 4（分段） | 无耗散，能量守恒 | 1D apex map | 部分稳定；可一步无差拍控制 |
| 直线杂耍 | 2（分段） | 弹性碰撞 + 能量反馈 | 1D 碰撞前映射 | 镜像律使映射可解、可稳 |

> **一条主线**：本章所有漂亮结果都来自"**把高维连续系统压到低维离散映射**"。压不下去的时候（例如 SLIP 的能量扰动方向），结论就只能是部分的。

---

## 4.1 极限环与轨道稳定性

### 4.1.1 从固定点到周期轨

第 3 章的稳定性都是针对**固定点** $x^*$ 的：

$$
\|x(t)-x^*\|\to 0\quad(t\to\infty).
$$

走路的目标却不是停在一个点上，而是**周而复始地走**。设一条周期解为

$$
x^*(t+T)=x^*(t),\qquad \forall t,
$$

其中 $T$ 是步态周期。最朴素的想法是照搬固定点定义：

$$
\lim_{t\to\infty}\|x(t)-x^*(t)\|=0. \tag{1}
$$

**这个定义太强了，而且经常根本不成立。** 原因很简单：两条都收敛到同一周期轨的轨迹，可能始终存在一个**相位差**。就像两个人沿着同一条圆形跑道跑，速度都趋于一致，但一个总落后半步——逐时刻比较永远对不上，可他们明明在同一条轨道上。

Van der Pol 振子的时间响应图就是这样：不同初值的轨迹最终振幅相同、形状相同，但峰值时刻并不重合。因此我们要比较的不是"同一时刻的状态"，而是"到整条轨道的最近距离"。

### 4.1.2 Van der Pol 振子

Van der Pol 方程

$$
\ddot q+\mu(q^2-1)\dot q+q=0,\qquad \mu>0. \tag{2}
$$

写成状态方程（$x_1=q,\ x_2=\dot q$）：

$$
\dot x_1=x_2,\qquad
\dot x_2=-x_1-\mu(x_1^2-1)x_2. \tag{3}
$$

它像一个弹簧–质量–阻尼系统，但**阻尼系数不是常数**：

$$
c(x_1)=\mu(x_1^2-1).
$$

- $|x_1|>1$：$c>0$，阻尼耗散能量，振幅被压小；
- $|x_1|<1$：$c<0$，负阻尼注入能量，振幅被放大。

两种趋势在 $|x_1|\approx1$ 附近达到平衡，于是所有非零初值都被"夹"到同一条闭轨上。

**能量视角（原书未展开）**：取

$$
E=\frac12\dot q^2+\frac12 q^2.
$$

沿 (2) 求导：

$$
\dot E=q\dot q+\dot q\ddot q
=\dot q\big(\ddot q+q\big)
=\dot q\big[-\mu(q^2-1)\dot q\big]
=\mu(1-q^2)\dot q^2. \tag{4}
$$

所以：

- $|q|<1$：$\dot E>0$，能量增加；
- $|q|>1$：$\dot E<0$，能量减少；
- $|q|=1$：$\dot E=0$。

$\dot E$ 的符号完全由 $1-q^2$ 决定，这正是"负阻尼区注入、正阻尼区耗散"的严格表述。

**原点是怎样的平衡点？** 在 $x=0$ 线性化 (3)：

$$
A=\begin{bmatrix}0&1\\-1&\mu\end{bmatrix},
\qquad
\lambda^2-\mu\lambda+1=0.
$$

当 $0<\mu<2$ 时，

$$
\lambda_{1,2}=\frac{\mu}{2}\pm i\sqrt{1-\frac{\mu^2}{4}},
$$

实部 $\mu/2>0$，原点是**不稳定焦点**。轨迹从原点附近被"推出去"，最后落到外面的稳定极限环上。这解释了为什么极限环内部还有一个不稳定固定点：稳定与不稳定在此共存。

### 4.1.3 轨道稳定性的严格定义

设 $\Gamma=\{x^*(t):t\in[0,T)\}$ 是周期轨。定义轨迹到轨道的距离

$$
d\big(x(t),\Gamma\big)=\min_{\tau}\|x(t)-x^*(\tau)\|. \tag{5}
$$

**轨道稳定（orbital stability）**：对任意 $\epsilon>0$，存在 $\delta>0$，使得

$$
d\big(x(t_0),\Gamma\big)<\delta
\ \Longrightarrow\
d\big(x(t),\Gamma\big)<\epsilon,\qquad \forall t\ge t_0. \tag{6}
$$

若进一步

$$
d\big(x(t),\Gamma\big)\to0\quad(t\to\infty), \tag{7}
$$

则称**渐近轨道稳定**；若指数收敛，则称**指数轨道稳定**。

注意：这里允许轨迹沿轨道方向"滑动"，只禁止它横向离开轨道。这正是走路需要的稳定性——你走快一点或慢一点没关系，别摔倒就行。

### 4.1.4 二维系统的两个经典结论

**(1) Poincaré–Bendixson 定理**：若平面系统存在一个**不含固定点**的非空紧不变集，则该集合必包含一条闭轨。

这给二维系统提供了一个"找极限环"的存在性工具：先找一个被轨迹困住、又不可能停下来的区域，闭轨就藏在里面。

**(2) 梯度系统没有闭轨**：若系统可写成

$$
\dot x=-\nabla V(x),
$$

则

$$
\frac{d}{dt}V(x(t))=-\|\nabla V\|^2\le0,
$$

$V$ 单调不增，轨迹不可能返回起点形成闭轨（除非停在固定点）。同理，若 Lyapunov 函数满足 $\dot V\prec0$（严格负定），也不可能有闭轨。

> **后果**：第 3 章那套"找一个 $\dot V\prec0$ 的函数"的 Lyapunov 方法，不能直接用来证明极限环稳定。后面章节会把 Lyapunov 思想改造成"轨道 Lyapunov 函数 / 横向坐标"的形式。
>
> **自检**：在 (4) 中，若把 $\mu$ 换成 0，$\dot E=0$，系统变成保守谐振子，所有轨道都是闭轨——这正说明"闭轨"并不自动等于"稳定极限环"；稳定性还需要周边轨迹向它收敛。

---

## 4.2 庞加莱映射（Poincaré Maps）

### 4.2.1 截面、横截性与首次返回

考虑 $n$ 维自治系统

$$
\dot{\mathbf{x}}=\mathbf{f}(\mathbf{x}),\qquad \mathbf{x}\in\mathbb{R}^n.
$$

取一个 $n-1$ 维曲面 $S$，称为**庞加莱截面（surface of section）**。要求：

1. **横截（transverse）**：对所有 $x\in S$，$\mathbf{f}(x)\notin T_xS$，即流不是沿着 $S$ 滑动，而是"穿过去"；
2. **返回性**：从 $S$ 出发的轨迹经过有限时间后会再次穿过 $S$。

定义**首次返回时间**

$$
\tau(x)=\min\{t>0:\ \phi_t(x)\in S\},
$$

以及**庞加莱映射（return map）**

$$
\mathbf{x}_p[n+1]=\mathbf{P}(\mathbf{x}_p[n]),\qquad
\mathbf{P}(x)=\phi_{\tau(x)}(x). \tag{8}
$$

其中 $\phi_t$ 是连续系统的流。记 $\mathbf{x}_p[n]=\mathbf{x}(t_n)$，$t_n$ 是第 $n$ 次穿越 $S$ 的时刻。

> **技术注记**：教材允许把原连续系统的固定点也当作映射的固定点，并令返回时间为 0。固定点不满足横截条件（$\mathbf{f}=0$），数值上要小心处理，但分析上无害。

### 4.2.2 周期轨 ↔ 映射的固定点

这是全章的核心等价关系：

$$
\mathbf{x}_p^*=\mathbf{P}(\mathbf{x}_p^*)
\quad\Longleftrightarrow\quad
\text{存在唯一一条经过 }\mathbf{x}_p^*\text{ 的周期轨}. \tag{9}
$$

**必要性**：若 $\mathbf{P}(x^*)=x^*$，则从 $x^*$ 出发，经过 $\tau(x^*)>0$ 后回到 $x^*$，轨迹闭合，形成周期轨。

**充分性**：若 $\Gamma$ 是周期轨且 $x^*\in S\cap\Gamma$，则从 $x^*$ 出发的轨迹下次回到 $S$ 时，必然仍在同一条轨道上；由横截性与流的唯一性，它回到的正是 $x^*$（否则会与轨道相交两次，破坏唯一性）。

**稳定性等价**：若 $x^*$ 是 $\mathbf{P}$ 的稳定（渐近稳定、指数稳定）固定点，则对应的周期轨 $\Gamma$ 是稳定（渐近稳定、指数稳定）极限环。若返回时间 $\tau$ 有上界，还可以把离散映射的收敛性提升为连续时间的轨道稳定性。

> **技术注记**：上面的等价关系默认"周期轨每个周期只穿过 $S$ 一次"。若一条周期轨在一个周期内多次穿过 $S$，则单次返回映射的固定点只对应其中一段；完整周期要用 $\mathbf{P}^k$ 的固定点来描述。本章的行走/奔跑模型都取每个步态周期穿越截面一次，因此可以直接用 $\mathbf{P}$。

这个等价关系把"连续系统的极限环问题"降成了"离散映射的固定点问题"，维数从 $n$ 降到 $n-1$。

### 4.2.3 线性化：特征值与 Floquet 乘子

在固定点 $x_p^*$ 处线性化 (8)：

$$
\delta\mathbf{x}_p[n+1]
=\mathbf{A}_p\,\delta\mathbf{x}_p[n],
\qquad
\mathbf{A}_p=\left.\frac{\partial\mathbf{P}}{\partial\mathbf{x}}\right|_{\mathbf{x}_p^*}.
\tag{10}
$$

**判据**：若 $\mathbf{A}_p$ 的所有特征值满足

$$
|\lambda_i|<1,\qquad i=1,\dots,n-1, \tag{11}
$$

则周期轨**局部指数轨道稳定**；若有某个 $|\lambda_i|>1$，则不稳定；若有特征值落在单位圆上，线性化无法判定（临界情形）。

**为什么是 $n-1$ 个特征值？** 因为 $S$ 是 $n-1$ 维的，$\mathbf{P}$ 是 $S\to S$ 的映射，$\mathbf{A}_p$ 的维度是 $(n-1)\times(n-1)$。

**一个容易混淆的点**：如果在**原始 $n$ 维坐标**里做特征值分析，会多出一个**零特征值**。它对应"沿周期轨方向的扰动"——这种扰动只改变相位，不改变首次返回的状态。零特征值不影响稳定性判断，但必须排除。作为对照，**单值矩阵（monodromy matrix）** $\partial\phi_T/\partial x$ 沿周期轨方向的特征值是 **1**（不是 0），因为它描述的是"经过一个完整周期后"的状态，而相位扰动仍然在轨道上。两者不要混。

### 4.2.4 一维映射与阶梯图

当 $n=2$ 时，$S$ 是一维的，返回映射是一维映射

$$
x_p[n+1]=f(x_p[n]). \tag{12}
$$

一维映射可以用**阶梯图（staircase diagram）** 直观分析：

1. 画出 $y=f(x)$ 和 $y=x$；
2. 交点就是固定点；
3. 从初值 $(x_0,x_0)$ 出发，竖直移动到 $(x_0,f(x_0))$；
4. 水平移动到 $(f(x_0),f(x_0))$；
5. 重复。

**稳定性判据**：在固定点处

$$
|f'(x^*)|<1\ \Longrightarrow\ \text{局部指数稳定},\qquad
|f'(x^*)|>1\ \Longrightarrow\ \text{不稳定}. \tag{13}
$$

若 $0<f'(x^*)<1$，单调收敛；若 $-1<f'(x^*)<0$，振荡收敛（每一步在固定点两侧跳）。

### 4.2.5 Van der Pol 的返回映射

取截面

$$
S=\{(q,\dot q):q=0,\ \dot q\ge0\}. \tag{14}
$$

这是二维相平面上的一条半直线，流横截穿过它。定义

$$
\dot q_p[n+1]=P\big(\dot q_p[n]\big). \tag{15}
$$

**数值结果**（$\mu=0.2$，附录 A 脚本）：

| $\dot q_p[n]$ | 0.3 | 0.5 | 0.8 | 1.0 | 1.5 | 2.0 | 3.5 |
|---|---|---|---|---|---|---|---|
| $\dot q_p[n+1]$ | 0.549 | 0.873 | 1.269 | 1.472 | 1.814 | 2.005 | 2.223 |

- 固定点 $\dot q_p^*\approx2.007$，对应稳定极限环；
- 在固定点处的斜率 $P'(\dot q_p^*)\approx0.283<1$，所以极限环局部指数稳定；
- 原点 $\dot q_p=0$ 是固定点（返回时间为 0），但它是**不稳定**的：从 $0^+$ 出发会被推到外侧。

**为什么原点是固定的？** 因为 $(0,0)$ 是原系统的平衡点，轨迹停在那里，永远不会再"返回"截面。按照前面的约定，把返回时间定义为 0，它就成为一个固定点。阶梯图上，$P$ 在原点附近位于 $y=x$ 上方（$P(0.3)=0.549>0.3$），所以原点是不稳定固定点，这与 §4.1.2 的线性化结论一致。

> **自检**：这张表满足 $P(2.005)\approx2.005$，且 $P$ 在固定点附近的斜率约为 $0.28$，与"极限环稳定"一致。若把 $\mu$ 调大，极限环形状会偏离正弦，但"固定点 + $|P'|<1$"的结构不变。

---

## 4.3 行走简单模型总览

### 4.3.1 被动动态行走

**被动动态行走器（passive-dynamic walker）** 没有电机、没有控制器，只靠重力沿一个小斜坡走下去。Steve Collins 与 Andy Ruina 在 2001 年做出的三维被动行走机器人是这一思想的代表作：它由重力驱动，却能走出一条**稳定的极限环步态**。

这件事初看很反直觉：没有控制器，为什么不会摔倒？

答案是**力学本身提供了稳定性**：

- 每一步脚跟着地时的**非弹性碰撞**耗散能量；
- 下坡时**重力**补充能量；
- 两者在某个步态上达到平衡，形成稳定极限环；
- 偏离平衡时，碰撞损失与重力输入自动改变，把状态拉回来。

这正是 §4.1 中 Van der Pol 振子的翻版：能量注入 + 能量耗散的平衡产生稳定周期轨。后面的无轮缘轮会把这一点算得清清楚楚。

### 4.3.2 McMahon 的弹道行走假说

在被动行走器出现之前，McMahon（1980）从生物力学角度提出了一个更早的模型。他观察到：

- 支撑腿的肌肉活动（EMG）相对较高；
- **摆动腿的肌肉活动很低**，只在摆动开始和结束附近明显。

于是他提出**弹道行走（ballistic walking）** 模型：矢状面内的三连杆摆——

1. 踝到髋的"支撑腿"；
2. 髋到膝的"大腿"；
3. 膝到摆动脚的"小腿"。

摆动腿近似做自由摆，支撑腿像一个倒立摆。这个模型能大致复现步行运动学，但有明显局限：

- 忽略了支撑腿的**柔顺性（compliance）**；
- 只描述了连续摆动相，没有描述"迈步"这个碰撞事件。

尽管粗糙，它开启了"用低维摆模型理解步态"的传统，直接启发了 McGeer 的被动动态行走器。

### 4.3.3 从简单到复杂的建模阶梯

| 模型 | 增加了什么 | 去掉了什么假设 | 主要用途 |
|---|---|---|---|
| 无轮缘轮 | 支撑相 + 碰撞 | 摆动腿动力学 | 理解"重力 vs 碰撞"的能量平衡 |
| 罗盘步态 | 两条腿、髋关节、摆动相 | 膝盖 | 研究摆动腿与步态极限环 |
| 膝盖步行者 | 膝关节、knee-strike | 曲足、驱动 | 更接近人类步态、抬脚高度 |
| SLIP | 弹簧腿、飞行相 | 多关节、碰撞耗散 | 理解奔跑、跳跃、跨尺度相似性 |

> **建模哲学**：每一步只增加一个必要的物理机制，直到模型能解释目标现象为止。简单模型的价值不在于"像"，而在于**能算、能解释、能给出控制洞见**。

---

## 4.4 无轮缘轮（The Rimless Wheel）

### 4.4.1 模型、坐标与假设

无轮缘轮是一个刚性"轮子"，只有辐条、没有轮缘，髋部（轮心）有一个集中质量 $m$。相邻辐条的夹角为 $2\alpha$，轮半径为 $l$。轮子放在倾角为 $\gamma$ 的斜坡上。

**三条核心建模假设**：

1. **碰撞完全非弹性且瞬时**：碰撞前后只守恒绕碰撞点的角动量，其余能量损失；
2. **支撑脚是理想铰链**：不打滑、不脱离；
3. **支撑切换瞬时完成**：没有双支撑相。

**坐标约定**：$\theta$ 是支撑腿（从轮心指向触地点）与竖直方向的夹角，顺时针为正，$\theta=0$ 对应支撑腿竖直向上（质心在触地点正上方），这是单摆的**不稳定平衡**。一次支撑相内

$$
\theta\in[\gamma-\alpha,\ \gamma+\alpha]. \tag{16}
$$

支撑开始时 $\theta(0^+)=\gamma-\alpha$；下一根辐条触地时 $\theta=\gamma+\alpha$。

> **为什么是 $\gamma\pm\alpha$？** 斜坡法线相对竖直方向倾斜 $\gamma$。轮子是多边形，触地时支撑辐条相对斜坡法线的夹角为 $\pm\alpha$，所以相对竖直方向的夹角就是 $\gamma\pm\alpha$。

### 4.4.2 支撑相动力学：完整拉格朗日推导

以当前触地点为原点，质心高度

$$
h=l\cos\theta. \tag{17}
$$

动能与势能：

$$
T=\frac12ml^2\dot\theta^2,\qquad
U=mgl\cos\theta. \tag{18}
$$

拉格朗日量 $L=T-U$。对 $\theta$ 的欧拉–拉格朗日方程：

$$
\frac{d}{dt}\left(\frac{\partial L}{\partial\dot\theta}\right)-\frac{\partial L}{\partial\theta}=0. \tag{19}
$$

逐项计算：

$$
\frac{\partial L}{\partial\dot\theta}=ml^2\dot\theta,
\qquad
\frac{d}{dt}\frac{\partial L}{\partial\dot\theta}=ml^2\ddot\theta, \tag{20}
$$

$$
\frac{\partial L}{\partial\theta}
=-\frac{\partial U}{\partial\theta}
=mgl\sin\theta. \tag{21}
$$

代入 (19)：

$$
ml^2\ddot\theta-mgl\sin\theta=0
\quad\Longrightarrow\quad
\boxed{\ddot\theta=\frac{g}{l}\sin\theta}. \tag{22}
$$

这正是**无阻尼单摆**的方程，只是相图围绕的是单摆的不稳定平衡点 $\theta=0$。总能量

$$
E=\frac12ml^2\dot\theta^2+mgl\cos\theta \tag{23}
$$

满足

$$
\dot E=ml^2\dot\theta\ddot\theta-mgl\sin\theta\,\dot\theta
=\dot\theta\big(ml^2\ddot\theta-mgl\sin\theta\big)=0. \tag{24}
$$

所以在两次碰撞之间，能量严格守恒；能量只在碰撞瞬间损失。

### 4.4.3 跨步阈值 $\omega_1$

设支撑切换刚完成时

$$
\theta(0^+)=\gamma-\alpha,\qquad \dot\theta(0^+)=\omega_0.
$$

要迈出一步，质心必须从 $\theta=\gamma-\alpha$ 翻过最高点 $\theta=0$。由能量守恒：

$$
\frac12ml^2\omega_0^2+mgl\cos(\gamma-\alpha)
\ \ge\ \frac12ml^2\dot\theta^2\big|_{\theta=0}+mgl. \tag{25}
$$

在临界情形，到达 $\theta=0$ 时速度恰好为零。于是最小初速度

$$
\boxed{
\omega_1=\sqrt{2\frac{g}{l}\left[1-\cos(\gamma-\alpha)\right]}
}. \tag{26}
$$

讨论：

- $\gamma=\alpha$：$\omega_1=0$，质心在切换瞬间恰好位于触地点正上方，不需要额外动能即可"倒"过去；
- $\gamma>\alpha$：质心在切换瞬间已经位于触地点前方（下坡侧），**站立平衡根本不存在**，$\omega_1$ 的"翻越势垒"图像失效，可视为 $\omega_1=0$；
- $\gamma<\alpha$：质心在触地点后方（上坡侧），必须靠动能翻过最高点才能迈步。

### 4.4.4 触地前速度

从 $\theta=\gamma-\alpha$ 到 $\theta=\gamma+\alpha$，重力做正功。能量守恒：

$$
\frac12ml^2(\dot\theta^-)^2+mgl\cos(\gamma+\alpha)
=
\frac12ml^2\omega_0^2+mgl\cos(\gamma-\alpha). \tag{27}
$$

移项：

$$
(\dot\theta^-)^2
=\omega_0^2+2\frac{g}{l}\big[\cos(\gamma-\alpha)-\cos(\gamma+\alpha)\big]. \tag{28}
$$

利用余弦差公式

$$
\cos(\gamma-\alpha)-\cos(\gamma+\alpha)
=2\sin\alpha\sin\gamma, \tag{29}
$$

得到

$$
\boxed{
(\dot\theta^-)^2=\omega_0^2+4\frac{g}{l}\sin\alpha\sin\gamma
}. \tag{30}
$$

其中 $t^-$ 表示碰撞前瞬间。

> **能量解释**：一次支撑相中，重力使质心下降
> $$
> \Delta h=l\big[\cos(\gamma-\alpha)-\cos(\gamma+\alpha)\big]
> =2l\sin\alpha\sin\gamma,
> $$
> 对应重力做功 $2mgl\sin\alpha\sin\gamma$。这正好等于 (30) 中动能增量 $\frac12ml^2\cdot4(g/l)\sin\alpha\sin\gamma$。

### 4.4.5 足端碰撞：角动量守恒的几何推导

碰撞瞬间，旧支撑腿（角 $\gamma+\alpha$）离地，新辐条（角 $\gamma-\alpha$）触地。两根辐条的夹角为 $2\alpha$。

碰撞前，轮心速度

$$
\mathbf{v}^-=l\dot\theta^-\,\mathbf{e}_\perp^{\text{old}}, \tag{31}
$$

方向垂直于旧支撑腿，大小为 $l\dot\theta^-$。

碰撞后，轮子绕**新触地点**转动。新触地点到轮心的力臂为 $l$，但轮心速度在新腿法向的投影需要乘 $\cos(2\alpha)$：

$$
v_\perp^{\text{new}}=l\dot\theta^-\cos(2\alpha). \tag{32}
$$

对新触地点取角动量。碰撞前：

$$
L^-=-ml^2\dot\theta^-\cos(2\alpha). \tag{33}
$$

碰撞后：

$$
L^+=-ml^2\dot\theta^+. \tag{34}
$$

碰撞冲量作用在新触地点，因此**绕该点的外力矩为零**，角动量守恒：

$$
L^-=L^+.
$$

于是

$$
\boxed{\dot\theta^+=\dot\theta^-\cos(2\alpha)}. \tag{35}
$$

**能量损失**：碰撞前后动能之比为

$$
\frac{\frac12ml^2(\dot\theta^+)^2}{\frac12ml^2(\dot\theta^-)^2}
=\cos^2(2\alpha). \tag{36}
$$

所以每次碰撞损失的比例为

$$
1-\cos^2(2\alpha)=\sin^2(2\alpha). \tag{37}
$$

> **数值感受**：当 $\alpha=\pi/8$ 时，$2\alpha=\pi/4$，$\cos^2(2\alpha)=1/2$——**每次脚跟着地损失一半动能**。这就是为什么下坡是必要的：重力必须在每步补回这 50%。

### 4.4.6 步到步返回映射

把 (30) 与 (35) 复合，得到向前迈步的返回映射：

$$
\boxed{
\dot\theta^+_{n+1}
=\cos(2\alpha)\sqrt{(\dot\theta_n^+)^2+4\frac{g}{l}\sin\alpha\sin\gamma}
}. \tag{38}
$$

**推导**：设第 $n$ 步开始时的角速度为 $\dot\theta_n^+$。按 (30)，触地前

$$
(\dot\theta^-)^2=(\dot\theta_n^+)^2+4\frac{g}{l}\sin\alpha\sin\gamma.
$$

按 (35)，碰撞后

$$
\dot\theta_{n+1}^+=\dot\theta^-\cos(2\alpha),
$$

开方即得 (38)。注意这里取了正根，对应继续向前走。

#### 其余分支

如果速度不够翻过最高点，轮子会先向前摆、停下、再摆回来，然后与旧辐条发生碰撞。定义反向迈步阈值

$$
\omega_2=-\sqrt{2\frac{g}{l}\left[1-\cos(\alpha+\gamma)\right]}. \tag{39}
$$

对 $\gamma\le\alpha$，完整映射为

$$
\dot\theta_{n+1}^+=
\begin{cases}
\cos(2\alpha)\sqrt{(\dot\theta_n^+)^2+4\frac{g}{l}\sin\alpha\sin\gamma},
& \omega_1<\dot\theta_n^+,\\[8pt]
-\dot\theta_n^+\cos(2\alpha),
& \omega_2<\dot\theta_n^+<\omega_1,\\[8pt]
-\cos(2\alpha)\sqrt{(\dot\theta_n^+)^2-4\frac{g}{l}\sin\alpha\sin\gamma},
& \dot\theta_n^+<\omega_2.
\end{cases} \tag{40}
$$

三个分支的物理含义：

1. **$\dot\theta_n^+>\omega_1$**：动能足够，完成一次向前迈步；重力做正功，碰撞损失一部分。
2. **$\omega_2<\dot\theta_n^+<\omega_1$**：动能不足以翻过最高点，轮子**原路摆回**，与旧触地点碰撞；速度方向反转，大小乘以 $\cos(2\alpha)$。
3. **$\dot\theta_n^+<\omega_2$**：反向速度足够大，完成一次**向后迈步**；从当前触地点到后方新触地点，质心要上升
   $$
   \Delta h=2l\sin\alpha\sin\gamma,
   $$
   所以动能减少 $2mgl\sin\alpha\sin\gamma$，对应根号内的 $-4(g/l)\sin\alpha\sin\gamma$；再乘碰撞因子 $\cos(2\alpha)$。

**阈值关系**：把 (39) 平方并利用 (29) 的同类恒等式：

$$
\omega_2^2-4\frac{g}{l}\sin\alpha\sin\gamma
=2\frac{g}{l}\big[1-\cos(\alpha+\gamma)\big]
-4\frac{g}{l}\sin\alpha\sin\gamma
=2\frac{g}{l}\big[1-\cos(\alpha-\gamma)\big]
=\omega_1^2. \tag{41}
$$

所以

$$
\boxed{\omega_2^2-\omega_1^2=4\frac{g}{l}\sin\alpha\sin\gamma}. \tag{42}
$$

在 $\dot\theta_n=\omega_1$ 或 $\omega_2$ 时，映射**无定义**：从这些初值出发，轮子会恰好停在不稳定平衡点 $\theta=0,\dot\theta=0$，永远不会再返回截面。

当 $\gamma>\alpha$ 时，站立平衡不存在，$\omega_1=0$，映射退化为

$$
\dot\theta_{n+1}^+=
\begin{cases}
\cos(2\alpha)\sqrt{(\dot\theta_n^+)^2+4\frac{g}{l}\sin\alpha\sin\gamma},
& 0\le\dot\theta_n^+,\\[8pt]
-\dot\theta_n^+\cos(2\alpha),
& \omega_2<\dot\theta_n^+<0,\\[8pt]
-\cos(2\alpha)\sqrt{(\dot\theta_n^+)^2-4\frac{g}{l}\sin\alpha\sin\gamma},
& \dot\theta_n^+\le\omega_2.
\end{cases} \tag{43}
$$

### 4.4.7 固定点与稳定性

固定点要求

$$
\dot\theta_{n+1}^+=\dot\theta_n^+=\omega^*. \tag{44}
$$

**（1）站立固定点**

$$
\omega^*_{stand}=0, \tag{45}
$$

仅当 $\gamma<\alpha$ 时存在（质心可以落在支撑多边形内）。

**（2）滚动固定点**

把 $\omega^*$ 代入向前分支 (38)：

$$
\omega^*=\cos(2\alpha)\sqrt{(\omega^*)^2+D},
\qquad
D:=4\frac{g}{l}\sin\alpha\sin\gamma. \tag{46}
$$

两边平方：

$$
(\omega^*)^2=\cos^2(2\alpha)\big[(\omega^*)^2+D\big]. \tag{47}
$$

整理：

$$
(\omega^*)^2\big[1-\cos^2(2\alpha)\big]
=D\cos^2(2\alpha). \tag{48}
$$

用 $1-\cos^2(2\alpha)=\sin^2(2\alpha)$：

$$
(\omega^*)^2=\frac{\cos^2(2\alpha)}{\sin^2(2\alpha)}D
=\cot^2(2\alpha)\,D. \tag{49}
$$

所以

$$
\boxed{
\omega^*_{roll}
=\cot(2\alpha)\sqrt{4\frac{g}{l}\sin\alpha\sin\gamma}
}. \tag{50}
$$

它成为合法固定点的条件是 $\omega^*_{roll}>\omega_1$（否则轮子根本走不到下一步）。

**[笔记] 能量平衡的另一种推导（更直观）**：

设每步开始动能为 $K$。重力在一步中补入

$$
\Delta K_g=2mgl\sin\alpha\sin\gamma. \tag{51}
$$

碰撞后动能变为

$$
K^+=\cos^2(2\alpha)\big(K+\Delta K_g\big). \tag{52}
$$

固定点要求 $K^+=K$：

$$
K\big[1-\cos^2(2\alpha)\big]
=\cos^2(2\alpha)\Delta K_g. \tag{53}
$$

代入 $K=\frac12ml^2\omega^2$、$\Delta K_g=2mgl\sin\alpha\sin\gamma$，解得同样的 (50)。这说明滚动步态的本质就是**重力输入与碰撞耗散的平衡**。

**（3）稳定性**

对向前分支求导：

$$
P(\omega)=\cos(2\alpha)\sqrt{\omega^2+D},
\qquad
P'(\omega)=\cos(2\alpha)\frac{\omega}{\sqrt{\omega^2+D}}. \tag{54}
$$

在固定点处，由 (46)：

$$
\sqrt{(\omega^*)^2+D}=\frac{\omega^*}{\cos(2\alpha)}. \tag{55}
$$

代入 (54)：

$$
\boxed{
P'(\omega^*)=\cos^2(2\alpha)
}. \tag{56}
$$

因为 $\cos^2(2\alpha)<1$（只要 $\alpha$ 不是 $\pi/2$ 的整数倍），所以

$$
|P'(\omega^*)|<1
\quad\Longrightarrow\quad
\text{滚动极限环局部指数稳定}. \tag{57}
$$

> **漂亮的结论**：稳定性竟然只由几何角 $\alpha$ 决定，与 $g,l,\gamma$ 无关！$\alpha=\pi/8$ 时 $P'=0.5$，偏离固定点后误差每步减半。
>
> **直觉**：如果这一步速度偏大，下一步重力输入相对不变、碰撞损失按比例增大，速度被拉回；反之亦然。碰撞的耗散是"自校正"的来源。

### 4.4.8 分岔与吸引域

固定点的存在性随 $\gamma$ 变化：

- **$\gamma$ 很小**：重力输入不足以补偿碰撞损失，没有滚动固定点，轮子最终停下（站立）；
- **$\gamma$ 增大到临界值 $\gamma_c$**：滚动固定点从 $\omega_1$ 处分岔出来；
- **$\gamma_c<\gamma<\alpha$**：同时存在站立固定点（稳定）和滚动固定点（稳定），吸引域由初值决定；
- **$\gamma>\alpha$**：站立平衡消失，只剩滚动固定点。

临界条件 $\omega^*_{roll}(\gamma_c)=\omega_1(\gamma_c)$。对 $g=9.8,l=1,\alpha=\pi/8$ 数值求解：

$$
\gamma_c\approx0.0682\ \text{rad}\approx3.91^\circ. \tag{58}
$$

数值表（$m=l=1,\ g=9.8,\ \alpha=\pi/8$）：

| $\gamma$ | $\omega_1$ | $\omega^*_{roll}$ | 滚动固定点？ | 站立固定点？ |
|---|---|---|---|---|
| 0.05 | 1.0676 | 0.8659 | 否（$\omega^*<\omega_1$） | 是 |
| 0.08 | 0.9749 | 1.0949 | 是 | 是 |
| 0.15 | 0.7579 | 1.4972 | 是 | 是 |
| 0.50 | 0.3357 | 2.6818 | 是 | 否（$\gamma>\alpha$） |

相图上，收敛到滚动固定点的区域（吸引域）与收敛到站立的分界线清晰可见：初值速度足够大就进入滚动步态，否则摆几下后停下。

### 4.4.9 站立稳定性的微妙之处

在**原始连续坐标**里，站立是系统的一个固定点；但在 hybrid 模型里，它对应**无限频繁的碰撞**（轮子同时由两根辐条支撑，或不断在两根辐条间微幅切换）。这类"Zeno 现象"不能直接套用普通固定点的线性化：

- 返回映射在 $\omega=0$ 处是奇异的（返回时间趋于无穷或 0）；
- 碰撞序列可能不收敛到有限时间；
- 需要专门的 hybrid 系统稳定性理论。

教材在此处只给了定性说明。对我们的目的而言，重要的是：**行走的"自然"稳定性概念是周期轨稳定性，而不是站立固定点稳定性**。站立只是 $\gamma<\alpha$ 时的一个特殊平衡，不能代表行走。

> **自检**：本节所有关键公式都可用能量守恒 + 角动量守恒推出，没有引入额外假设。附录 A 验证了 (50) 的固定点方程与 (56) 的导数；数值表中 $\omega^*_{roll}<\omega_1$ 的情形确实不是合法固定点（轮子走不到下一步）。

---

## 4.5 罗盘步态（The Compass Gait）

### 4.5.1 模型与坐标

无轮缘轮只建模了支撑腿，并假设"碰撞时总有一条腿恰好到位"。罗盘步态去掉这个假设：只保留两根辐条作为两条腿，在髋部加一个铰链，并给每条腿加一个集中质量，用来研究**摆动腿动力学**。

模型参数：

- 腿长 $l=a+b$，其中 $a$ 是支撑腿质点到脚的距离，$b$ 是摆动腿质点到髋的距离；
- 每条腿一个点质量 $m$，髋部一个点质量 $m_h$；
- 髋关节可施加力矩 $\tau$，**地面没有任何驱动**；
- 广义坐标 $\mathbf{q}=[\theta_{st},\theta_{sw}]^\top$，分别是支撑腿和摆动腿的绝对角。

> **坐标符号约定**：原书的矩阵对应"两腿角度取相反方向"的镜像约定。具体地，取
> $$
> \mathbf{p}_h=l\begin{bmatrix}\sin\theta_{st}\\ \cos\theta_{st}\end{bmatrix},\quad
> \mathbf{p}_{st}=a\begin{bmatrix}\sin\theta_{st}\\ \cos\theta_{st}\end{bmatrix},\quad
> \mathbf{p}_{sw}=\mathbf{p}_h-b\begin{bmatrix}\sin\theta_{sw}\\ \cos\theta_{sw}\end{bmatrix}.
> $$
> 如果你采用更"自然"的 $\mathbf{p}_{sw}=\mathbf{p}_h+b[\sin\theta_{sw},\cos\theta_{sw}]^\top$，耦合项的符号会整体反过来。两种约定物理等价（相当于把 $x$ 轴取反），但**不能混用**，否则 $M_{12}$、$C$、$\tau_g$ 的符号会互相矛盾。

此外还假设：

- 摆动腿在摆动过程中可以**收缩抬脚**（retract）以离地，且不影响其质心位置；
- 脚跟着地为完全非弹性碰撞，碰撞后新支撑脚速度为零；
- 支撑脚是理想铰链，不打滑。

### 4.5.2 运动学

记 $s_{st}=\sin\theta_{st}$，$c_{st}=\cos\theta_{st}$，$s_{sw}=\sin\theta_{sw}$，$c_{sw}=\cos\theta_{sw}$，$\Delta=\theta_{st}-\theta_{sw}$。

三个质点的位置（以支撑脚为原点）：

$$
\mathbf{p}_h=l\begin{bmatrix}s_{st}\\ c_{st}\end{bmatrix},\qquad
\mathbf{p}_{st}=a\begin{bmatrix}s_{st}\\ c_{st}\end{bmatrix},\qquad
\mathbf{p}_{sw}=
\begin{bmatrix}
l s_{st}-b s_{sw}\\
l c_{st}-b c_{sw}
\end{bmatrix}. \tag{59}
$$

对时间求导：

$$
\dot{\mathbf{p}}_h=l\dot\theta_{st}\begin{bmatrix}c_{st}\\ -s_{st}\end{bmatrix},\qquad
\dot{\mathbf{p}}_{st}=a\dot\theta_{st}\begin{bmatrix}c_{st}\\ -s_{st}\end{bmatrix}, \tag{60}
$$

$$
\dot{\mathbf{p}}_{sw}=
\begin{bmatrix}
l c_{st}\dot\theta_{st}-b c_{sw}\dot\theta_{sw}\\
-l s_{st}\dot\theta_{st}+b s_{sw}\dot\theta_{sw}
\end{bmatrix}. \tag{61}
$$

### 4.5.3 动能与势能

**动能**：

$$
T=\frac12m_h\|\dot{\mathbf{p}}_h\|^2
+\frac12m\|\dot{\mathbf{p}}_{st}\|^2
+\frac12m\|\dot{\mathbf{p}}_{sw}\|^2. \tag{62}
$$

前两项：

$$
\|\dot{\mathbf{p}}_h\|^2=l^2\dot\theta_{st}^2,\qquad
\|\dot{\mathbf{p}}_{st}\|^2=a^2\dot\theta_{st}^2. \tag{63}
$$

第三项展开：

$$
\|\dot{\mathbf{p}}_{sw}\|^2
=l^2\dot\theta_{st}^2+b^2\dot\theta_{sw}^2
-2lb\cos(\theta_{st}-\theta_{sw})\dot\theta_{st}\dot\theta_{sw}. \tag{64}
$$

（交叉项来自 $c_{st}c_{sw}+s_{st}s_{sw}=\cos\Delta$。）

合并：

$$
\boxed{
T=\frac12\big[(m_h+m)l^2+ma^2\big]\dot\theta_{st}^2
+\frac12mb^2\dot\theta_{sw}^2
-mlb\cos\Delta\,\dot\theta_{st}\dot\theta_{sw}
}. \tag{65}
$$

**势能**：

$$
U=m_hg\,l c_{st}+mg\,a c_{st}+mg\big(l c_{st}-b c_{sw}\big). \tag{66}
$$

整理：

$$
\boxed{
U=\big[(m_h+m)l+ma\big]g\cos\theta_{st}-mbg\cos\theta_{sw}
}. \tag{67}
$$

### 4.5.4 拉格朗日方程与标准形式

对 $\theta_{st}$：

$$
\frac{\partial T}{\partial\dot\theta_{st}}
=\big[(m_h+m)l^2+ma^2\big]\dot\theta_{st}
-mlb\cos\Delta\,\dot\theta_{sw}. \tag{68}
$$

求全导数（注意 $\Delta$ 依赖时间）：

$$
\frac{d}{dt}\frac{\partial T}{\partial\dot\theta_{st}}
=\big[(m_h+m)l^2+ma^2\big]\ddot\theta_{st}
-mlb\cos\Delta\,\ddot\theta_{sw}
+mlb\sin\Delta\,(\dot\theta_{st}-\dot\theta_{sw})\dot\theta_{sw}. \tag{69}
$$

$$
\frac{\partial T}{\partial\theta_{st}}
=mlb\sin\Delta\,\dot\theta_{st}\dot\theta_{sw}. \tag{70}
$$

$$
\frac{\partial U}{\partial\theta_{st}}
=-\big[(m_h+m)l+ma\big]g\sin\theta_{st}. \tag{71}
$$

代入

$$
\frac{d}{dt}\frac{\partial T}{\partial\dot\theta_{st}}
-\frac{\partial T}{\partial\theta_{st}}
+\frac{\partial U}{\partial\theta_{st}}=0, \tag{72}
$$

得到

$$
\boxed{
\big[(m_h+m)l^2+ma^2\big]\ddot\theta_{st}
-mlb\cos\Delta\,\ddot\theta_{sw}
-mlb\sin\Delta\,\dot\theta_{sw}^2
-\big[(m_h+m)l+ma\big]g\sin\theta_{st}=0
}. \tag{73}
$$

对 $\theta_{sw}$：

$$
\frac{\partial T}{\partial\dot\theta_{sw}}
=mb^2\dot\theta_{sw}-mlb\cos\Delta\,\dot\theta_{st}, \tag{74}
$$

$$
\frac{d}{dt}\frac{\partial T}{\partial\dot\theta_{sw}}
=mb^2\ddot\theta_{sw}-mlb\cos\Delta\,\ddot\theta_{st}
+mlb\sin\Delta\,(\dot\theta_{st}-\dot\theta_{sw})\dot\theta_{st}, \tag{75}
$$

$$
\frac{\partial T}{\partial\theta_{sw}}
=-mlb\sin\Delta\,\dot\theta_{st}\dot\theta_{sw}, \tag{76}
$$

$$
\frac{\partial U}{\partial\theta_{sw}}=mbg\sin\theta_{sw}. \tag{77}
$$

代入并整理（$\dot\theta_{st}\dot\theta_{sw}$ 项抵消）：

$$
\boxed{
-mlb\cos\Delta\,\ddot\theta_{st}+mb^2\ddot\theta_{sw}
+mlb\sin\Delta\,\dot\theta_{st}^2
+mbg\sin\theta_{sw}=\tau
}. \tag{78}
$$

**为什么 $\theta_{st}$ 方程的右端是 0、$\theta_{sw}$ 方程的右端是 $\tau$？** 因为髋关节力矩是**内部力矩**：它对支撑腿和摆动腿等大反向。写成矩阵形式，输入向量为

$$
\mathbf{B}=\begin{bmatrix}-1\\1\end{bmatrix}. \tag{79}
$$

于是标准形式

$$
\mathbf{M}(\mathbf{q})\ddot{\mathbf{q}}
+\mathbf{C}(\mathbf{q},\dot{\mathbf{q}})\dot{\mathbf{q}}
=\boldsymbol{\tau}_g(\mathbf{q})+\mathbf{B}\tau \tag{80}
$$

中：

$$
\boxed{
\mathbf{M}=
\begin{bmatrix}
(m_h+m)l^2+ma^2 & -mlb\cos\Delta\\
-mlb\cos\Delta & mb^2
\end{bmatrix}
}, \tag{81}
$$

$$
\boxed{
\mathbf{C}=
\begin{bmatrix}
0 & -mlb\sin\Delta\,\dot\theta_{sw}\\
mlb\sin\Delta\,\dot\theta_{st} & 0
\end{bmatrix}
}, \tag{82}
$$

$$
\boxed{
\boldsymbol{\tau}_g=
\begin{bmatrix}
\big[(m_h+m)l+ma\big]g\sin\theta_{st}\\
-mbg\sin\theta_{sw}
\end{bmatrix}
}. \tag{83}
$$

### 4.5.5 欠驱动的数学结构

$\mathbf{B}=[-1,1]^\top$ 有两层含义：

1. **秩为 1**：只有两个广义坐标的**差值**方向可以被直接驱动，即髋关节相对角；支撑腿的绝对角没有直接输入。
2. **地面反力不做功**：支撑脚固定，地面反力作用点速度为零；髋力矩是内力矩。因此机器人在支撑相内是标准的欠驱动机械系统。

和无轮缘轮相比，罗盘步态多了一个自由度（摆动腿），但**支撑腿的绝对运动仍然只能靠耦合间接控制**。这也是所有腿式机器人的共同结构：脚不能"抓地"，只能通过接触点和身体的相对运动来间接影响质心。

> **技术注记（C 矩阵不唯一）**：这里的 $\mathbf{C}$ 恰好是 Christoffel 形式，满足 $\dot{\mathbf{M}}-2\mathbf{C}$ 反对称。原书其他地方可能给出等价的非 Christoffel 形式；只要 $\mathbf{C}\dot{\mathbf{q}}$ 相同，动力学就相同。做能量分析时建议统一用 Christoffel 形式。

### 4.5.6 碰撞：浮动基座 + 投影（完整推导） [笔记]

脚跟着地是瞬时非弹性碰撞。碰撞前，摆动脚以某个速度撞地；碰撞后它必须静止并成为新的支撑脚。这个过程不能用普通的连续动力学描述，需要**冲量–动量**关系。

教材给出的四步法：

#### 第一步：加浮动基座

在碰撞前的支撑脚尖处加一个自由的 $(x,y)$ 关节，得到

$$
\mathbf{q}_{fb}=[x,y,\theta_{st},\theta_{sw}]^\top. \tag{84}
$$

此时支撑脚尖固定，故碰撞前

$$
\dot x^-=\dot y^-=0. \tag{85}
$$

#### 第二步：计算浮动基座质量矩阵

用与前面相同的动能表达式，把 $x,y$ 的平动也计入，得到 $\mathbf{M}_{fb}(\mathbf{q}_{fb})$。按 §4.5.1 的镜像约定，它的显式为

$$
\mathbf{M}_{fb}=
\begin{bmatrix}
2m+m_h & 0 & (am+lm+lm_h)c_{st} & -bmc_{sw}\\
0 & 2m+m_h & -(am+lm+lm_h)s_{st} & bms_{sw}\\
(am+lm+lm_h)c_{st} & -(am+lm+lm_h)s_{st} & a^2m+l^2m+l^2m_h & -blm\cos\Delta\\
-bmc_{sw} & bms_{sw} & -blm\cos\Delta & b^2m
\end{bmatrix}. \tag{86}
$$

它只是把 $\mathbf{M}$ 嵌进一个 $4\times4$ 的平动–转动耦合矩阵；附录 A 用符号计算验证了这一点。

#### 第三步：写出摆动脚尖位置与雅可比

碰撞前摆动脚尖（即即将触地的点）位置

$$
\boldsymbol\phi(\mathbf{q}_{fb})=
\begin{bmatrix}
x+l\sin\theta_{st}-b\sin\theta_{sw}\\
y+l\cos\theta_{st}-b\cos\theta_{sw}
\end{bmatrix}. \tag{87}
$$

雅可比

$$
\mathbf{J}=\frac{\partial\boldsymbol\phi}{\partial\mathbf{q}_{fb}}=
\begin{bmatrix}
1 & 0 & l\cos\theta_{st} & -b\cos\theta_{sw}\\
0 & 1 & -l\sin\theta_{st} & b\sin\theta_{sw}
\end{bmatrix}. \tag{88}
$$

#### 第四步：冲量投影

设碰撞冲量为 $\mathbf{J}^\top\boldsymbol\lambda$（$\boldsymbol\lambda$ 是二维冲量），动量变化满足

$$
\mathbf{M}_{fb}\big(\dot{\mathbf{q}}^+-\dot{\mathbf{q}}^-\big)
=\mathbf{J}^\top\boldsymbol\lambda. \tag{89}
$$

碰撞后摆动脚尖静止：

$$
\mathbf{J}\dot{\mathbf{q}}^+=0. \tag{90}
$$

由 (89) 解出 $\dot{\mathbf{q}}^+$，再代入 (90) 求 $\boldsymbol\lambda$：

$$
\dot{\mathbf{q}}^+
=\Big[\mathbf{I}
-\mathbf{M}_{fb}^{-1}\mathbf{J}^\top
\big(\mathbf{J}\mathbf{M}_{fb}^{-1}\mathbf{J}^\top\big)^{-1}\mathbf{J}
\Big]\dot{\mathbf{q}}^-. \tag{91}
$$

**几何解释**：方括号是一个投影算子。它在 $\mathbf{M}_{fb}$ 定义的动能度量下，把碰撞前速度**正交投影**到"满足 $\mathbf{J}\dot{\mathbf{q}}^+=0$"的约束子空间。等价地，它使碰撞后的动能在该约束下最小（Gauss 最小约束原理），同时保证冲量只沿约束方向作用。

实际操作：

1. 取 $\dot{\mathbf{q}}^-$ 的后两个分量（$\dot\theta_{st}^-,\dot\theta_{sw}^-$），前两个分量为 0；
2. 用 (91) 得到 $\dot{\mathbf{q}}^+$；
3. 取后两个分量，得到碰撞后的关节速度；
4. **交换支撑腿与摆动腿的角色**：
   $$
   \theta_{st}\leftarrow\theta_{sw},\qquad
   \theta_{sw}\leftarrow\theta_{st},\qquad
   \dot\theta_{st}\leftarrow\dot\theta_{sw},\qquad
   \dot\theta_{sw}\leftarrow\dot\theta_{st}. \tag{92}
   $$

> **为什么可以只取后两个分量？** 因为碰撞后新的支撑脚速度为 0（由约束保证），浮动基座的平动速度完全由新支撑脚固定的条件决定；在最小坐标 $[\theta_{st},\theta_{sw}]$ 中，平动已经被消去。
>
> **守恒量**：碰撞前后绕**新触地点**的角动量守恒。冲量作用在新触地点，力矩为零；投影公式 (91) 正是这一守恒律在浮动基座坐标下的实现。

### 4.5.7 极限环与吸引域

数值积分这些方程，确实能得到一条**稳定的极限环**：

- 支撑相：轨迹近似无轮缘轮的单摆弧线；
- 脚跟着地：速度出现明显的向下跳变（碰撞损失）；
- 摆动相：摆动腿绕髋关节画一个大弧，然后重新触地。

教材给出的典型参数：

$$
m=5\,\text{kg},\quad m_h=10\,\text{kg},\quad a=b=0.5\,\text{m},\quad
\mathbf{x}(0)=[0,0,0.4,-2.0]^\top. \tag{93}
$$

但要注意：**吸引域是一条很窄的带**。与无轮缘轮相比，罗盘步态对初值敏感得多；教材说"需要花一些功夫才能找到能稳定行走的初值"。

原因可以直观理解：

- 无轮缘轮有大量辐条，碰撞几何天然地把状态"夹"回滚动解；
- 罗盘步态只有两条腿，摆动腿的落点必须与支撑腿的节奏精确配合，否则下一步的碰撞构型就错了；
- 碰撞是耗散的，但耗散太强或太弱都会破坏极限环。

> **工程含义**：被动行走原理漂亮，但"能走"不等于"能抗扰"。真正的机器人必须加控制（能量注入、落脚点调节、躯干姿态控制）来扩大吸引域。这也是后续章节（LQR、轨迹优化、接触控制）的动机。

### 4.5.8 数值验证

附录 A 做了两项检查：

1. **符号推导**：用 SymPy 从 $T,U$ 重新推导 (81)–(83)，与书中的 $\mathbf{M},\mathbf{C},\boldsymbol{\tau}_g$ 逐项对比，差为零；
2. **碰撞投影**：取一组随机状态与速度，验证 (91) 给出的 $\dot{\mathbf{q}}^+$ 满足 $\mathbf{J}\dot{\mathbf{q}}^+=0$，且动量变化 $\mathbf{M}_{fb}(\dot{\mathbf{q}}^+-\dot{\mathbf{q}}^-)$ 确实落在 $\mathbf{J}^\top$ 的列空间（即只由冲量产生）。

> **自检**：本节最容易错的是符号约定。记住一条判据：$\mathbf{M}$ 必须对称正定，$\mathbf{C}$ 必须满足 $\mathbf{C}\dot{\mathbf{q}}$ 与拉格朗日方程一致。若你算出的 $\mathbf{M}_{12}$ 符号与原书相反，先检查摆动腿质心是不是取在了髋的另一侧。

---

## 4.6 膝盖步行者与被动行走的扩展

### 4.6.1 Kneed Walker：三连杆 ↔ 两连杆的混合系统

罗盘步态没有膝盖，摆动腿只能像一根刚性杆一样绕过髋关节，抬脚高度受限，也难以适应崎岖地形。**膝盖步行者（kneed walker）** 给摆动腿加一个膝关节，使步态更接近人类，并提高离地间隙（foot clearance）。

**模型**：

- 每条腿两连杆（大腿 + 小腿），每根连杆一个集中质量；
- 支撑腿在支撑相内视为一根刚性杆；
- 摆动腿的膝关节在摆动初期**自由**，直到腿伸直；
- 腿伸直时发生 **knee-strike**（膝碰撞），膝盖锁定；
- 随后系统变成两连杆的罗盘步态；
- 脚跟着地发生 **heel-strike**，两腿交换角色，膝盖重新解锁。

**一个完整步态周期**：

```text
三连杆单摆（支撑腿 + 摆动大腿 + 摆动小腿，膝自由）
        │  摆动腿弯曲、前摆
        ▼
   knee-strike（膝伸直、锁定；非弹性碰撞，速度跳变）
        │
        ▼
两连杆罗盘步态（膝盖锁死）
        │  脚跟着地
        ▼
   heel-strike（非弹性碰撞，速度跳变；两腿交换）
        │
        ▼
三连杆单摆（新的摆动腿膝盖解锁）
```

因此一个周期包含：

- **两个连续动力学阶段**：三连杆阶段、两连杆阶段；
- **两个离散碰撞事件**：knee-strike、heel-strike。

这是一个典型的 **hybrid limit cycle**。数值上，给定合适的质量分布与初始条件，可以找到稳定的周期解。相图上：

- 上半部分是摆动相；
- 下半部分是支撑相；
- 两个 heel-strike 和两个 knee-strike 表现为速度的瞬时跳变；
- 极限环按顺时针方向遍历。

**与罗盘步态的对比**：

| 特性 | 罗盘步态 | 膝盖步行者 |
|---|---|---|
| 摆动腿 | 刚性 | 大腿 + 小腿，膝可弯 |
| 离散事件 | 1 个（heel-strike） | 2 个（knee-strike + heel-strike） |
| 抬脚高度 | 低 | 高 |
| 崎岖地形能力 | 差 | 较好 |
| 分析难度 | 4 状态 | 更多状态、更多切换 |

### 4.6.2 曲足（Curved Feet）

在脚底加一段圆弧（曲足），可以显著扩大膝盖步行者极限环的**吸引域**。直观原因：

- 直脚触地时，接触点瞬间从脚跟着地切换到整个脚掌，构型约束很"硬"；
- 曲足相当于一个**滚动接触**，接触点随身体姿态连续移动，碰撞更"柔和"；
- 曲率提供了额外的几何自由度，使步态对落点误差更宽容。

McGeer 的膝盖步行器还使用了"膝盖去弹跳器（knee de-bouncers）"等机械技巧来避免膝盖在支撑相中意外弯曲。这类被动行走器的成功启动本身并不容易，需要精确的初始条件与地面条件。

### 4.6.3 能量注入：从斜坡到平地

被动行走器依赖下坡：重力在每步中补回碰撞损失。要在**平地**上行走，必须人为注入少量能量。Collins 等（2005）系统研究了这一问题：

- 在脚踝处做**蹬地（push-off）**；
- 在髋部施加小力矩；
- 或者用弹簧–棘轮机构存储/释放能量。

令人惊讶的是，**很小的能量输入**就能让被动行走器在平地上高效行走，能量效率甚至优于当时许多主动控制的双足机器人。代表性平台：

- Cornell 双足机器人；
- TU Delft 双足机器人；
- MIT 的 "Toddler"（学习型双足，曾在 MIT 博物馆展出）。

> **控制启示**：如果机械本身已经有一个稳定的被动步态，控制器只需要做"小幅修正 + 能量补充"，而不需要从头规划每一个关节轨迹。这是"**控制与机械协同设计**"的早期范例。

### 4.6.4 更远的扩展：Strandbeest

荷兰艺术家 Theo Jansen 的 **Strandbeest** 把被动行走思想推向了艺术极端：用一套连杆机构把连续旋转变成多足的行走步态，完全靠风力驱动，在沙滩上"自己走路"。

MIT 的 Robin Deits 曾在 Drake 中复现了 Jansen 最早的机构之一。它说明：

- 即使没有传感器和计算机，**机构设计本身**也能编码出稳定的步态；
- 被动动态行走的核心不是"控制算法"，而是"动力学与几何"；
- 对机器人设计者而言，**好的机械结构可以大幅降低控制的负担**。

> **自检**：膝盖步行者的关键在于事件顺序——先 knee-strike（膝伸直锁定），再 heel-strike（脚跟着地换腿）。如果顺序写反，或者膝盖锁定后忘记切换质量矩阵，仿真会出现非物理的速度跳变。相图上每个周期应恰好有 **2 次 heel-strike + 2 次 knee-strike**。

---

## 4.7 SLIP：弹簧负载倒立摆

### 4.7.1 模型

行走与奔跑的质心轨迹有本质区别：

- **行走**：质心从支撑脚上方"拱"过去（vaulting），像无轮缘轮；
- **奔跑**：质心在支撑脚上方**压缩下沉**，像弹簧–质量系统。

**SLIP（Spring-Loaded Inverted Pendulum）** 用一个最简模型抓住奔跑的本质：

- 一个集中质量 $m$；
- 一条无质量、带弹簧的腿，原长 $l_0$，刚度 $k$；
- 腿角 $\theta$ 可在飞行相瞬时调整；
- 支撑相完全"弹道"，不加控制。

系统是分段的：飞行相和支撑相用不同的状态变量与动力学。

### 4.7.2 飞行相

状态

$$
\mathbf{x}=[x,\ z,\ \dot x,\ \dot z]^\top. \tag{94}
$$

动力学就是抛体运动：

$$
\dot{\mathbf{x}}=
\begin{bmatrix}
\dot x\\ \dot z\\ 0\\ -g
\end{bmatrix}. \tag{95}
$$

腿无质量，因此可以认为腿角能在飞行相被瞬时设置到任意值；在建模时把 $\theta$ 当作控制输入：

$$
\theta=u. \tag{96}
$$

### 4.7.3 支撑相：完整拉格朗日推导

以触地点为原点，状态

$$
\mathbf{x}=[r,\ \theta,\ \dot r,\ \dot\theta]^\top. \tag{97}
$$

质心位置

$$
\mathbf{x}_m=
\begin{bmatrix}
-r\sin\theta\\
r\cos\theta
\end{bmatrix}. \tag{98}
$$

（$\theta$ 从竖直方向量起，$r$ 为腿长。）

动能与势能：

$$
T=\frac12m\big(\dot r^2+r^2\dot\theta^2\big), \tag{99}
$$

$$
U=mgr\cos\theta+\frac12k(l_0-r)^2. \tag{100}
$$

拉格朗日量 $L=T-U$。

**对 $r$**：

$$
\frac{\partial L}{\partial\dot r}=m\dot r
\quad\Longrightarrow\quad
\frac{d}{dt}\frac{\partial L}{\partial\dot r}=m\ddot r, \tag{101}
$$

$$
\frac{\partial L}{\partial r}
=mr\dot\theta^2-mg\cos\theta+k(l_0-r). \tag{102}
$$

代入欧拉–拉格朗日方程：

$$
m\ddot r-\big[mr\dot\theta^2-mg\cos\theta+k(l_0-r)\big]=0, \tag{103}
$$

即

$$
\boxed{
m\ddot r-mr\dot\theta^2+mg\cos\theta-k(l_0-r)=0
}. \tag{104}
$$

**对 $\theta$**：

$$
\frac{\partial L}{\partial\dot\theta}=mr^2\dot\theta
\quad\Longrightarrow\quad
\frac{d}{dt}\frac{\partial L}{\partial\dot\theta}
=mr^2\ddot\theta+2mr\dot r\dot\theta, \tag{105}
$$

$$
\frac{\partial L}{\partial\theta}=mgr\sin\theta. \tag{106}
$$

于是

$$
\boxed{
mr^2\ddot\theta+2mr\dot r\dot\theta-mgr\sin\theta=0
}. \tag{107}
$$

支撑相没有控制输入：弹簧力与重力完全决定运动。

### 4.7.4 能量守恒

总机械能

$$
E=\frac12m\big(\dot r^2+r^2\dot\theta^2\big)
+mgr\cos\theta+\frac12k(l_0-r)^2. \tag{108}
$$

**方法一（结构论证）**：$L$ 不显含时间，且动能是速度的二次型、约束不显含时间（scleronomic），所以 $E$ 沿运动恒定。

**方法二（直接验证）**：求导

$$
\dot E
=m\dot r\ddot r+mr\dot r\dot\theta^2+mr^2\dot\theta\ddot\theta
-mg\dot r\cos\theta+mg r\sin\theta\,\dot\theta
-k(l_0-r)\dot r. \tag{109}
$$

把 (104) 乘以 $\dot r$、(107) 乘以 $\dot\theta$ 后相加：

$$
\dot r\big[m\ddot r-mr\dot\theta^2+mg\cos\theta-k(l_0-r)\big]
+\dot\theta\big[mr^2\ddot\theta+2mr\dot r\dot\theta-mgr\sin\theta\big]=0. \tag{110}
$$

这正是 $\dot E=0$。附录 A 用 SymPy 复核了 (104)、(107) 与 $\dot E=0$。

> **关键区别**：SLIP 的落地**没有速度跳变**。腿无质量、脚无质量，触地不产生冲量，所以没有碰撞耗散。整个系统是**分段哈密顿**的，能量严格守恒。这与无轮缘轮"碰撞损失一半动能"形成鲜明对照，也是 §4.8 "部分稳定性"的根源。

### 4.7.5 切换条件与状态转换

**飞行 → 支撑**：脚触地，即

$$
z-l_0\cos\theta\le0. \tag{111}
$$

触地瞬间没有速度跳变。给定触地时的飞行状态 $(x,z,\dot x,\dot z)$ 和腿角 $\theta_{td}$，脚的位置为

$$
x_f=x+l_0\sin\theta_{td},\qquad z_f=0. \tag{112}
$$

支撑相初值：

$$
r=l_0,\qquad \theta=\theta_{td}, \tag{113}
$$

$$
\dot r=-\dot x\sin\theta_{td}+\dot z\cos\theta_{td}, \tag{114}
$$

$$
\dot\theta=-\frac{\dot x\cos\theta_{td}+\dot z\sin\theta_{td}}{l_0}. \tag{115}
$$

（由 (98) 求导并反解得到。）

**支撑 → 飞行**：弹簧回到原长并继续伸长：

$$
r\ge l_0. \tag{116}
$$

设此时脚的位置为 $x_f$，则飞行相初值：

$$
x=x_f-r\sin\theta,\qquad z=r\cos\theta, \tag{117}
$$

$$
\dot x=-\dot r\sin\theta-r\cos\theta\,\dot\theta, \tag{118}
$$

$$
\dot z=\dot r\cos\theta-r\sin\theta\,\dot\theta. \tag{119}
$$

> **自检**：把 (113)–(115) 与 (117)–(119) 互相代入，应该回到原状态。这两组转换式也是附录 A 仿真的核心；任何符号错误都会导致能量不守恒（可以用 $E$ 的变化量作为调试指标）。

---

## 4.8 Apex-to-Apex 映射与部分稳定性

### 4.8.1 降维：为什么还能画一维图？

无轮缘轮只有两个状态 $(\theta,\dot\theta)$，截面降一维后得到一维映射。SLIP 有四个状态，看起来没法和一维图打交道。但坐标选得好的话，仍然可以：

取**跳跃最高点（apex）** 作为庞加莱截面：

$$
\dot z=0. \tag{120}
$$

在最高点，水平位置 $x$ 与飞行动力学无关（系统有平移对称性），腿角 $\theta$ 可以在飞行相任意设置，也不进入飞行动力学。剩下两个量：

$$
z,\qquad \dot x. \tag{121}
$$

**关键观察**：在 apex，$z$ 与 $\dot x$ 并不独立。因为能量守恒，

$$
E=\frac12m\dot x^2+mgz=E_0. \tag{122}
$$

给定总能量 $E_0$ 和高度 $z$，水平速度就被确定（差一个符号）：

$$
\dot x=\pm\sqrt{\frac{2}{m}(E_0-mgz)}. \tag{123}
$$

所以可以在"能量恒定的切片"上，把映射写成**一维**的

$$
z[n+1]=P(z[n]). \tag{124}
$$

### 4.8.2 无量纲化

为了减少参数依赖，定义无量纲量：

$$
\tilde E=\frac{E}{mgl_0},\qquad
\tilde k=\frac{kl_0}{mg}. \tag{125}
$$

Geyer 等（2005）使用的典型参数：

$$
\tilde k=10.7,\qquad \tilde E=1.61,\qquad \theta_{td}=30^\circ. \tag{126}
$$

### 4.8.3 数值返回映射与固定点

用附录 A 的数值积分（$m=1,l_0=1,g=9.8$，按 (125) 换算 $k,E$），从 apex 积分到下一次 apex，得到下表：

| $z[n]$ | 0.868 | 0.878 | 0.888 | 0.898 | 0.908 | 0.918 | 0.928 | 0.938 | 0.948 |
|---|---|---|---|---|---|---|---|---|---|
| $z[n+1]$ | 0.8757 | 0.8774 | 0.8845 | 0.8941 | 0.9055 | 0.9184 | 0.9327 | 0.9483 | 0.9651 |

用阶梯图分析，有两个固定点：

1. **稳定固定点**：
   $$
   z^*\approx0.8768\,\text{m},\qquad P'(z^*)\approx0.488<1. \tag{127}
   $$
2. **不稳定固定点**：
   $$
   z^*\approx0.9168\,\text{m},\qquad P'(z^*)\approx1.346>1. \tag{128}
   $$

教材把这两个值近似写成 **0.87 m（稳定）** 与 **0.92 m（不稳定）**，与我们的数值一致。

**物理解读**：

- 稳定固定点较低，对应"压缩较深、水平速度较大"的步态；
- 不稳定固定点较高，对应"压缩较浅、水平速度较小"的步态；
- 从低于稳定固定点的初值出发，一步后会跳到一个稍高的高度；从两者之间出发，会收敛到稳定固定点；从高于不稳定固定点出发，会发散或失败。

### 4.8.4 失败模式：映射的定义域

apex-to-apex 映射不是对所有权 $z$ 都有定义：

1. **下边界**：若
   $$
   z<l_0\cos\theta_{td}, \tag{129}
   $$
   则在 apex 时脚已经低于地面，物理上不可能。对 $\theta_{td}=30^\circ$，$l_0\cos30^\circ\approx0.866$，这正是表中 $z=0.868$ 附近映射突然"截断"的原因。
2. **上边界/水平能量不足**：若 $z$ 太大（相对 $\tilde E$），水平速度太小，质心在支撑相压缩弹簧后没有足够的前向动能，会被弹回原方向，甚至向后倒。此时映射仍然可以积分，但不再对应"继续向前跑"的步态。

这两个边界都是**几何/能量约束**，不是数值误差。

### 4.8.5 部分稳定性：一个必须理解的陷阱

上面的"稳定固定点"结论有一个前提：**总能量恒定**。

一维映射 (124) 是把二维状态 $(z,\dot x)$ 投影到 $z$ 上得到的，投影时用了 (122) 的能量守恒。因此它只能告诉我们：

> 在总能量不变的扰动方向上，步态是稳定的。

它**不能**告诉我们：

- 地面高度变化导致能量改变时，系统是否还能回到步态；
- 推力/阻力扰动改变总能量时，稳定性如何；
- 不同能量水平之间的吸引关系。

这类稳定性称为**部分稳定性（partial stability）**。教材特别强调：

- 无轮缘轮的稳定性来自碰撞**耗散**，是"真"的渐近稳定；
- SLIP 是**分段哈密顿系统**，没有耗散，所谓"稳定固定点"只是能量切片上的几何性质；
- 这种"守恒系统也能表现出稳定周期轨"的现象曾引起理论物理界的兴趣（Holmes 等）。

> **工程含义**：SLIP 的 apex map 非常适合设计**每步一次**的控制器（§4.9），但不能直接用来证明机器人在真实地形上的稳定性。真实系统需要额外的能量调节机制（例如 Raibert 的跳跃高度控制）。

> **自检**：附录 A 的仿真中，每次 apex 的总能量都与初值一致（最大偏差约 $9\times10^{-10}$，即数值积分误差量级），这既验证了积分器与切换逻辑，也说明了为什么只能得到"部分稳定"。若把能量守恒人为破坏（例如加入阻尼），稳定固定点会移动，甚至消失。

---

## 4.9 SLIP 控制

### 4.9.1 每步一个决策

SLIP 的"腿无质量"假设有一个很大的好处：飞行相中腿角可以瞬时改变。因此控制可以抽象成**每步一个标量决策**：

$$
u[n]=\theta_{td}[n], \tag{130}
$$

即第 $n$ 次跳跃的触地角。apex-to-apex 映射变成参数化映射：

$$
z[n+1]=P\big(z[n],u[n]\big). \tag{131}
$$

由于能量守恒把状态压成一维，$P$ 是标量映射，控制器设计大大简化。

**最直接的做法**：在固定点 $(z^*,u^*)$ 处线性化，

$$
\delta z[n+1]=a\,\delta z[n]+b\,\delta u[n], \tag{132}
$$

其中

$$
a=\frac{\partial P}{\partial z}\Big|_{(z^*,u^*)},\qquad
b=\frac{\partial P}{\partial u}\Big|_{(z^*,u^*)}. \tag{133}
$$

若 $b\ne0$，可以用离散 LQR 或极点配置设计反馈

$$
\delta u[n]=-K\,\delta z[n]. \tag{134}
$$

### 4.9.2 无差拍控制（Deadbeat Control）

如果控制代价取零（与"无质量腿"假设一致），离散最优控制的解会变得非常激进：**一步把状态打到目标**。

对一维系统 (132)，取

$$
\delta u[n]=-\frac{a}{b}\,\delta z[n]
\quad\Longrightarrow\quad
\delta z[n+1]=0. \tag{135}
$$

这就是**无差拍控制**：一个采样周期内精确收敛。

**为什么离散时间可以做到，连续时间不行？**

- 连续时间中，若控制代价为零，最优控制通常是无界的，或者退化为 bang-bang；
- 离散时间中，只要输入有界且系统在一步内可控，就可以用**有限幅值**的输入实现有限步精确收敛；
- 对 SLIP，Geyer 等证明：在总能量允许的范围内，触地角与下一 apex 高度之间是**可逆**的，因此 (135) 的分母 $b\ne0$，无差拍控制可实现。

> **不是所有系统都能无差拍**：它要求映射在目标点附近可逆（相当于离散系统的相对阶为 1、且输入直接出现在下一步输出中）。SLIP 的幸运之处在于"触地角 → 下一跳高度"恰好是一一对应的。

### 4.9.3 盲跑崎岖地形

Ernst 等（2009）发现了一个更漂亮的结果：**无差拍触地角是目标高度（或前进速度）的光滑函数**。

于是可以预先设计一条**开环摆腿轨迹**

$$
\theta_{leg}(t)=\Theta(t-t_{\text{apex}}), \tag{136}
$$

其中 $t_{\text{apex}}$ 是上一次到达最高点的时刻。无论地面在什么高度、什么时候出现，只要脚在某个时刻碰到地面，当时的腿角就恰好是**无差拍角**。

这意味着：

- **不需要感知地形高度**；
- 只需要知道"距离上次 apex 过了多久"；
- 触地时腿角自动匹配当前所需的死拍控制量；
- 可以在崎岖地面上"盲跑"。

这在直觉上很难接受——"盲"、"奔跑"、"崎岖地形"、"无差拍"四个词居然能同时出现。但它有严格的模型依据：

1. 腿无质量，飞行相可瞬时摆腿；
2. 触地角与下一跳高度一一对应；
3. 地面高度只改变触地时刻，不改变"时刻 → 腿角"的映射。

这条结果还为生物力学中的 **ground-speed matching** 提供了一个控制论解释：人和动物在触地前会主动把腿向前摆，使脚以合适的速度/角度"迎"向地面。这看起来像复杂的神经控制，其实可能只是"按时间扫腿"的开环策略。

### 4.9.4 扩展与 Raibert 跳跃机器人

**SLIP 的生物学普适性**：

- 实验数据显示，从 $0.001\,\text{kg}$ 的蟑螂到 $135\,\text{kg}$ 的马，无量纲腿刚度都接近

$$
\tilde k=\frac{kl_0}{mg}\approx10. \tag{137}
$$

- 这说明 SLIP 抓住了跨尺度、跨物种的**动态相似性**；
- SLIP 的扩展还能解释蟑螂的横向稳定性。

**Raibert 的 Leg Laboratory**：

- 1980 年代，Marc Raibert 在 CMU/MIT 造出了一系列跳跃机器人；
- 2D Hopper 采用**三部分解耦控制**：跳跃高度、身体姿态、前进速度分别控制；
- 四足奔跑可以看成"四条腿像一条腿一样"的对称控制问题；
- 这些机器人在时间上早于 SLIP 模型的分析，但行为与 SLIP 高度吻合。

**现代机器人**：

- Spot Mini（Boston Dynamics）、Mini Cheetah（MIT）等采用**轻腿、点足**设计；
- 轻腿使"腿可瞬时摆到落点"的假设更接近现实；
- 点足简化了接触建模，也提高了对地形误差的鲁棒性；
- 这些平台的控制器仍然大量借用本章的"每步一次决策 + 返回映射"思想。

> **自检**：无差拍控制的关键是 $b\ne0$。如果触地角对下一跳高度的影响在某个工作点饱和（例如腿太短、角度限幅），$b\to0$，无差拍就会失效，必须退回 LQR 或加入能量调节。

---

## 4.10 杂耍（Juggling）

杂耍看起来与腿式机器人无关，但它用了**完全相同的工具**：混合动力学 + 庞加莱映射 + 低维返回映射。教材把它放在本章末尾，是为了说明这套方法的普适性。

### 4.10.1 直线杂耍模型

最简单的模型是**直线杂耍者（line juggler）**：

- 拍子（机器人）和球都被约束在一条竖直线上；
- 拍子位置 $r$，球位置 $b$；
- 球在空中满足

$$
\ddot b=-g; \tag{138}
$$

- 拍子有速度控制，可直接命令 $\dot r$；
- 拍球是瞬时弹性碰撞，恢复系数 $0\le e\le1$。

碰撞前后的速度关系（恢复系数定义）：

$$
v_b^+-v_p=-e\big(v_b^--v_p\big), \tag{139}
$$

即

$$
v_b^+=(1+e)v_p-e\,v_b^-. \tag{140}
$$

这里 $v_p=\dot r$ 是拍子速度，$v_b=\dot b$ 是球速度。

### 4.10.2 镜像律（Mirror Law）

Buehler 提出把拍子的运动限制为球的**缩放镜像**：

$$
r=-k(b,\dot b)\,b. \tag{141}
$$

这个看似奇怪的约束有两个关键好处：

1. **碰撞总发生在 $r=b=0$**：因为当 $b=0$ 时 $r=0$，球和拍子恰好在原点相遇；
2. **拍子速度与球速度成比例**：在 $b=0$ 处对 (141) 求导，

$$
\dot r=-k(0,\dot b)\,\dot b
-\underbrace{\left(\frac{\partial k}{\partial b}\dot b+
\frac{\partial k}{\partial\dot b}\ddot b\right)b}_{=0}
=-k(0,\dot b)\,\dot b. \tag{142}
$$

于是碰撞速度只由一个标量 $k(0,\dot b)$ 决定。

**为什么叫"镜像"？** 球向上运动时，拍子按比例向下让开；球向下运动时，拍子迎上去。拍子像球在镜子里的像，但缩放系数可以随状态变化。

### 4.10.3 能量反馈

选择

$$
k(b,\dot b)=\kappa_0+\frac{\kappa_1}{m}\big(E_d-E\big), \tag{143}
$$

其中

$$
E=mgb+\frac12m\dot b^2 \tag{144}
$$

是球的总能量，$E_d$ 是期望能量。

直观理解：

- 球能量低于期望：$k$ 增大，拍子迎球更"积极"，碰撞时注入能量；
- 球能量高于期望：$k$ 减小，拍子"退让"，碰撞时吸走能量。

### 4.10.4 返回映射与稳定性

取"碰撞前瞬间"为庞加莱截面。经过一次碰撞 + 一次飞行，得到一维返回映射：

$$
\boxed{
\dot b[n+1]
=\dot b[n]\left[1-\frac{1+e}{2}\kappa_1
\big(\dot b^2[n]-\dot b_d^2\big)\right]
}. \tag{145}
$$

其中 $\dot b_d$ 是对应期望能量 $E_d$ 的球速。

**[笔记] 推导思路**（教材给出结果，这里补全逻辑）：

1. 由镜像律 (142)，碰撞时拍子速度 $\dot r=-k(0,\dot b)\dot b$；
2. 由碰撞律 (140)，碰后球速
   $$
   \dot b^+=(1+e)(-k\dot b)-e\dot b
   =-\big[(1+e)k+e\big]\dot b; \tag{146}
   $$
3. 球在空中飞行时能量守恒，回到 $b=0$ 时速度大小不变、方向反转，所以下一次碰撞前的速度大小就是 $|\dot b^+|$；
4. 把 (143) 在 $b=0$ 处代入。此时 $E=\frac12m\dot b^2$，取 $E_d=\frac12m\dot b_d^2$，则
   $$
   k(0,\dot b)=\kappa_0+\frac{\kappa_1}{2}\big(\dot b_d^2-\dot b^2\big). \tag{147}
   $$
   为使 $\dot b_d$ 成为固定点，只需选择 $\kappa_0$ 满足
   $$
   (1+e)\kappa_0+e=1, \tag{148}
   $$
   于是碰撞后的速度倍率为
   $$
   (1+e)k+e
   =1-\frac{1+e}{2}\kappa_1\big(\dot b^2-\dot b_d^2\big), \tag{149}
   $$
   整理即得 (145)。

**固定点**：

$$
\dot b^*=\dot b_d. \tag{150}
$$

**稳定性**：令

$$
f(v)=v\left[1-\beta(v^2-v_d^2)\right],
\qquad
\beta=\frac{1+e}{2}\kappa_1. \tag{151}
$$

求导：

$$
f'(v)=1-\beta(v^2-v_d^2)-2\beta v^2
=1+\beta v_d^2-3\beta v^2. \tag{152}
$$

在 $v=v_d$ 处：

$$
f'(v_d)=1-2\beta v_d^2. \tag{153}
$$

稳定条件 $|f'(v_d)|<1$：

$$
\boxed{
0<\frac{1+e}{2}\kappa_1\dot b_d^2<1
}. \tag{154}
$$

- 若 $0<\beta v_d^2<\frac12$，$f'(v_d)\in(0,1)$，单调收敛；
- 若 $\frac12<\beta v_d^2<1$，$f'(v_d)\in(-1,0)$，振荡收敛；
- 若 $\beta v_d^2>1$，固定点不稳定，映射可能出现周期 2 或混沌。

**全局行为**：$f(v)$ 是一个三次型映射，除了局部稳定条件外，还存在一个吸引域；只要初值落在域内，就会收敛到 $\dot b_d$。教材指出这对所有相关初值都成立（在一个有界区间内）。

> **与腿式机器人的联系**：杂耍和行走的数学结构完全同构——
> - 连续飞行/支撑相 ↔ 连续动力学；
> - 碰撞 ↔ 脚跟着地；
> - 镜像律 ↔ 落脚点策略；
> - 能量反馈 ↔ 行走的能量注入；
> - 一维返回映射 ↔ 步态稳定性。
>
> 这就是教材把杂耍放在本章的原因：**一旦你掌握了"混合动力学 + 庞加莱映射"这套语言，具体是腿还是拍子并不重要。**

> **自检**：由 (153)，当 $\beta v_d^2=1/2$ 时 $f'=0$，收敛最快（一步到位）；当 $\beta v_d^2\to1$ 时 $f'\to-1$，收敛越来越慢并伴随振荡。这与 (154) 的边界完全一致。

---

## 4.11 习题：一维跳跃器（One-Dimensional Hopper）

### 4.11.1 题目目标

实现 Raibert（1984）提出的一维跳跃控制器：通过控制弹簧的**预压缩量（preload）**，让跳跃器形成稳定的共振振荡，并跳到指定高度。

习题要求写两个部分：

1. 给定状态，返回系统的总机械能；
2. 跳跃控制器（Drake 的 `LeafSystem`），每个采样时刻读取状态，输出弹簧预载，使下一次跳跃达到期望高度。

### 4.11.2 一维跳跃器模型

设质量 $m$ 只能沿竖直方向运动，高度为 $z$；腿是刚度为 $k$、原长为 $l_0$ 的弹簧。分两个阶段：

**支撑相**（脚在地面，$z\le l_0$）：

$$
m\ddot z=k(l_0-z)-mg. \tag{155}
$$

**飞行相**（脚离地）：

$$
m\ddot z=-mg. \tag{156}
$$

切换条件：

- 支撑 → 飞行：$z=l_0$ 且 $\dot z>0$（弹簧回到原长并继续伸长）；
- 飞行 → 支撑：$z=l_0$ 且 $\dot z<0$（脚再次触地）。

**总机械能**：

支撑相：

$$
E=\frac12m\dot z^2+mgz+\frac12k(l_0-z)^2. \tag{157}
$$

飞行相：

$$
E=\frac12m\dot z^2+mgz. \tag{158}
$$

在飞行相中 $E$ 守恒；在支撑相中，如果弹簧是理想的、没有主动做功，$E$ 也守恒。**预载**的作用是在触地瞬间改变弹簧的势能，从而改变总能量。

### 4.11.3 能量控制器的推导 [笔记]

设期望的跳跃最高点为 $h$。在最高点 $\dot z=0$，弹簧未压缩，所以

$$
E_d=mgh. \tag{159}
$$

设触地瞬间高度 $z=l_0$、速度为 $\dot z^-$，弹簧预压缩量为 $\Delta$（即触地时弹簧已经被压缩了 $\Delta$）。触地后的总能量：

$$
E^+=\frac12m(\dot z^-)^2+mg l_0+\frac12k\Delta^2. \tag{160}
$$

希望这次跳跃恰好达到 $h$，则

$$
E^+=E_d=mgh. \tag{161}
$$

解出所需预压缩量：

$$
\boxed{
\Delta^2=\frac{2}{k}\left[mgh-\frac12m(\dot z^-)^2-mg l_0\right]
=\frac{m}{k}\left[2g(h-l_0)-(\dot z^-)^2\right]
}. \tag{162}
$$

若括号内为负，说明当前下落速度已经携带了过多能量，仅靠"增加预载"无法降到目标高度，需要制动或改变策略（例如减小弹簧刚度、使用离合器）。

> **模型约定**：这里把预载理想化为"在触地高度 $z=l_0$ 处注入的弹簧势能 $\frac12k\Delta^2$"，忽略了预载本身使腿长缩短 $l_0\to l_0-\Delta$ 带来的高度变化。若你的模型显式包含这一项，应把 (160) 中的 $mg l_0$ 换成 $mg(l_0-\Delta)$，解二次方程
> $$
> k\Delta^2-2mg\Delta+m\dot z^2-2mg(h-l_0)=0, \tag{163}
> $$
> 取物理上较小的正根；在小预载极限下，它回到 (162) 的近似。

**物理解读**：

- 跳跃高度完全由总能量决定；
- 触地速度 $\dot z^-$ 越大，需要的预载越小；
- 预载把"触地时的动能 + 重力势能"补到"目标高度的势能"；
- 如果每步都精确执行 (162)，系统在**一步内**达到目标高度——这又是一个无差拍（deadbeat）思想。

### 4.11.4 Drake 实现要点

控制器是一个 `LeafSystem`：

```python
class HoppingController(LeafSystem):
    def __init__(self, m, k, l0, g, h_desired):
        super().__init__()
        self.DeclareVectorInputPort("state", 2)      # [z, zdot]
        self.DeclareVectorOutputPort("preload", 1, self.CalcOutput)
        # 保存参数...

    def CalcOutput(self, context, output):
        z, zdot = self.get_input_port(0).Eval(context)
        # 只在触地时更新预载；飞行相保持上一次的值
        # 按 (162) 计算 Delta，饱和到 [0, Delta_max]
        ...
        output.SetFromVector([delta])
```

实现时要注意：

1. **触发时机**：预载应在触地瞬间设置，而不是每个采样时刻都改；否则会把连续动力学变成"每步都在注入/抽取能量"的时变系统。
2. **饱和**：$\Delta$ 必须限制在物理可行范围内（弹簧不能拉伸到负压缩）。
3. **能量函数**：习题第一部分要求写的能量函数应与 (157)–(158) 一致，注意分段。
4. **数值事件检测**：支撑/飞行切换要用零穿越检测（witness function），否则触地时刻不准确，能量会漂移。

### 4.11.5 与本章主线的联系

这道题是整章的缩影：

- **能量分析** → 找周期轨（§4.4、§4.8）；
- **离散决策** → 每步一次预载（§4.9）；
- **无差拍** → 一步达到目标高度；
- **hybrid 系统** → 支撑/飞行切换 + 事件检测。

> **自检**：
> 1. 在飞行相，$E=mg h$ 应保持不变；到最高点时 $\dot z=0$、$z=h$。
> 2. 由 (162) 算出的 $\Delta$ 代入 (160)，应精确得到 $E=mgh$。
> 3. 若 $\dot z^-=0$，则 $\Delta=\sqrt{2m g(h-l_0)/k}$，这是"从静止压缩弹簧弹到高度 $h$"的经典结果。
> 4. 若 $h=l_0$，则 $\Delta=0$（只要 $\dot z^-=0$），跳跃器刚好到达弹簧原长，不腾空。

---

## 附录 A：数值验证脚本

以下脚本验证了本笔记中的关键结论：

- Van der Pol 返回映射的固定点与斜率；
- 无轮缘轮的阈值、滚动固定点、稳定性导数与分岔临界角；
- 罗盘步态的 $\mathbf{M},\mathbf{C},\boldsymbol{\tau}_g$ 符号推导，以及碰撞投影；
- SLIP 的拉格朗日方程、能量守恒、apex-to-apex 映射与固定点；
- 杂耍映射的固定点稳定性判据。

```python
import numpy as np
import sympy as sp
from scipy.integrate import solve_ivp
from scipy.optimize import brentq

# ============================================================
# 1. Van der Pol: Poincare map fixed point and slope
# ============================================================
def vdp_check(mu=0.2):
    def f(t, y):
        q, v = y
        return [v, -q - mu * (q*q - 1) * v]
    def hit(t, y):
        return y[0]
    hit.terminal = False
    hit.direction = 1
    def P(v0):
        sol = solve_ivp(f, [0, 20], [0.0, v0], events=hit,
                        rtol=1e-11, atol=1e-13)
        for t, y in zip(sol.t_events[0], sol.y_events[0]):
            if t > 1e-6:
                return y[1]
        return np.nan
    vstar = brentq(lambda v: P(v) - v, 0.4, 3.0)
    h = 1e-5
    slope = (P(vstar + h) - P(vstar - h)) / (2 * h)
    return vstar, slope

# ============================================================
# 2. Rimless wheel: thresholds, fixed point, stability, bifurcation
# ============================================================
def rimless_check(g=9.8, l=1.0, alpha=np.pi/8):
    rows = []
    D = lambda gam: 4 * g / l * np.sin(alpha) * np.sin(gam)
    w1 = lambda gam: np.sqrt(max(0.0, 2 * g / l * (1 - np.cos(gam - alpha))))
    w2 = lambda gam: -np.sqrt(2 * g / l * (1 - np.cos(alpha + gam)))
    wr = lambda gam: np.cos(2*alpha) / np.sin(2*alpha) * np.sqrt(D(gam))
    for gam in [0.05, 0.08, 0.15, 0.50]:
        rows.append((gam, w1(gam), w2(gam), wr(gam),
                     wr(gam) > w1(gam), gam < alpha))
    c = np.cos(2*alpha)
    gam = 0.15
    w = wr(gam)
    dP = c * w / np.sqrt(w*w + D(gam))
    fcrit = lambda gam: wr(gam) - w1(gam)
    gcrit = brentq(fcrit, 1e-4, alpha - 1e-4)
    return rows, dP, gcrit

# ============================================================
# 3. Compass gait: symbolic M, C, tau_g and collision projection
# ============================================================
def compass_check():
    t = sp.symbols('t')
    m, mh, a, b, l, g = sp.symbols('m m_h a b l g', positive=True)
    ths = sp.Function('ths')(t); thw = sp.Function('thw')(t)
    q = sp.Matrix([ths, thw]); qd = q.diff(t)
    sst, cst = sp.sin(ths), sp.cos(ths)
    ssw, csw = sp.sin(thw), sp.cos(thw)
    # mirror convention matching the book
    ph = l*sp.Matrix([sst, cst])
    pst = a*sp.Matrix([sst, cst])
    psw = ph - b*sp.Matrix([ssw, csw])
    T = sum(sp.Rational(1,2)*mass*(p.diff(t).T*p.diff(t))[0]
            for mass, p in [(mh, ph), (m, pst), (m, psw)])
    U = g*(mh*ph[1] + m*pst[1] + m*psw[1])
    L = T - U
    eom = [sp.expand(sp.diff(sp.diff(L, qd[i]), t) - sp.diff(L, q[i]))
           for i in range(2)]
    qdd = [ths.diff(t,2), thw.diff(t,2)]
    M = sp.Matrix(2, 2, lambda i,j: sp.simplify(eom[i].coeff(qdd[j])))
    taug = -sp.Matrix([sp.simplify(sp.diff(U, q[i])) for i in range(2)])
    C = sp.zeros(2, 2)
    for i in range(2):
        for j in range(2):
            C[i,j] = sum(sp.Rational(1,2)*(sp.diff(M[i,j], q[k])
                        + sp.diff(M[i,k], q[j]) - sp.diff(M[j,k], q[i]))*qd[k]
                        for k in range(2))
    C = sp.Matrix(2, 2, lambda i,j: sp.simplify(C[i,j]))

    Mref = sp.Matrix([[(mh+m)*l**2 + m*a**2, -m*l*b*sp.cos(ths-thw)],
                      [-m*l*b*sp.cos(ths-thw), m*b**2]])
    Cref = sp.Matrix([[0, -m*l*b*sp.sin(ths-thw)*qd[1]],
                      [m*l*b*sp.sin(ths-thw)*qd[0], 0]])
    tref = sp.Matrix([((mh+m)*l + m*a)*g*sp.sin(ths),
                      -m*b*g*sp.sin(thw)])
    err = [sp.simplify(M-Mref), sp.simplify(C-Cref), sp.simplify(taug-tref)]

    # collision projection check
    x, y = sp.symbols('x y', real=True)
    qfb = sp.Matrix([x, y, ths, thw])
    ph2 = sp.Matrix([x + l*sp.sin(ths), y + l*sp.cos(ths)])
    pst2 = sp.Matrix([x + a*sp.sin(ths), y + a*sp.cos(ths)])
    psw2 = ph2 - b*sp.Matrix([sp.sin(thw), sp.cos(thw)])
    def Jac(p):
        return sp.Matrix([[sp.diff(pi, qj) for qj in qfb] for pi in p])
    Jh, Jst, Jsw = Jac(ph2), Jac(pst2), Jac(psw2)
    Mfb = sp.simplify(mh*Jh.T*Jh + m*Jst.T*Jst + m*Jsw.T*Jsw)
    J = Jac(psw2)
    subs = {m:5, mh:10, a:0.5, b:0.5, l:1.0, x:0, y:0, ths:0.2, thw:-0.3}
    M0 = np.array(sp.Matrix(Mfb.subs(subs)).tolist(), float)
    J0 = np.array(sp.Matrix(J.subs(subs)).tolist(), float)
    qdm = np.array([0.0, 0.0, 1.2, -2.0])
    Proj = (np.eye(4) - np.linalg.inv(M0) @ J0.T
            @ np.linalg.inv(J0 @ np.linalg.inv(M0) @ J0.T) @ J0)
    qdp = Proj @ qdm
    foot_after = float(np.linalg.norm(J0 @ qdp))
    dp = M0 @ (qdp - qdm)
    rank_check = np.linalg.matrix_rank(np.column_stack([dp, J0.T]))
    return err, foot_after, rank_check

# ============================================================
# 4. SLIP: symbolic equations + apex map simulation
# ============================================================
def slip_symbolic():
    t = sp.symbols('t')
    m, k, l0, g = sp.symbols('m k l0 g', positive=True)
    r = sp.Function('r')(t); th = sp.Function('th')(t)
    x = -r*sp.sin(th); z = r*sp.cos(th)
    T = sp.Rational(1,2)*m*(x.diff(t)**2 + z.diff(t)**2)
    U = m*g*z + sp.Rational(1,2)*k*(l0-r)**2
    L = T - U
    Er = [sp.simplify(sp.diff(sp.diff(L, r.diff(t)), t) - sp.diff(L, r)),
          sp.simplify(sp.diff(sp.diff(L, th.diff(t)), t) - sp.diff(L, th))]
    book = [m*r.diff(t,2) - m*r*th.diff(t)**2 + m*g*sp.cos(th) - k*(l0-r),
            m*r**2*th.diff(t,2) + 2*m*r*r.diff(t)*th.diff(t)
            - m*g*r*sp.sin(th)]
    err = [sp.simplify(Er[i]-book[i]) for i in range(2)]
    E = T + U
    rdd = sp.solve(Er[0], r.diff(t,2))[0]
    thdd = sp.solve(Er[1], th.diff(t,2))[0]
    dE = sp.simplify(sp.diff(E, t).subs({r.diff(t,2): rdd,
                                         th.diff(t,2): thdd}))
    return err, dE

def slip_apex_check():
    m, l0, g = 1.0, 1.0, 9.81
    kt, Et = 10.7, 1.61
    k = kt*m*g/l0
    E0 = Et*m*g*l0
    th_td = np.deg2rad(30.0)

    def flight(t, y):
        return [y[2], y[3], 0.0, -g]
    def flight_hit(t, y):
        return y[1] - l0*np.cos(th_td)
    flight_hit.terminal = True; flight_hit.direction = -1

    def stance(t, y):
        r, th, rd, thd = y
        rdd = (m*r*thd**2 - m*g*np.cos(th) + k*(l0-r))/m
        thdd = (m*g*r*np.sin(th) - 2*m*r*rd*thd)/(m*r**2)
        return [rd, thd, rdd, thdd]
    def liftoff(t, y):
        return y[0] - l0
    liftoff.terminal = True; liftoff.direction = 1

    def P(z0):
        vx2 = 2*(E0/m - g*z0)
        if vx2 <= 0:
            return None
        vx = np.sqrt(vx2)
        sol = solve_ivp(flight, [0, 5], [0.0, z0, vx, 0.0],
                        events=flight_hit, rtol=1e-10, atol=1e-12)
        if len(sol.t_events[0]) == 0:
            return None
        x, z, vx, vz = sol.y[:, -1]
        xf = x + l0*np.sin(th_td)
        r, th = l0, th_td
        rd = -vx*np.sin(th) + vz*np.cos(th)
        thd = -(vx*np.cos(th) + vz*np.sin(th))/r
        sol2 = solve_ivp(stance, [0, 3], [r, th, rd, thd],
                         events=liftoff, rtol=1e-10, atol=1e-12)
        if len(sol2.t_events[0]) == 0:
            return None
        r, th, rd, thd = sol2.y[:, -1]
        x = xf - r*np.sin(th); z = r*np.cos(th)
        vx = -rd*np.sin(th) - r*np.cos(th)*thd
        vz = rd*np.cos(th) - r*np.sin(th)*thd
        def apex(t, y):
            return y[3]
        apex.terminal = True; apex.direction = -1
        sol3 = solve_ivp(flight, [0, 5], [x, z, vx, vz],
                         events=apex, rtol=1e-10, atol=1e-12)
        if len(sol3.t_events[0]) == 0:
            return None
        x, z, vx, vz = sol3.y[:, -1]
        return z, 0.5*m*vx**2 + m*g*z

    roots = []
    for lo, hi in [(0.867, 0.88), (0.90, 0.93)]:
        roots.append(brentq(lambda z: P(z)[0] - z, lo, hi))
    slopes = []
    for z in roots:
        h = 1e-5
        slopes.append((P(z+h)[0] - P(z-h)[0])/(2*h))
    table = [(z, P(z)[0]) for z in np.arange(0.868, 0.949, 0.010)]
    energies = [P(z)[1] for z in np.arange(0.868, 0.949, 0.010)]
    return roots, slopes, table, max(energies)-min(energies)

# ============================================================
# 5. Juggler map derivative
# ============================================================
def juggler_check():
    v, vd, beta = sp.symbols('v vd beta', positive=True)
    f = v*(1 - beta*(v**2 - vd**2))
    return sp.simplify(sp.diff(f, v).subs(v, vd))

if __name__ == '__main__':
    print("=== 1. Van der Pol ===")
    vstar, slope = vdp_check()
    print(f"fixed point v*={vstar:.6f}, P'(v*)={slope:.6f}")

    print("\n=== 2. Rimless wheel ===")
    rows, dP, gcrit = rimless_check()
    print("gamma   w1      w2      w_roll   roll_valid stand_valid")
    for r in rows:
        print(f"{r[0]:.2f}  {r[1]:.4f}  {r[2]:+.4f}  {r[3]:.4f}"
              f"   {r[4]!s:5s}      {r[5]!s:5s}")
    print(f"P'(w*)={dP:.4f} (cos^2(2a)={np.cos(np.pi/4)**2:.4f})")
    print(f"critical gamma={gcrit:.6f} rad = {np.rad2deg(gcrit):.4f} deg")

    print("\n=== 3. Compass gait ===")
    err, foot_after, rank_check = compass_check()
    print("M-Mref zero:", err[0].is_zero_matrix)
    print("C-Cref zero:", err[1].is_zero_matrix)
    print("taug-taug_ref zero:", err[2].is_zero_matrix)
    print(f"post-impact swing-foot speed = {foot_after:.3e}")
    print(f"rank([M dq, J^T]) = {rank_check} (should be 2)")

    print("\n=== 4. SLIP ===")
    err, dE = slip_symbolic()
    print("SLIP EOM differences:", [e == 0 for e in err])
    print("dE/dt =", dE)
    roots, slopes, table, e_spread = slip_apex_check()
    for z, dz in table:
        print(f"z={z:.3f} -> {dz:.4f}")
    print(f"fixed points: {roots[0]:.6f} (slope {slopes[0]:.4f}), "
          f"{roots[1]:.6f} (slope {slopes[1]:.4f})")
    print(f"max energy spread at apex = {e_spread:.3e}")

    print("\n=== 5. Juggler ===")
    print("f'(vd) =", juggler_check())
```

**运行结果（要点）**：

```text
=== 1. Van der Pol ===
fixed point v*=2.007079, P'(v*)=0.282827

=== 2. Rimless wheel ===
gamma   w1      w2      w_roll   roll_valid stand_valid
0.05  1.0676  -1.3746  0.8659   False      True
0.08  0.9749  -1.4660  1.0949   True       True
0.15  0.7579  -1.6781  1.4972   True       True
0.50  0.3357  -2.7027  2.6818   True       False
P'(w*)=0.5000 (cos^2(2a)=0.5000)
critical gamma=0.068229 rad = 3.9093 deg

=== 3. Compass gait ===
M-Mref zero: True
C-Cref zero: True
taug-taug_ref zero: True
post-impact swing-foot speed = 5.594e-16
rank([M dq, J^T]) = 2 (should be 2)

=== 4. SLIP ===
SLIP EOM differences: [True, True]
dE/dt = 0
z=0.868 -> 0.8757
z=0.878 -> 0.8774
z=0.888 -> 0.8845
z=0.898 -> 0.8941
z=0.908 -> 0.9055
z=0.918 -> 0.9184
z=0.928 -> 0.9327
z=0.938 -> 0.9483
z=0.948 -> 0.9651
fixed points: 0.876815 (slope 0.4880), 0.916831 (slope 1.3462)
max energy spread at apex = 9.063e-10

=== 5. Juggler ===
f'(vd) = 1 - 2*beta*vd**2
```

**结果解读**：

- Van der Pol 的固定点斜率 $0.283<1$，极限环稳定；
- 无轮缘轮 $P'=0.5=\cos^2(\pi/4)$，与解析式完全一致；
- 罗盘步态的 $\mathbf{M},\mathbf{C},\boldsymbol{\tau}_g$ 与书中矩阵逐项为零；
- 碰撞投影后摆动脚速度约 $10^{-16}$（数值零），动量变化落在 $\mathbf{J}^\top$ 的列空间；
- SLIP 能量在 $10^{-9}$ 量级内守恒（数值积分误差），稳定/不稳定固定点分别为 $0.8768$ 与 $0.9168$；
- 杂耍映射的导数给出稳定性条件 $0<\beta v_d^2<1$。

---

## 附录 B：公式速查

| 内容 | 公式 |
|---|---|
| Van der Pol | $\ddot q+\mu(q^2-1)\dot q+q=0$；$\dot E=\mu(1-q^2)\dot q^2$ |
| 轨道稳定性 | $d(x,\Gamma)=\min_\tau\|x-x^*(\tau)\|$ |
| 庞加莱映射 | $x_p[n+1]=P(x_p[n])$；固定点 $\leftrightarrow$ 周期轨 |
| 周期轨稳定性 | $\mathbf{A}_p=DP(x_p^*)$，所有 $\|\lambda_i\|<1$ |
| 无轮缘轮支撑相 | $\ddot\theta=(g/l)\sin\theta$；$E=\frac12ml^2\dot\theta^2+mgl\cos\theta$ |
| 跨步阈值 | $\omega_1=\sqrt{2(g/l)[1-\cos(\gamma-\alpha)]}$ |
| 触地前速度 | $(\dot\theta^-)^2=\omega_0^2+4(g/l)\sin\alpha\sin\gamma$ |
| 碰撞 | $\dot\theta^+=\dot\theta^-\cos(2\alpha)$；动能保留 $\cos^2(2\alpha)$ |
| 返回映射 | $\dot\theta^+_{n+1}=\cos(2\alpha)\sqrt{(\dot\theta_n^+)^2+4(g/l)\sin\alpha\sin\gamma}$ |
| 反向阈值 | $\omega_2=-\sqrt{2(g/l)[1-\cos(\alpha+\gamma)]}$ |
| 滚动固定点 | $\omega^*_{roll}=\cot(2\alpha)\sqrt{4(g/l)\sin\alpha\sin\gamma}$ |
| 固定点稳定性 | $P'(\omega^*)=\cos^2(2\alpha)<1$ |
| 罗盘步态质量矩阵 | $M_{11}=(m_h+m)l^2+ma^2$，$M_{12}=-mlb\cos\Delta$，$M_{22}=mb^2$ |
| 罗盘步态科氏项 | $C_{12}=-mlb\sin\Delta\,\dot\theta_{sw}$，$C_{21}=mlb\sin\Delta\,\dot\theta_{st}$ |
| 罗盘步态重力项 | $\tau_{g,st}=[(m_h+m)l+ma]g\sin\theta_{st}$，$\tau_{g,sw}=-mbg\sin\theta_{sw}$ |
| 髋关节输入 | $\mathbf{B}=[-1,1]^\top$ |
| 碰撞投影 | $\dot q^+=[I-M_{fb}^{-1}J^\top(JM_{fb}^{-1}J^\top)^{-1}J]\dot q^-$ |
| SLIP 支撑相 | $m\ddot r-mr\dot\theta^2+mg\cos\theta-k(l_0-r)=0$；$mr^2\ddot\theta+2mr\dot r\dot\theta-mgr\sin\theta=0$ |
| SLIP 能量 | $E=\frac12m(\dot r^2+r^2\dot\theta^2)+mgr\cos\theta+\frac12k(l_0-r)^2$ |
| SLIP 切换 | 触地 $z-l_0\cos\theta\le0$；离地 $r\ge l_0$ |
| 无量纲量 | $\tilde E=E/(mgl_0)$，$\tilde k=kl_0/(mg)$ |
| SLIP 固定点 | $z^*\approx0.8768$（稳定），$z^*\approx0.9168$（不稳定） |
| 无差拍 | $\delta u=-a/b\,\delta z$ |
| 杂耍镜像律 | $r=-k(b,\dot b)b$；$k=\kappa_0+(\kappa_1/m)(E_d-E)$ |
| 杂耍返回映射 | $\dot b[n+1]=\dot b[n][1-\frac{1+e}{2}\kappa_1(\dot b^2[n]-\dot b_d^2)]$ |
| 杂耍稳定性 | $f'(v_d)=1-2\beta v_d^2$，$0<\beta v_d^2<1$ |

---

## 附录 C：本章最容易错的 10 个点

1. **把轨迹稳定性当轨道稳定性**。走路允许相位漂移，不能用 $\|x(t)-x^*(t)\|\to0$；要用到轨道的最近距离。
2. **混淆返回映射与单值矩阵的特征值**。返回映射沿周期轨方向是**零特征值**；单值矩阵（一个完整周期的流雅可比）沿该方向是 **1**。两者不是同一个对象。
3. **无轮缘轮的 $\theta$ 零位/正方向搞错**。本章约定 $\theta=0$ 在竖直向上、顺时针为正；换坐标后 $\omega_1,\omega_2$ 的符号会变。
4. **碰撞时错误地使用能量守恒**。脚跟着地是非弹性碰撞，只能对**碰撞点**用角动量守恒；能量按 $\cos^2(2\alpha)$ 损失。
5. **忽略了碰撞损失因子 $\cos(2\alpha)$**。直接把触地前速度当成下一步初速度，会高估速度、错判稳定性。
6. **把 $\omega^*_{roll}$ 当成永远有效**。它必须是合法固定点，即 $\omega^*_{roll}>\omega_1$；否则轮子走不到下一步。
7. **忘记 $\gamma>\alpha$ 时站立固定点消失**。此时质心永远在支撑多边形外，只有滚动解。
8. **罗盘步态的镜像符号混用**。$M_{12}$、$C$、$\tau_g$ 必须来自同一套坐标约定；混用会让能量不守恒或出现虚假的不稳定。
9. **碰撞投影忘记浮动基座或忘记交换腿**。必须先加 $(x,y)$ 浮动基座、用 $\mathbf{J}$ 投影，再交换支撑/摆动角色；顺序错了，碰撞后的速度没有物理意义。
10. **把 SLIP 的 apex map 稳定当成全稳定**。SLIP 无耗散，apex map 只在能量恒定的切片上成立，是**部分稳定性**；它不能保证对地形高度/能量扰动的鲁棒性。

> **附带的两个小陷阱**：
> - SLIP 触地**没有速度跳变**（腿无质量），不要套用行走的碰撞公式；
> - 膝盖步行者的离散事件顺序是 **knee-strike → heel-strike**，且每次事件后都要切换动力学模型与质量矩阵。

---

## 附录 D：与 Ch. 3 的衔接

| Ch. 3 | Ch. 4 |
|---|---|
| 固定点稳定性 | 周期轨稳定性 |
| 线性化 $A_{\text{lin}},B_{\text{lin}}$ | 返回映射线性化 $DP$ |
| 能控性/Kalman 秩 | 返回映射的可逆性/死拍可控性 |
| LQR（连续时间） | 离散 LQR / deadbeat |
| 能量整形（swing-up） | 能量平衡（重力 vs 碰撞） |
| 接触尚未出现 | 接触 + 混合动力学 |

**一句话总结**：第 3 章教你如何把系统稳定到一个**点**；第 4 章教你如何把系统稳定到一条**轨道**。工具从"连续线性化"换成了"庞加莱映射 + 离散线性化"，但核心思想完全一致：**在平衡对象附近线性化，看特征值是否在稳定域内**。

