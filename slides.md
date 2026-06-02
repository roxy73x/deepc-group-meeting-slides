---
marp: true
paginate: true
size: 16:9
math: mathjax
html: true
---

<style>
section {
  font-family: "Microsoft YaHei", "Noto Sans SC", Arial, sans-serif;
  background: #ffffff;
  color: #0f172a;
  padding: 54px 72px 48px 72px;
  font-size: 26px;
  line-height: 1.35;
}
section::before { content: ""; position: absolute; left: 0; top: 0; width: 12px; height: 100%; background: #2563eb; }
h1 { color: #1e40af; font-size: 44px; margin: 0 0 24px 0; }
h2 { color: #1e40af; font-size: 32px; margin: 0 0 16px 0; }
h3 { color: #1e40af; font-size: 27px; margin: 0 0 10px 0; }
li { margin: 8px 0; }
strong { color: #1e40af; }
.small { font-size: 20px; color: #64748b; }
.note { border-left: 6px solid #2563eb; background: #eff6ff; padding: 14px 18px; border-radius: 8px; margin-top: 18px; color: #334155; }
.cols { display: grid; grid-template-columns: 1fr 1fr; gap: 28px; }
.cols3 { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 18px; }
.box { border: 1px solid #dbeafe; border-radius: 14px; padding: 18px 20px; background: #ffffff; }
.bluebox { background: #eff6ff; }
.tbl { width: 100%; border-collapse: collapse; font-size: 23px; }
.tbl.compact { font-size: 18px; }
.tbl th { background: #eff6ff; color: #1e40af; }
.tbl th, .tbl td { border: 1px solid #bfdbfe; padding: 11px 12px; text-align: center; vertical-align: middle; }
.tbl.compact th, .tbl.compact td { padding: 8px 9px; }
.good { color: #16a34a; font-weight: 700; }
.eq { font-family: "Times New Roman", "Cambria Math", serif; font-size: 34px; text-align: center; color: #1e40af; margin: 18px 0; }
.ref { font-size: 16px; color: #64748b; margin-top: 10px; }
img { max-width: 100%; max-height: 430px; object-fit: contain; }
</style>

<!-- _class: title -->

# Phase-aware DeePC for Quadcopter Tracking

面向多机动轨迹的无人机数据驱动预测控制  
**选题、实验进展与关键难点**

<div class="small" style="margin-top:60px;">
组会汇报 · DeePC / UAV Tracking / Gazebo Validation
</div>

---

# 1. 从 MPC 到 DeePC：基本形式与优化问题

DeePC 可以理解为对传统 MPC 的一种数据驱动改写：**MPC 用显式模型预测未来，DeePC 用历史输入输出数据张成未来轨迹。**

<div class="cols">
<div>

**MPC**

<div class="eq">
x<sub>k+1</sub> = Ax<sub>k</sub> + Bu<sub>k</sub>
</div>

基于模型预测未来状态，并求解：

<div class="eq">
min Σ ||y<sub>k</sub> − r<sub>k</sub>||<sub>Q</sub><sup>2</sup> + Σ ||u<sub>k</sub>||<sub>R</sub><sup>2</sup>
</div>

</div>
<div>

**DeePC**

<div class="eq">
[U<sub>p</sub>;Y<sub>p</sub>;U<sub>f</sub>;Y<sub>f</sub>]g = [u<sub>ini</sub>;y<sub>ini</sub>;u;y]
</div>

用数据矩阵隐式表示系统行为，并求解：

<div class="eq">
min Σ ||y<sub>k</sub> − r<sub>k</sub>||<sub>Q</sub><sup>2</sup> + Σ ||u<sub>k</sub>||<sub>R</sub><sup>2</sup> + λ<sub>g</sub>||g||<sup>2</sup> + λ<sub>y</sub>||σ<sub>y</sub>||<sup>2</sup>
</div>

</div>
</div>

<div class="note">
原始 DeePC 工作给出了数据驱动 MPC 的基本框架。后续工作进一步关注：非线性系统中如何选择数据、如何根据工作区域切换局部 DeePC 表示。
</div>

---

# 2. 参考工作一：Select-DeePC / Online Data Selection

<div class="cols">
<div>

## Choose Wisely, 2025

**核心思想**

- 面向非线性系统，标准 DeePC 的全局数据矩阵可能包含大量无关数据
- 每个控制时刻只选择最相关的数据列
- 用局部相关数据在 trajectory space 中隐式线性化当前系统行为
- 文中验证了 norm-based 与 manifold-embedding-based selection

</div>
<div>

## 对本项目的启发

**不是所有飞行数据都同等有用**

- smooth tracking 需要平滑稳定数据
- transition 需要包含转弯、加减速的数据
- step-like 需要覆盖目标突变后的恢复过程

<div class="eq">
D_k = D(φ_k)
</div>

这自然引出 **phase-dependent local data selection**。

</div>
</div>

<div class="ref">
Ref: Näf, Moffat, Eising, Dörfler, “Choose Wisely: Data-Enabled Predictive Control for Nonlinear Systems Using Online Data Selection”, 2025.
</div>

---

# 3. 参考工作二：Gain-Scheduling DeePC

<div class="cols">
<div>

## GS-DeePC, 2025

**核心思想**

- 非线性系统存在明显 operating-region dependence
- 不使用一个全局 Hankel 矩阵
- 按可测 scheduling variable 划分局部工作区域
- 每个区域构造局部 Hankel 数据表示
- 通过区域切换处理非线性系统控制

</div>
<div>

## 对本项目的启发

**无人机也有类似“工作区域/机动阶段”**

- smooth：稳定跟踪，控制输入应更平滑
- transition：曲率或速度方向变化，预测误差容易放大
- step-like：目标突变，重点是减少超调和恢复时间

<div class="eq">
(λ_g, λ_y)_k = (λ_g, λ_y)(φ_k)
</div>

这对应 **maneuver-aware parameter scheduling**。

</div>
</div>

<div class="ref">
Ref: Zieglmeier et al., “Gain-Scheduling Data-Enabled Predictive Control for Nonlinear Systems with Linearized Operating Regions”, 2025.
</div>

---

# 4. 从已有研究到 UAV tracking 选题

两篇近期工作给出的共同启发：**DeePC 的数据和参数可以随系统状态、任务上下文或工作区域变化。**  
本项目将这一思想迁移到无人机轨迹跟踪中的机动阶段切换。

<div class="cols">
<div>

**已有工作关注**

- Select-DeePC：按当前任务选择相关数据
- GS-DeePC：按工作区域切换局部 DeePC 表示
- 共同目标：提升非线性系统中的预测准确性和优化可行性

</div>
<div>

**本文选题**

> 对于 smooth、transition、step-like 等不同轨迹阶段，是否可以根据当前阶段调整 DeePC 的正则化参数和数据支持，从而提升无人机轨迹跟踪性能？

</div>
</div>

<div class="note">
本项目不是重新提出 DeePC，而是形成 UAV-aware phase-conditioned DeePC：按无人机轨迹阶段定义 phase，并加入 hover-centered input、飞行包线约束和 phase-dependent data interface。
</div>

---

# 5. 方法框架：公式来源与参数调度

相位变量 φ<sub>k</sub> 不是从某篇论文中直接照搬的公式，而是把两篇参考工作的思想映射到 UAV 轨迹跟踪中：

<table class="tbl compact">
<tr><th>来源</th><th>文献中的思想</th><th>本文中的映射</th></tr>
<tr><td>Select-DeePC</td><td>每个时刻根据当前轨迹选择相关数据列</td><td>不同 UAV phase 对应不同数据相关性：D<sub>k</sub>=D(φ<sub>k</sub>)</td></tr>
<tr><td>GS-DeePC</td><td>用 measurable scheduling variable 选择局部 Hankel 表示</td><td>用可在线计算的 trajectory phase 作为 scheduling variable</td></tr>
<tr><td>本文 UAV 设计</td><td>参考轨迹几何与跟踪误差可直接在线获得</td><td>φ<sub>k</sub>=f(r<sub>k</sub>, r<sub>k−1</sub>, e<sub>k</sub>)</td></tr>
</table>

<div class="eq">
φ<sub>k</sub> = f(r<sub>k</sub>, r<sub>k−1</sub>, e<sub>k</sub>) ∈ { smooth, transition, step-like }
</div>

---

# 6. 方法框架：超参数选择理由

当前先验证 phase-conditioned regularization，因此根据 phase 选择 DeePC 正则化参数：

<table class="tbl compact">
<tr><th>Phase</th><th>λ<sub>g</sub></th><th>λ<sub>y</sub></th><th>选择理由</th></tr>
<tr><td>smooth</td><td>30</td><td>1.0e4</td><td>沿用 static baseline；较强 g 正则化抑制过拟合和输入抖动</td></tr>
<tr><td>transition</td><td>20</td><td>1.2e4</td><td>降低 λ<sub>g</sub> 提高响应灵活性；提高 λ<sub>y</sub> 容忍轨迹切换时的数据不匹配</td></tr>
<tr><td>step-like</td><td>12</td><td>1.0e4</td><td>进一步降低 λ<sub>g</sub>，避免目标突变时控制过于保守；λ<sub>y</sub> 保持 baseline 防止误差被过度松弛</td></tr>
</table>

<div class="note">
这组参数不是理论最优解，而是基于 static baseline 和初步参数扫描得到的经验折中。后续需要通过更系统的网格搜索、phase-resolved 指标和消融实验确认泛化性。
</div>

---

# 7. UAV-specific controller adaptations

这一页讲的是**控制器层面的 UAV 适配**：把 DeePC 的输入、约束和安全边界改成更符合四旋翼执行特性的形式。

<div class="cols">
<div>

**1. Hover-centered input**

本文不直接优化绝对控制量，而是优化相对悬停输入的增量：

<div class="eq">
u<sub>k</sub> = u<sub>hover</sub> + Δu<sub>k</sub>
</div>

这样可以把控制量限制在悬停附近，减少无意义的大推力偏置。

</div>
<div>

**2. Flight-envelope constraints**

本文在 DeePC 优化中加入 UAV 执行安全边界：

<div class="eq">
Δu<sub>k</sub> ∈ U<sub>safe</sub>, &nbsp; y<sub>k</sub> ∈ Y<sub>safe</sub>
</div>

约束对象包括输入幅值、输入变化率、速度/加速度上界和轨迹误差边界。

</div>
</div>

<div class="note">
区别：这一页处理的是 controller formulation，即 DeePC 优化变量、输入形式和安全约束如何适配 UAV。
</div>

---

# 8. UAV-specific phase/data design

这一页讲的是**数据与相位层面的 UAV 适配**：把无人机轨迹几何特征显式用于 phase detection、数据组织和指标分析。

1. **Phase-balanced data dictionary**  
   本文按 smooth / transition / step-like 组织飞行数据，使数据集中不仅有平滑飞行，也覆盖转弯、加减速和阶跃恢复过程。

2. **Trajectory-geometry phase detector**  
   本文的相位标签由参考速度、参考加速度、曲率或目标跳变以及跟踪误差共同决定。

<div class="eq">
φ<sub>k</sub> = f(v<sup>ref</sup><sub>k</sub>, a<sup>ref</sup><sub>k</sub>, κ<sup>ref</sup><sub>k</sub>, e<sub>k</sub>)
</div>

3. **Axis-aware weighting / metrics**  
   本文区分 XY 平面误差和 Z 轴高度误差，用于权重设计和结果分析。

<div class="note">
区别：这一页处理的是 data and phase design，即怎么定义 phase、怎么组织数据、怎么分析 UAV tracking 误差。
</div>

---

# 9. 方法图

![width:920px](figures/fig01_method_overview.png)

---

# 10. 已有实验设置

实验采用最终协议 A：

<div class="cols">
<div>

**轨迹任务**

- step
- figure8

**随机种子**

- 41–50，共 10 个 seed

</div>
<div>

**A1: Static DeePC**

- 固定 λ<sub>g</sub>=30
- 固定 λ<sub>y</sub>=1e4

**A2: Maneuver-aware DeePC**

- smooth: 30 / 1e4
- transition: 20 / 1.2e4
- step-like: 12 / 1e4

</div>
</div>

---

# 11. 多 seed 结果

<table class="tbl">
<tr>
<th>Trajectory</th>
<th>Static Position RMSE</th>
<th>Maneuver-aware Position RMSE</th>
<th>Paired Result</th>
</tr>
<tr>
<td><strong>figure8</strong></td>
<td>0.0868 ± 0.0096</td>
<td class="good">0.0647 ± 0.0067</td>
<td class="good">10 / 10 wins</td>
</tr>
<tr>
<td><strong>step</strong></td>
<td>0.1364 ± 0.0234</td>
<td class="good">0.1046 ± 0.0133</td>
<td class="good">10 / 10 wins</td>
</tr>
</table>

---

# 12. 主结果图

![width:920px](figures/fig02_main_results.png)

---

# 13. Phase timeline 图

![width:920px](figures/fig03_phase_timeline.png)

<div class="small">
用途：说明 online phase switching 是实际控制过程中的切换，不是事后重新标注。
</div>

---

# 14. 结果如何解释

当前结果支持的结论：

- 固定参数 DeePC 在不同轨迹类型之间存在折中
- phase-conditioned regularization 能在 step 和 figure8 上稳定降低 RMSE
- 这说明“相位感知”方向有继续做的价值

当前结果还不能直接说明：

- local data selection 一定有效
- phase detector 已经足够可迁移
- Gazebo / ROS 中能保持同样收益

<div class="note">
后续重点不是继续盲目加轨迹，而是把机制解释和验证链路补完整。
</div>

---

# 15. 需要补的评价指标

<div class="cols3">
<div class="box">
<h3>整体指标</h3>
<ul>
<li>Tracking RMSE</li>
<li>Position RMSE</li>
<li>Max error</li>
</ul>
</div>
<div class="box bluebox">
<h3>分阶段指标</h3>
<ul>
<li>Smooth RMSE</li>
<li>Transition RMSE</li>
<li>Step-like RMSE</li>
</ul>
</div>
<div class="box">
<h3>求解指标</h3>
<ul>
<li>Mean solve time</li>
<li>Max solve time</li>
<li>Failure count</li>
</ul>
</div>
</div>

<div class="note">
如果提升主要来自 transition 或 step-like 阶段，才能更有力地支撑 phase-aware 的研究动机。
</div>

---

# 16. 难点一：相位定义不能太启发式

相位标签需要满足：

1. **可解释**：能对应轨迹几何或控制困难
2. **可复现**：不同 seed 和不同轨迹下定义一致
3. **可在线获得**：不能依赖未来信息或离线标签

<div class="eq">
φ<sub>k</sub> = f( ||r<sub>k</sub> − r<sub>k−1</sub>||, ||r<sub>k</sub> − 2r<sub>k−1</sub> + r<sub>k−2</sub>||, ||e<sub>k</sub>|| )
</div>

---

# 17. 难点二：数据越干净不一定越好

DeePC 依赖数据字典覆盖系统行为。采集数据时存在一个矛盾：

- excitation 太强：轨迹看起来脏，可能污染数据
- excitation 太弱：数据缺少充分激励，后续优化容易不可行

<div class="eq">
data quality &nbsp; vs. &nbsp; persistence of excitation
</div>

已有现象：去掉 excitation 后，采集轨迹更干净，但后续 DeePC 执行明显变差，甚至出现 infeasible。

---

# 18. 难点三：horizon 和数据集匹配

预测时域 N 不是单调调参旋钮。

- N 太短：预测能力不足
- N 太长：优化规模增大，数值问题和数据不匹配更明显
- 不同任务对 N 的需求不同

<div class="eq">
N<sub>dataset</sub> = N<sub>controller</sub>
</div>

---

# 19. 难点四：Gazebo / ROS 验证

Gazebo 验证的目标不是刷主结果，而是说明方法能进入更真实的控制链路。

需要注意：

- 使用 /clock，不能用 wall-clock 驱动控制步
- 参考轨迹索引要避免浮点 floor jitter
- 在线历史数据要满足因果关系

<div class="eq">
history should record (u<sub>k−1</sub>, y<sub>k</sub>), not (u<sub>k</sub>, y<sub>k</sub>)
</div>

---

# 20. Gazebo 轨迹预览

![width:860px](figures/gazebo_support384_off4_time_colored_xy.png)

---

# 21. 下一步计划

1. 固定公平 baseline，避免 static DeePC 太弱
2. 补充 phase-resolved RMSE 和 phase occupancy
3. 完成 A1 / A2 / A4 / A5 消融
4. 检查数据采集中的 excitation 强度
5. 做 Gazebo closeout：时间同步、因果记录、控制频率

---

# 22. 总结

- 当前项目不应表述为“复现 DeePC 无人机控制器”
- 更合适的表述是：**面向多机动阶段的 phase-aware UAV DeePC**
- 已有结果显示 A2 在 step 和 figure8 上稳定优于 static DeePC
- 后续关键是解释收益来源，并保证 Gazebo 验证不被时间和数据问题干扰

<div class="note">
希望讨论：后续优先补 A4/A5 消融，还是先完成 Gazebo closeout？
</div>
