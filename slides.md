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
.tbl th { background: #eff6ff; color: #1e40af; }
.tbl th, .tbl td { border: 1px solid #bfdbfe; padding: 11px 12px; text-align: center; }
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

# 2. 为什么固定参数可能不够：近期参考工作

本项目的直接参考工作可以收敛到两条最新方向：

<div class="cols3">
<div class="box bluebox">
<h3>原始 DeePC</h3>
<p>建立 DeePC 与 MPC 的关系，给出输入输出数据驱动的预测控制基本形式。</p>
<p class="small">Coulson, Lygeros, Dörfler, 2019</p>
</div>
<div class="box">
<h3>Select-DeePC</h3>
<p>每个时刻只选择与当前非线性任务最相关的数据列，避免固定全局数据集带来的冗余和失配。</p>
<p class="small">Näf, Moffat, Eising, Dörfler, 2025</p>
</div>
<div class="box bluebox">
<h3>Gain-Scheduling DeePC</h3>
<p>把非线性系统工作区间划分为局部区域，并在线切换局部 Hankel 矩阵或 DeePC 表示。</p>
<p class="small">Zieglmeier et al., 2025</p>
</div>
</div>

<div class="note">
这两篇工作都说明：DeePC 不一定要用固定的全局数据集和固定参数。我的项目进一步把这种“选择/切换”思想落到无人机轨迹阶段上。
</div>

---

# 3. 参考工作一：Select-DeePC / Online Data Selection

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

# 4. 参考工作二：Gain-Scheduling DeePC

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

# 5. 从已有研究到 UAV tracking 的切入点

两篇近期工作给出的共同启发：**DeePC 的数据和参数可以随系统状态、任务上下文或工作区域变化。**

<div class="cols">
<div>

**已有工作关注**

- Select-DeePC：按当前任务选择相关数据
- GS-DeePC：按工作区域切换局部 DeePC 表示
- 共同目标：提升非线性系统中的预测准确性和优化可行性

</div>
<div>

**本项目关注**

- 按无人机轨迹阶段定义 phase
- 按 phase 调整正则化参数
- 后续按 phase 选择数据字典
- 加入 hover-centered input 和飞行包线约束

</div>
</div>

<div class="note">
所以，本项目不是重新提出 DeePC，而是把“在线数据选择”和“增益调度式 DeePC”迁移到 quadrotor tracking 中，形成 UAV-aware phase-conditioned DeePC。
</div>

---

# 6. 当前选题

## Phase-aware / Maneuver-aware UAV DeePC

研究问题：

> 对于包含 smooth tracking、transition、step-like 等不同阶段的无人机轨迹，是否可以根据当前机动阶段调整 DeePC 的正则化或数据使用方式，从而提升跟踪性能？

对应三个层次：

1. **Phase detector**：识别当前处于哪类机动阶段
2. **Phase-conditioned regularization**：不同阶段使用不同 λ<sub>g</sub>, λ<sub>y</sub>
3. **Phase-dependent data selection**：不同阶段使用不同的数据支持集

---

# 7. 方法框架

相位标签由参考轨迹变化和跟踪误差共同决定：

<div class="eq">
φ<sub>k</sub> = f(r<sub>k</sub>, r<sub>k−1</sub>, e<sub>k</sub>) ∈ { smooth, transition, step-like }
</div>

根据相位 φ<sub>k</sub> 选择 DeePC 参数：

<table class="tbl">
<tr><th>Phase</th><th>λ<sub>g</sub></th><th>λ<sub>y</sub></th></tr>
<tr><td>smooth</td><td>30</td><td>1.0e4</td></tr>
<tr><td>transition</td><td>20</td><td>1.2e4</td></tr>
<tr><td>step-like</td><td>12</td><td>1.0e4</td></tr>
</table>

---

# 8. UAV-specific adaptations

在基础 phase-aware DeePC 之上，可以做几类面向四旋翼的特殊适配：

<div class="cols">
<div>

**Hover-centered input**

- 不直接优化绝对控制量，而是优化相对悬停输入的增量

<div class="eq">
u<sub>k</sub> = u<sub>hover</sub> + Δu<sub>k</sub>
</div>

</div>
<div>

**Flight-envelope constraints**

- 对速度、加速度、推力、倾角、输入变化率加约束
- 避免 DeePC 为追踪误差给出过激控制量

<div class="eq">
Δu<sub>k</sub> ∈ U<sub>safe</sub>, &nbsp; y<sub>k</sub> ∈ Y<sub>safe</sub>
</div>

</div>
</div>

<div class="note">
这样做以后，方法就不是简单的 phase-aware DeePC，而是结合了四旋翼悬停平衡点、飞行包线和执行安全约束的 UAV-aware DeePC。
</div>

---

# 9. UAV-specific data and phase design

1. **Phase-balanced data dictionary**  
   数据集中不能只有平滑飞行，还需要覆盖转弯、加减速、阶跃响应等机动片段。

2. **Trajectory-geometry phase detector**  
   相位不只由误差决定，也由参考轨迹的速度、加速度、曲率或目标跳变决定。

<div class="eq">
φ<sub>k</sub> = f(v<sup>ref</sup><sub>k</sub>, a<sup>ref</sup><sub>k</sub>, κ<sup>ref</sup><sub>k</sub>, e<sub>k</sub>)
</div>

3. **Axis-aware weighting**  
   四旋翼的 XY 平面运动和 Z 轴高度控制难度不同，可以对 XY / Z 误差设置不同权重。

<div class="eq">
||y<sub>k</sub> − r<sub>k</sub>||<sub>Q</sub><sup>2</sup> = q<sub>xy</sub> ||e<sub>xy</sub>||<sup>2</sup> + q<sub>z</sub> e<sub>z</sub><sup>2</sup>
</div>

---

# 10. 方法图

![width:920px](figures/fig01_method_overview.png)

---

# 11. 已有实验设置

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

# 12. 多 seed 结果

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

# 13. 主结果图

![width:920px](figures/fig02_main_results.png)

---

# 14. Phase timeline 图

![width:920px](figures/fig03_phase_timeline.png)

<div class="small">
用途：说明 online phase switching 是实际控制过程中的切换，不是事后重新标注。
</div>

---

# 15. 结果如何解释

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

# 16. 需要补的评价指标

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

# 17. 难点一：相位定义不能太启发式

相位标签需要满足：

1. **可解释**：能对应轨迹几何或控制困难
2. **可复现**：不同 seed 和不同轨迹下定义一致
3. **可在线获得**：不能依赖未来信息或离线标签

<div class="eq">
φ<sub>k</sub> = f( ||r<sub>k</sub> − r<sub>k−1</sub>||, ||r<sub>k</sub> − 2r<sub>k−1</sub> + r<sub>k−2</sub>||, ||e<sub>k</sub>|| )
</div>

---

# 18. 难点二：数据越干净不一定越好

DeePC 依赖数据字典覆盖系统行为。采集数据时存在一个矛盾：

- excitation 太强：轨迹看起来脏，可能污染数据
- excitation 太弱：数据缺少充分激励，后续优化容易不可行

<div class="eq">
data quality &nbsp; vs. &nbsp; persistence of excitation
</div>

已有现象：去掉 excitation 后，采集轨迹更干净，但后续 DeePC 执行明显变差，甚至出现 infeasible。

---

# 19. 难点三：horizon 和数据集匹配

预测时域 N 不是单调调参旋钮。

- N 太短：预测能力不足
- N 太长：优化规模增大，数值问题和数据不匹配更明显
- 不同任务对 N 的需求不同

<div class="eq">
N<sub>dataset</sub> = N<sub>controller</sub>
</div>

---

# 20. 难点四：Gazebo / ROS 验证

Gazebo 验证的目标不是刷主结果，而是说明方法能进入更真实的控制链路。

需要注意：

- 使用 /clock，不能用 wall-clock 驱动控制步
- 参考轨迹索引要避免浮点 floor jitter
- 在线历史数据要满足因果关系

<div class="eq">
history should record (u<sub>k−1</sub>, y<sub>k</sub>), not (u<sub>k</sub>, y<sub>k</sub>)
</div>

---

# 21. Gazebo 轨迹预览

![width:860px](figures/gazebo_support384_off4_time_colored_xy.png)

---

# 22. 下一步计划

1. 固定公平 baseline，避免 static DeePC 太弱
2. 补充 phase-resolved RMSE 和 phase occupancy
3. 完成 A1 / A2 / A4 / A5 消融
4. 检查数据采集中的 excitation 强度
5. 做 Gazebo closeout：时间同步、因果记录、控制频率

---

# 23. 总结

- 当前项目不应表述为“复现 DeePC 无人机控制器”
- 更合适的表述是：**面向多机动阶段的 phase-aware UAV DeePC**
- 已有结果显示 A2 在 step 和 figure8 上稳定优于 static DeePC
- 后续关键是解释收益来源，并保证 Gazebo 验证不被时间和数据问题干扰

<div class="note">
希望讨论：后续优先补 A4/A5 消融，还是先完成 Gazebo closeout？
</div>
