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
.warn { border-left-color: #f59e0b; background: #fffbeb; }
.cols { display: grid; grid-template-columns: 1fr 1fr; gap: 28px; }
.cols3 { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 18px; }
.box { border: 1px solid #dbeafe; border-radius: 14px; padding: 18px 20px; background: #ffffff; }
.bluebox { background: #eff6ff; }
.placeholder { border: 2px dashed #93c5fd; border-radius: 14px; background: #f8fbff; padding: 22px; min-height: 180px; display: flex; flex-direction: column; justify-content: center; align-items: center; text-align: center; color: #64748b; }
.tbl { width: 100%; border-collapse: collapse; font-size: 23px; }
.tbl.compact { font-size: 18px; }
.tbl th { background: #eff6ff; color: #1e40af; }
.tbl th, .tbl td { border: 1px solid #bfdbfe; padding: 11px 12px; text-align: center; vertical-align: middle; }
.tbl.compact th, .tbl.compact td { padding: 8px 9px; }
.good { color: #16a34a; font-weight: 700; }
.bad { color: #dc2626; font-weight: 700; }
.eq { font-family: "Times New Roman", "Cambria Math", serif; font-size: 34px; text-align: center; color: #1e40af; margin: 18px 0; }
.eq.small-eq { font-size: 24px; margin: 8px 0; }
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

Select-DeePC 关注的是非线性系统中的 DeePC 控制问题。如果一直使用同一个全局 Hankel 数据集，其中会包含大量和当前控制任务关系不大的数据，可能导致预测质量下降，也会增加优化中的数据失配。也就是**当前控制任务应优先使用当前更相关的数据**。

<div class="note">
该工作来自 DeePC 原作者 Florian Dörfler。
</div>

我先估计当前参考在数据字典中的投影系数，再用这个系数判断哪些 Hankel 列对当前参考更有贡献。具体实现不是直接删除不相关列，而是把原来的 g 正则化项改成带列权重的形式：

<div class="eq small-eq">
λ<sub>g</sub> ||g||<sub>2</sub><sup>2</sup> → λ<sub>g</sub> ||W<sub>k</sub><sup>1/2</sup>g||<sub>2</sub><sup>2</sup> = λ<sub>g</sub> Σ<sub>i</sub> w<sub>i</sub>g<sub>i</sub><sup>2</sup>
</div>

<div class="note">
相关列：w<sub>i</sub>=1，惩罚较小；不相关列：w<sub>i</sub>=w<sub>off</sub>&gt;1，惩罚较大。这样优化器会优先使用相关 Hankel 列，但仍保留其他列作为备选。
</div>

<div class="ref">
Ref: Näf, Moffat, Eising, Dörfler, “Choose Wisely: Data-Enabled Predictive Control for Nonlinear Systems Using Online Data Selection”, 2025. 本文实现为基于参考投影系数的软局部数据选择。
</div>

---

# 3. 参考工作二：Gain-Scheduled DeePC（动机参考）

<div class="cols">
<div>

## 文献大概内容

- 面向非线性系统中的 DeePC 控制问题
- 关注的问题是：固定 DeePC 配置可能只适合部分运行条件
- 基本结论：DeePC 的数据表示或控制配置可以随当前工况变化
- 本页只作为动机参考，不展开该文的具体算法细节

<div class="note">
这篇工作不是 UAV 场景，也不是本文要复现的算法；这里只引用其“固定配置可能不足”的思路。
</div>

</div>
<div>

## 本文借鉴方式

- 借鉴一个保守判断：无人机不同飞行状态下，固定 DeePC 配置可能不够
- 因此本文比较 static DeePC 与 maneuver-aware DeePC
- 具体实现仍以本项目的数据选择、参数设置和仿真实验为主
- 数据选择的直接参考仍是 Select-DeePC

<div class="eq small-eq">
(λ<sub>g</sub>, λ<sub>y</sub>)<sub>k</sub> 由 φ<sub>k</sub> 查表选择
</div>

<div class="note">
φ<sub>k</sub>：当前飞行阶段；λ<sub>g</sub>：g 系数正则化权重；λ<sub>y</sub>：输出松弛/数据不一致惩罚权重。
</div>

</div>
</div>

<div class="ref">
Ref: Guerrero, Lakshminarayanan, Rojas, “Gain-Scheduled Data-Enabled Predictive Control: A DeePC Approach for Nonlinear Systems”, 2025.
</div>

---

# 4. UAV-specific controller adaptations

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

# 5. UAV-specific phase/data design

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

# 6. 方法图

![width:920px](figures/fig01_method_overview.png)

---

# 7. 自写简化仿真环境与多 seed 结果

当前主结果来自**自写的轻量级 Python 仿真环境**，不是 Gazebo：

<div class="cols">
<div>

**仿真环境**

- 用于快速验证 DeePC 参数切换是否有效
- 轨迹任务：step、figure8
- 控制器：A1 Static DeePC vs A2 Maneuver-aware DeePC
- 随机种子：41–50，共 10 个 seed

</div>
<div>

**设置说明**

- A1：固定 λ<sub>g</sub>=30, λ<sub>y</sub>=1e4
- A2：按 smooth / transition / step-like 切换参数
- 目的：验证 phase-conditioned regularization 是否带来稳定收益
- 限制：该环境比 Gazebo 简化，不能代表最终实机/高保真仿真效果

</div>
</div>

<table class="tbl compact" style="margin-top:18px;">
<tr><th>Trajectory</th><th>Static Position RMSE</th><th>Maneuver-aware Position RMSE</th><th>Paired Result</th></tr>
<tr><td><strong>figure8</strong></td><td>0.0868 ± 0.0096</td><td class="good">0.0647 ± 0.0067</td><td class="good">10 / 10 wins</td></tr>
<tr><td><strong>step</strong></td><td>0.1364 ± 0.0234</td><td class="good">0.1046 ± 0.0133</td><td class="good">10 / 10 wins</td></tr>
</table>

---

# 8. 主结果图

![width:920px](figures/fig02_main_results.png)

---

# 9. Gazebo 模型的特殊性

Gazebo 中的 quadrotor tracking 问题比自写简化仿真更难，主要差别在于：

<div class="cols">
<div>

**模型与执行链路更复杂**

- 四旋翼刚体动力学更接近真实系统
- 存在电机/推力响应、饱和和姿态耦合
- 控制命令需要经过 ROS/Gazebo 的执行链路
- 轨迹误差会被低层控制器和模型延迟放大

</div>
<div>

**DeePC 数据假设更难满足**

- 数据采集和执行不是同一个理想离散系统
- /clock、控制频率和 reference index 对齐会影响历史数据
- 在线记录的 u/y 如果因果错位，会破坏 Hankel 数据一致性
- 简化仿真中的参数不一定能直接迁移到 Gazebo

</div>
</div>

<div class="note warn">
因此，Gazebo 不是简单复现实验结果的环境，而是当前项目中暴露系统难点的主要位置。
</div>

---

# 10. Gazebo 分支对比：量化结果

目前仓库中只有 best branch 的轨迹图，因此这里先用分支表说明对比关系：

<table class="tbl compact">
<tr><th>Branch</th><th>Completed</th><th>RMSE / m</th><th>Solver / ms</th><th>说明</th></tr>
<tr><td><strong>support384/off4</strong></td><td>yes</td><td class="good">2.0307</td><td>97.9</td><td>当前 best branch</td></tr>
<tr><td>support512/off4</td><td>yes</td><td>2.8865</td><td>31.8</td><td>同类 local selection，对比更大支持集</td></tr>
<tr><td>no local selection</td><td>yes</td><td>3.4126</td><td>163.7</td><td>无局部数据选择，对比数据支持策略</td></tr>
<tr><td>Q<sub>z</sub>=2.0 baseline</td><td>yes</td><td>3.7575</td><td>77.3</td><td>较弱 z 权重 baseline</td></tr>
</table>

<div class="note warn">
这张表能说明 best branch 优于几个 Gazebo 分支，但不能替代轨迹图对比。当前仍应表述为：Gazebo 中 best branch 相对更好，但整体仍未收敛。
</div>

---

# 11. Gazebo Circle-Box 对比：Static vs Maneuver-Aware

<div class="cols">
<div>

**XY 轨迹叠加对比**

![width:460px](figures/gazebo_circle_box_static_vs_aware_xy.png)

</div>
<div>

**误差曲线叠加对比**

![width:460px](figures/gazebo_circle_box_static_vs_aware_error.png)

</div>
</div>

---

# 12. Gazebo Circle-Easy：Static vs Maneuver-Aware

<div class="cols">
<div>

**XY 轨迹叠加对比**

![width:460px](figures/gazebo_circle_easy_static_vs_aware_xy.png)

</div>
<div>

**误差曲线叠加对比**

![width:460px](figures/gazebo_circle_easy_static_vs_aware_error.png)

</div>
</div>

---

# 13. Gazebo Circle-Easy 方法消融

<div class="cols">
<div>

![width:500px](figures/gazebo_circle_easy_ablation_overlay_xy.png)

</div>
<div>

<table class="tbl compact">
<tr>
<th>Variant</th>
<th>XY RMSE (m)</th>
<th>Max XY error (m)</th>
<th>Solver ms</th>
</tr>
<tr>
<td>Static DeePC</td>
<td>0.77166</td>
<td>3.000</td>
<td>28.334</td>
</tr>
<tr>
<td>Local-data only</td>
<td class="good">0.40831</td>
<td class="good">0.994</td>
<td>16.176</td>
</tr>
<tr>
<td>Full Maneuver-Aware</td>
<td>0.71303</td>
<td>2.604</td>
<td class="good">15.145</td>
</tr>
</table>

<div class="note">
这组消融说明：当前 Gazebo circle 上收益主要来自 local-data selection；完整 maneuver-aware bundle 没有稳定成为最优。
</div>

</div>
</div>

---

# 14. Gazebo Circle-Box（Causal Alignment）：Static vs Maneuver-Aware

<div class="cols">
<div>

**XY 轨迹叠加对比**

![width:460px](figures/gazebo_circle_box_causal_static_vs_aware_xy.png)

</div>
<div>

**误差曲线叠加对比**

![width:460px](figures/gazebo_circle_box_causal_static_vs_aware_error.png)

</div>
</div>

---

# 15. Circle-Box 不同 Horizon 对比：N=25 vs N=40 vs N=80

<div class="cols">
<div>

![width:500px](figures/gazebo_circle_box_horizon_overlay_xy.png)

</div>
<div>

<table class="tbl compact">
<tr>
<th>Setting</th>
<th>XY RMSE (m)</th>
<th>Max XY error (m)</th>
<th>Solver ms</th>
</tr>
<tr>
<td>N=25</td>
<td>7.63435</td>
<td>10.944</td>
<td>51.147</td>
</tr>
<tr>
<td>N=40</td>
<td class="good">6.22933</td>
<td class="good">9.231</td>
<td>71.598</td>
</tr>
<tr>
<td>N=80</td>
<td class="bad">19.33523</td>
<td class="bad">26.368</td>
<td class="bad">148.196</td>
</tr>
</table>

<div class="note">
`N=40` 是当前 `circle_box` 上更合理的平衡点；`N=80` 解算更慢，轨迹也明显发散。
</div>

</div>
</div>

---

# 16. 当前 Gazebo 结论

当前 Gazebo 结果的结论要谨慎表述：

- best branch 的 RMSE 低于几个 Gazebo 分支，包括 no-local-selection 和 Q<sub>z</sub>=2.0 baseline
- 但是所有分支都没有真正收敛，best branch 也存在后段漂移
- 目前还没有找到一组能够在 Gazebo 中稳定成功收敛的参数和数据配置

<div class="note warn">
因此，Gazebo 部分目前不是“成功验证方法”，而是说明高保真链路中仍存在模型差异、输入约束、数据激励、时间同步和因果记录问题。
</div>

---

# 17. 下一步计划

1. 固定自写仿真中的公平 baseline，补充 phase-resolved RMSE
2. 对 λ<sub>g</sub>、λ<sub>y</sub> 做更系统的网格搜索和消融
3. 检查数据采集中的 excitation 强度和 phase coverage
4. 补齐 Gazebo 的 baseline / best branch 对比图和误差曲线
5. 将 Gazebo 目标从“直接收敛”拆成 smoke test → tracking improvement → stable convergence

---

# 18. 总结

- 当前项目不应表述为“复现 DeePC 无人机控制器”
- 更合适的表述是：**面向多机动阶段的 phase-aware UAV DeePC**
- 自写简化仿真中，A2 在 step 和 figure8 上稳定优于 static DeePC
- Gazebo 中 best branch 相对更好，但所有分支都尚未成功收敛，系统验证仍是当前主要难点

<div class="note">
希望讨论：下一步应优先补简化仿真的消融实验，还是集中解决 Gazebo 中的收敛问题？
</div>
