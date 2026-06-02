# Gazebo Second-Layer Experiment Plan

## Goal

- Idea: extend the repaired non-Gazebo analysis onto Gazebo trajectories that can actually test phase-changing behavior.
- Hypothesis: if local-data selection adds value beyond bundles, the gain should appear first on a trajectory that elicits non-trivial `step_like`, not on trajectories that mostly stay in `transition`.
- Decision this experiment should inform: whether the next slide/story should use `half_circle_box` as the main second-layer diagnostic, or stay with `circle_box` horizon comparisons only.

## Main Experiments

- Name: `half_circle_box` same-dataset method comparison
- Question answered: does `full = bundles + local data` beat `bundles-only` on the best available phase-changing Gazebo diagnostic?
- Variables changed:
  - method: `static`, `bundles_only`, `full_phase_bias`
  - local-data mode only for `full_phase_bias`
- Variables held fixed:
  - trajectory: `half_circle_box`
  - dataset: `deepc_gazebo_autopilot_collect_pose_planar_half_circle_box_h40_v1.npz`
  - horizon: `N=40`
  - speed: `1.0 m/s`
  - autopilot weights and startup settings from preset `gazebo_half_circle_box_horizon40_matched_v1`
- Metrics:
  - `xy_rmse_m`
  - `max_xy_error_m`
  - `solver_ms`
  - `regime_usage`
  - `first_solve_failed`
- Expected outcome:
  - `bundles_only` should be at least as good as the current historical full-support run.
  - `full_phase_bias` is only worth keeping if it lowers `xy_rmse_m` or `max_xy_error_m` without a large compute blow-up.

- Name: `circle_box` anchored comparison
- Question answered: on the already-known matched `circle_box` line, does repaired `full_phase_bias` recover anything over the known `N=40` full-support result?
- Variables changed:
  - method: `static`, `bundles_only`, `full_phase_bias`
- Variables held fixed:
  - trajectory: `circle_box`
  - dataset: `deepc_gazebo_autopilot_collect_pose_planar_circle_box_h40_v1.npz`
  - horizon: `N=40`
  - speed: `0.6 m/s`
  - causal alignment and matched-data settings from preset `gazebo_circle_box_horizon40_matched_causal_v1`
- Metrics:
  - `xy_rmse_m`
  - `max_xy_error_m`
  - `solver_ms`
  - `regime_usage`
- Expected outcome:
  - `step_like` may still stay near zero.
  - this run is mainly a control to check whether the repaired local-data stack is harmless on the existing closeout trajectory.

## Ablations

- Component removed or altered: remove local data, keep bundles
- Why it matters: this is the current best simplified-sim baseline and the main comparator for whether local data adds anything in Gazebo.
- Minimum comparison set:
  - `bundles_only`
  - `full_phase_bias`

- Component removed or altered: remove bundles and local data
- Why it matters: `static` tells us whether any gain is due to maneuver-aware logic at all, rather than generic matched-data tuning.
- Minimum comparison set:
  - `static`
  - `bundles_only`
  - `full_phase_bias`

- Component removed or altered: trajectory choice
- Why it matters: `half_circle_box` and `circle_box` stress different failure modes.
- Minimum comparison set:
  - `half_circle_box`: `bundles_only` vs `full_phase_bias`
  - `circle_box`: `bundles_only` vs `full_phase_bias`

## Controls

- Control type: historical matched `circle_box` `N=40` reference
- Why it is needed: this is the cleanest existing Gazebo phase-changing closeout with matched data.
- Pass / fail condition:
  - if rerun `bundles_only` is far worse than historical `xy_rmse_m=6.22933`, treat the whole stack as unstable and stop interpretation.

- Control type: `half_circle_box` regime activation check
- Why it is needed: this trajectory is useful only if it still elicits non-trivial `step_like`.
- Pass / fail condition:
  - if `step_like` collapses to near zero, it is no longer serving as the intended detector diagnostic.

- Control type: no fresh `D-shape` or `rounded_arc_rect` before validation
- Why it is needed: both are currently poor uses of time.
- Pass / fail condition:
  - do not schedule them until `half_circle_box` or `circle_box` yields a clear methodological answer.

## Failure Risks

- Risk 1: `half_circle_box` dataset quality is already imperfect.
  - Mitigation: use it only as a method-comparison diagnostic, not as a final performance closeout.

- Risk 2: Gazebo nondeterminism can swamp small gains.
  - Mitigation: first run single-attempt comparisons and only expand to repeat runs if the effect size is meaningful.

- Risk 3: repaired `phase_bias_support` may still not transfer from simplified sim to Gazebo.
  - Mitigation: keep `bundles_only` as the main anchor and stop if `full_phase_bias` is consistently worse on both trajectories.

## Estimated Compute

- Cheapest useful run:
  - one trajectory, two methods, one attempt: roughly a few tens of minutes wall-clock once Gazebo is stable.
- Full run cost:
  - `2 trajectories x 3 methods`, each as a matched execute only, is an afternoon-scale batch on one machine.
- Rerun expectation:
  - likely `1` repeat only for runs that look surprisingly good or surprisingly bad.
- Bottleneck:
  - Gazebo wall-clock and manual artifact inspection, not code execution itself.

## Result Criteria

- What counts as success:
  - `full_phase_bias` beats `bundles_only` on `half_circle_box` by a visible margin in `xy_rmse_m` or `max_xy_error_m`, while keeping solver cost in the same order.

- What counts as no effect:
  - `full_phase_bias` and `bundles_only` differ only marginally, or trade a tiny error gain for a large compute penalty.

- What outcome would falsify the idea:
  - `full_phase_bias` is worse than `bundles_only` on both `half_circle_box` and `circle_box`.

- What would trigger a follow-up experiment:
  - `full_phase_bias` helps on `half_circle_box` but not `circle_box`.
  - if that happens, the next run should be a fresh post-fillet `rounded_arc_rect`, not `D-shape`.

## Concrete Run Order

1. `half_circle_box`: `bundles_only`
2. `half_circle_box`: `full_phase_bias`
3. `half_circle_box`: `static`
4. `circle_box`: `bundles_only`
5. `circle_box`: `full_phase_bias`
6. `circle_box`: `static`

Stop early if either of these happens:

- `full_phase_bias` is clearly worse than `bundles_only` on both trajectories.
- `bundles_only` itself regresses badly relative to the historical matched references.

## Concrete Commands

These are the commands to use as the starting point. The base preset already carries the matched dataset path and controller stack.

`half_circle_box` base preset:

```bash
python /home/roxy/Deepc/benchmarks/data_driven_mpc/ros_gp_mpc/scripts/gazebo_deepc_optimize.py \
  --preset gazebo_half_circle_box_horizon40_matched_v1 \
  --output-dir /home/roxy/Deepc/research_workbench/deepc_uav_system_survey/artifacts/<run_name>
```

`circle_box` base preset:

```bash
python /home/roxy/Deepc/benchmarks/data_driven_mpc/ros_gp_mpc/scripts/gazebo_deepc_optimize.py \
  --preset gazebo_circle_box_horizon40_matched_causal_v1 \
  --output-dir /home/roxy/Deepc/research_workbench/deepc_uav_system_survey/artifacts/<run_name>
```

Method variants to overlay on those presets:

- `static`
```bash
DEEPC_REGIME_AWARE=0 \
DEEPC_LOCAL_DATA_SELECTION=0
```

- `bundles_only`
```bash
DEEPC_REGIME_AWARE=1 \
DEEPC_LOCAL_DATA_SELECTION=0
```

- `full_phase_bias`
```bash
DEEPC_REGIME_AWARE=1 \
DEEPC_LOCAL_DATA_SELECTION=1 \
DEEPC_LOCAL_DATA_MODE=phase_bias_support \
DEEPC_LOCAL_DATA_OFF_WEIGHT=2.0 \
DEEPC_LOCAL_DATA_PHASE_LABEL_STEP_JUMP_THRESHOLD=0.6 \
DEEPC_LOCAL_DATA_PHASE_LABEL_TRANSITION_JUMP_THRESHOLD=0.12 \
DEEPC_LOCAL_DATA_PHASE_LABEL_TRANSITION_TOTAL_VARIATION_THRESHOLD=0.6 \
DEEPC_LOCAL_DATA_PHASE_LABEL_TRANSITION_NET_CHANGE_THRESHOLD=0.2
```

## Current Recommendation

- Use `half_circle_box` as the main second-layer diagnostic.
- Keep `circle_box` as the anchored control and horizon-context trajectory.
- Do not spend the next batch on `D-shape`.
- Do not use the existing `rounded_arc_rect` execute as current evidence; only revisit it after a fresh post-fillet matched execute.
