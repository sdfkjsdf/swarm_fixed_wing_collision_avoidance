# MASD uncertainty and latency audit — 2026-09-05

## Scope and outcome

Authoritative requirements: the user's seven-part measurement request in this
conversation. Preserve AD < 0 activation, controller decisions and the current
MASD parameters; measure first and propose calibration only from evidence.
This report describes project implementation, not an undisclosed Lockheed formula.

**Outcome:** age alignment includes both mean and covariance. The observed U95
coverage is insufficient in the command-matched population. AD loss has multiple
contributors, including changes in the nominal-PMR time. A numerical calibrated
Q or additional distance budget is **not identified** by these measurements.

No activation threshold, Q, or distance margin was changed in this task.
Optional runtime instrumentation was added; it is not entirely test-only code.
Its parameter `masd_diagnostics_enabled` defaults to false. The measurement run
enabled it. Instrumentation has nonzero CPU/network overhead and is not proof
that two independent closed-loop runs will execute identical trajectories.

## Evidence and execution

Repository baseline: `67ebf73` (`Fix distributed candidate library delivery`).
Pre-existing uncommitted roll-model edits set the source default to 0.415 s.
Those edits were preserved. The measurement explicitly overrides tau_phi to
**0.5 s on all five nodes**, matching the historical comparison run.

| Run | Recording | Common fixed-wing interval | EKF-position minimum / time below 10 m |
|---|---|---|---|
| Historical | `hybrid_formation_commit_67ebf73_repro_200s_20260905_01` | approximately 180 s | 9.770 m / 0.2 s |
| Instrumented | `hybrid_formation_masd_budget_200s_20260905_01` | 180.9 s | 5.644 m / 1.7 s |

Both are 200-s Formation pentagon recordings, with node 0 on Raspberry Pi and
nodes 1–4 plus PX4/Gazebo on PC. The instrumented run is a measurement run, not a
safety improvement or a controlled causal A/B experiment. No conclusion that
instrumentation, Q, or a particular latency alone caused the performance
difference is supported.

The instrumented run had 722 common valid graph epochs, matching discrete
component tuples in all 722, and zero CandidateSetsIncomplete records. All 668
common qualified selection epochs had matching tuples. Raw ARM/x86 floating
hash equality is not the agreement criterion. Aggregate graph evaluation p95
was 7.232 ms and maximum 11.055 ms; this alone does not measure the entire
state-to-execution chain.

Local and isolated Pi measurement builds used Release. Local guidance flags
were `-O3 -DNDEBUG`. The 39 worker tests passed, including trace-enabled versus
trace-disabled control-result equivalence. Shell syntax, Python compilation and
`git diff --check` passed. This is not a whole-package safety certification.

The original Pi `collision-avoidance:distributed` image and
`collision-avoidance-dev` container were preserved. A separate measurement image
was built. The temporary experiment container was stopped after recording;
the existing launcher cleanup wait required manual completion. Bag and ULogs
were retained on PC, and post-recording ULog samples were excluded from analysis.

## 1. Current MASD and source age

For this configuration:

```
MASD = 2.144 m + 10 m + U95(t_effective) + 0 m
U95  = sqrt(7.814727903251179 * los^T (P_i + P_j) los)
AD   = PMR - MASD
```

`P_i` and `P_j` are 3-D position covariance blocks in common NED. Cross-aircraft
covariance is assumed zero. The multiplier is support from a nominal 95% 3-D
ellipsoid, not a one-sided 1-D 1.645-sigma interval.

Source evidence in `collision_avoidance`:

- `src/selection/ManeuverCombinationEvaluator.cpp:125`: `interpolateCone()`
  interpolates both position and position covariance with the same indices.
- The same file, lines 595–668: both aircraft use source age plus evaluation
  horizon, including at the PMR instant.
- The same file, lines 671–693: the U95 and MASD arithmetic above.
- `src/nodes/vtol_guidance_main.cpp:180`: DSD, communication margin and the
  half-wingspan parameters are passed to the evaluator.
- `src/estimation/trajectory_prediction/TrajectoryIntent.cpp:310`: the receiver
  propagates covariance along the reconstructed mean at 0.1-s steps.

The available common horizon is shortened by source age; it is not extrapolated
beyond the transmitted 4.5-s support. There is **no position-only age shift bug**
in these functions. Do not add source age again as a distance margin.

The analyzer reuses the production C++ receiver and covariance propagator.
Maximum replayed-versus-recorded global AD discrepancy was 0.00004294 m in the
historical run and 0.00004410 m in the instrumented run, over 722 epochs each.

This confirms numerical reconstruction, not that source timestamp labels are
perfect wall-clock timestamps or that hypothetical commands were executed.

## 2. Ground truth and clock boundary

The existing HILS separation summary uses transformed PX4 **estimated** odometry.
For coverage, this audit instead uses simulator `vehicle_global_position_groundtruth`
from ULog. The geographic coordinates are inverted with the Gazebo-classic
projection actually used by this fixture (radius 6,353,000 m, shared home
47.397742 deg / 8.545594 deg / 488 m), not a different WGS84 scale.

Evidence: firmware `Tools/simulation/gazebo-classic/sitl_gazebo-classic/include/common.h:253`,
`src/gazebo_groundtruth_plugin.cpp:140`, and
`src/modules/simulation/simulator_mavlink/SimulatorMavlink.cpp:590`.

With ULog timestamps mapped using raw DDS timesync observations, the estimated
simulator-ground-truth minima are:

| Run | Minimum | Pair / time after common FW start | Any pair below 10 m |
|---|---:|---|---:|
| Historical | 7.911 m | 0–1 / 74.35 s | 0.85 s |
| Instrumented | 5.610 m | 0–2 / 145.75 s | 2.53 s |

These use a 10-ms interpolation grid, with ground-truth gaps above 150 ms rejected.
They are not extra independent samples or exact continuous-time minima. They
must not be mixed with the older EKF-position metric.

### Timestamp labels are not exact UTC

PX4 serializes timestamps using a **filtered estimated offset**, while raw
timesync observations record the current handshake offset. The filtered value
lagged the observations during SILS. Across nodes, median label-to-observed-time
differences were approximately 12–46 ms; p95 was 40–68 ms.

Evidence: firmware `src/modules/uxrce_dds_client/uxrce_dds_client.cpp:64` and
`src/lib/timesync/Timesync.cpp:45`. The analyzed timesync instance was verified
to have `SOURCE_PROTOCOL_DDS=2`.

Mapping physical ULog receipt time with only the filtered offset produced
impossible negative transport delays. The report therefore separates:

1. the timestamp labels actually used by the controller;
2. raw-observation-based wall-time reconstruction for physical events;
3. same-host monotonic intervals for proposal/commit/publication.

Read-only Pi-to-PC clock probes bounded the offset at approximately
57.12–59.89 ms before the run and 62.46–65.24 ms after it. No clock was changed.
Interpolation between probes and raw DDS observations still has uncertainty.
Two reconstructed publish-to-receipt intervals remained slightly negative
(-2.916 ms and -1.422 ms); they remain visible and are not clamped away.

## 3. U95 coverage

Measure at the declared absolute prediction time:

```
e_R_positive = max(0, predicted_range - simulator_actual_range)
covered = (e_R_positive <= U95)
```

Three populations are kept separate:

- **Operational:** assembled hypothetical tuples versus subsequent motion.
  This population includes commands never executed; it cannot calibrate Q.
- **Lateral matched:** the actual PX4 lateral command agrees from source to
  target. This is the strongest available control check in the historical bag.
- **Speed and lateral matched:** additionally requires active avoidance and
  matching published ground-speed command. It does not establish perfect EAS,
  altitude, or longitudinal closed-loop tracking.

### Instrumented run: speed-and-lateral matched subset

| Horizon | Samples | Represented epochs | Coverage |
|---|---:|---:|---:|
| 0–0.5 s | 1,582 | 103 | 71.87% |
| 0.5–1 s | 623 | 65 | 80.26% |
| 1–2 s | 335 | 28 | 85.37% |
| 2–4.5 s | 10 | 3 | 100%, **insufficient evidence** |
| All | 2,550 | 103 | 75.80% |

The last row/bin cannot certify 95% long-horizon performance. Overlapping
predictions, repeated pairs and adjacent epochs are correlated. Bootstrap
intervals in the CSV are descriptive within this one encounter, not independent
Monte Carlo confidence bounds.

| Maneuver class (maximum absolute commanded bank in pair) | Samples | Coverage |
|---|---:|---:|
| 0 deg | 26 | 57.69% |
| 15 deg | 345 | 71.30% |
| 30 deg | 215 | 54.88% |
| 50 deg | 1,964 | 79.12% |
| State-opposed roll direction | 391 | 54.48% |
| No state-opposed roll direction | 2,159 | 79.67% |
| At nominal PMR instant | 169 | 94.67% |

The reversal label means that a new commanded bank opposes the initial measured
bank, with initial absolute bank above 5 deg. It is an explicit state-based
proxy, not an inferred undisclosed previous command. Class populations also
have different horizon distributions; their percentages are not causal rankings
of maneuver quality.

Historical lateral-matched coverage was 86.10% over 1,137 samples: 81.90%, 92.95%,
93.13%, and 1/1 across the four horizon bins. The historical bag cannot verify
the additional ground-speed condition. Earlier exploratory percentages based on
EKF odometry or filtered-offset wall-time mapping are superseded by these tables.

### Q provenance and interpretation

Production default continuous spectral-density diagonal is
`[0.25, 0.25, 0.25, 0.04, 0.001, 0.04, 0.001]` in
`[p_N,p_E,h,V,course,h_dot,roll]` order.
`UncertaintyTypes.hpp:44` defines it; `TrajectoryUncertainty.cpp:244` adds `Q*dt`
after `F*P*F^T`. No reproducible fit and held-out validation for this exact
production Q were found in the inspected code/configuration.

Undercoverage does **not** identify Q as the sole defect. Initial navigation
uncertainty, mean bias, timestamp labels, input consistency and model residual
are still mixed. In particular, poor near-zero-horizon coverage cannot be
corrected solely by increasing future process noise without checking P0/time.

## 4. Latency distributions

All figures below are milliseconds. Stages use different eligible sample sets;
**do not add their percentiles** to obtain an end-to-end percentile.

| Stage | n | Median | p95 | p99 | Max |
|---|---:|---:|---:|---:|---:|
| Latest fusion sample → proposal, observed-clock reconstruction | 3340 | 161.15 | 169.99 | 174.56 | 185.93 |
| Latest belief publication → proposal, observed-clock reconstruction | 3340 | 9.12 | 18.00 | 22.33 | 33.95 |
| Frozen candidate source → proposal, observed-clock reconstruction | 3340 | 201.80 | 216.10 | 221.72 | 263.02 |
| Proposal ready → commit, same-host monotonic | 3339 | 26.80 | 67.52 | 76.97 | 93.60 |
| Commit while active → selected ROS publication | 541 | 23.64 | 39.01 | 42.94 | 43.83 |
| Commit while inactive → later active ROS publication | 54 | 108.56 | 207.99 | 243.67 | 246.15 |
| Actual ROS publish call | 27139 | 0.053 | 0.140 | 0.162 | 1.270 |
| ROS publication → matching PX4 receipt | 267 | 5.56 | 12.30 | 17.79 | 25.99 |
| PX4 receipt → 10% measured roll response | 64 | 240.00 | 558.50 | 584.80 | 610.00 |
| Proposal → 10% roll response, linked actual transitions | 62 | 371.86 | 617.17 | 641.07 | 647.23 |

Five ROS/PX4 transitions had no unambiguous match. The two slightly negative
receipt intervals are excluded from downstream physical-chain matching but
retained in the raw receipt distribution. PX4 receipt is identified from actual
ULog input-value transitions, not any later sample with the same value.

The ~152-ms difference between fusion and publication is already subject to
fusion-horizon propagation. Frozen source age is already subject to trajectory
time alignment. Inactive-at-commit waiting includes the activation policy, not
just transport or scheduler delay. The 10% roll-response statistic includes roll
dynamics and must not be added again as a communication margin.

An exploratory fixed-tau additional-delay fit hit its 0.2-s search boundary in
24/64 cases. It is **not identified residual delay**, and no budget is derived
from it. A reliable residual-delay identification must distinguish actual PX4
attitude-target formation, existing roll dynamics and remaining timing error.

## 5. Inter-cycle AD loss

`L=max(0,AD_previous-AD_current)`. Same pair and same input revisions are compared
separately from candidate/active-mode changes.

| Instrumented run population | n | Median loss | p95 | p99 | Max |
|---|---:|---:|---:|---:|---:|
| Actual monitor, same pair/input/active mode | 60128 | 0.00 m | 2.43 m | 4.46 m | 19.85 m |
| Uniform 20-Hz offline replay, same pair/input | 34316 | 0.00 m | 2.57 m | 4.46 m | 22.98 m |
| Uniform 4-Hz offline replay, same pair/input | 5414 | 0.00 m | 12.37 m | 19.49 m | 38.33 m |
| Actual consecutive 4-Hz graph minima, node 0 | 720 | 0.09 m | 7.40 m | 15.05 m | 30.85 m |

Actual same-mode monitor intervals had median 40 ms, p95 64 ms and p99 96 ms;
they must not be mislabeled an exact uniform 50-ms series. Offline results use
an integer-microsecond uniform grid and are not each node's exact cache history.
Changed-input losses are recorded separately in JSON/CSV, not pooled here.

Historical same-input offline loss p95 was 2.024 m at 20 Hz and 9.098 m at 4 Hz.
The historical actual node-0 graph-minimum loss p95 was 7.436 m.

### A directly recorded discontinuity

Node 4 monitoring node 2, 173.309 s after common FW start, same input/mode:

```
interval:       0.064 s
AD:             +8.315 m -> -5.017 m
PMR change:     -3.692 m
MASD change:    +9.640 m
PMR horizon:    0.2 s -> 4.4 s
```

The uncertainty is evaluated at the **nominal minimum-range instant**. When
that minimizer moves, its uncertainty can change sharply even when the minimum
range itself changes modestly. This is one measured mechanism, not a claim that
all violations have this cause. Of 72 same-input/mode positive-to-negative
monitor crossings, 7 had PMR-horizon changes above one second.

The code currently computes `Range(t_PMR)-MASD(t_PMR)`, not
`min_t(Range(t)-MASD(t))`. This audit does not replace that agreed definition.
Increasing Q without analyzing this distinction can increase such AD jumps.

## 6. MASD budget and double-counting audit

| Source | Current treatment | May it be added again wholesale? |
|---|---|---|
| Aircraft geometry | 2.144 m explicit | No |
| DSD | 10 m explicit | No |
| Initial navigation covariance | EKF belief mapped to P0 | No; validate its coverage/mapping first |
| EKF fusion horizon | Mean and covariance propagated to publication horizon | No |
| Received source age | Mean and covariance at age + horizon | No |
| Nominal roll lag / rate limit | Predictor tau=0.5 s and 70 deg/s limit | No |
| Random process/model uncertainty | Fixed Q propagated into U95 | No; calibrate and test residual first |
| Cubic compression error | Mean reconstructed; no separate fitted error budget | Do not silently assume Q covers it |
| Actual coordination/publication delay | Measured here; no fitted residual budget | Only the unmodeled remainder, if identified |
| Clock-label error | Filtered DDS time labels; nonzero observed discrepancy | Not the same as network transit time |
| Candidate/mode changes | Different motion hypotheses | Not a random process-noise sample by default |
| Bcomm | 0 m | No arbitrary replacement |

`acceptOwnshipBelief()` at `ManeuverSelectionWorker.cpp:392` compensates fusion
delay using `m_current_best_id` from the candidate table. Whether that command
matches actual past Formation/avoidance input during every fusion interval is
an unresolved input-contract issue, not fixed by this analysis.

## 7. Recommended calibrated MASD and open issues

Retain current control settings for this measurement handoff. The supported
candidate structure remains:

```
MASD = D_aircraft + DSD + U95_calibrated(t_effective)
       + B_delay_residual
       [+ B_model_residual only if calibrated covariance still misses it]
```

**No numerical Q, B_delay_residual or B_model_residual is recommended yet.**
The minimum evidence-based calibration order is:

1. Establish physical timestamp/P0 consistency and validate fusion compensation
   against the command actually executed. Future Q cannot explain every t≈0 error.
2. Fit covariance/process-noise using executed-command residuals, checking both
   position covariance and one-sided separation coverage. Keep systematic bias
   distinct from zero-mean noise. Preserve the production mean model for this
   comparison unless a separate change is approved.
3. Validate on independent held-out runs, including 2–4.5-s held commands,
   both turn directions, high bank and reversals. Current long-horizon samples
   are inadequate, and the full operational population is not a calibration set.
4. Only then assess remaining application jitter and model residual. Do not use
   the 0.25-s selection interval, total roll response, sum of p95 values, or an
   AD-loss percentile as an automatic additive distance budget.

Other open issues: cross-aircraft navigation-error correlation, cubic error
coverage, finite 0.1-s PMR sampling, nominal-PMR versus minimum-AD semantics,
unmatched transport events, and measurement-induced scheduling overhead.

## Five-axis static conformity audit

Scope: measurement implementation against the user's current request, not a
fresh literature audit or flight-safety certification. Source baseline is the
seven explicit requirements; implementation evidence is the code and artifacts
identified above and in `analysis/README_MASD.md`.

| Axis | Status | Source / implementation evidence | Reason and impact | Confidence |
|---|---|---|---|---|
| 1. Source accuracy | PASS | User items 1–7; evaluator, uncertainty and firmware time-conversion sites above | Project equations are attributed to code; no invented Lockheed formula | High |
| 2. Interpretation fidelity | PASS | User items 2, 3, 5; separated coverage populations and latency classes | Unexecuted commands, clock labels and roll response are not called calibrated random/communication error | High |
| 3. Complexity proportionality | PASS | User item 3; optional bounded trace queue and production C++ offline bridge | Existing calculations reused; no new solver or control subscription | High |
| 4. Implementation correctness | PASS for inspected measurement arithmetic | Replay errors <0.000045 m, 39 worker tests, explicit clock/ground-truth mapping | Numerical reconstruction checked; statistical/model sufficiency remains outside this pass | Moderate |
| 5. Directional alignment | PASS | User item 7; unchanged activation, Q and MASD with default-off instrumentation | Measurement precedes correction; no early trigger or magic margin | High |

Cross-axis boundary: source traceability and measurement conformity do not imply
95% coverage or safe control. **Current coverage adequacy: FAIL in the observed
matched population. A calibrated 95% long-horizon safety claim: INDETERMINATE.**
Evidence needed to resolve that claim is independent, command-consistent,
clock-validated long-horizon validation after calibration. No whole-system
five-axis safety PASS is asserted.

## Result files

Relative to `maneuver_selection_hils/result/summary/`:

- `masd_budget_67ebf73_20260905/`: historical-run replay and clock probes.
- `masd_budget_trace_20260905/`: instrumented coverage, runtime pair loss,
  latency samples, ground-truth separation, JSON summary and overview PNG.
- `hybrid_formation_masd_budget_200s_20260905_01/`: original HILS summary based
  on estimated common odometry; retained without rewriting its metric definition.

No commit or push was performed by this measurement task.
