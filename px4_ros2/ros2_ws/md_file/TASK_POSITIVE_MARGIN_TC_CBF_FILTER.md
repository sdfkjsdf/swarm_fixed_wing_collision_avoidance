# Positive-Margin TC-CBF Candidate Filter

## 1. Objective

Add a ROS-independent, sampled candidate-admissibility gate before the existing
PMR/MASD/AD scoring path. The gate shall use the already reconstructed
46-point trajectory intents and shall not add a new ROS node, worker thread,
trajectory predictor, covariance propagator, optimizer, or root solver.

Primary implementation specification:

- `/home/hmcl/workspace/reference/paper/tc_cbf_schmitt_interval_safe_control_note_v3.md`
- `/home/hmcl/workspace/reference/paper/tc_cbf_schmitt_interval_safe_control_note_v3.pdf`

## 2. Evidence Boundary

### Directly supported by the cited sources

- TC-CBF uses left/right turning-circle clearance based on position, heading,
  speed, and maximum turning capability.
- The source discrete condition is
  `h[k+1] >= (1 - gamma) h[k]`, with `0 < gamma <= 1`.
- Auto ACAS predicts a finite candidate library and scores trajectory
  combinations with PMR/MASD/AD after candidate generation.
- Schmitt-Fichter supports direction-fixed scalar reduction for multi-obstacle
  avoidance, but it does not define this project's finite fixed-wing filter.

### Project synthesis, not a direct source claim

- The positive-margin shifted condition is
  `h[k+1] >= (1 - gamma) h[k] + gamma h_ref`.
- Fixed-wing turning radius is mapped to
  `rho = V_horizontal^2 / a_lat_max` under the existing coordinated-turn model.
- The existing Desired Separation Distance is reused as `h_ref`; aircraft
  half-wingspans form the geometric clearance radius.
- A signed bank candidate fixes one NED turn direction. The zero-bank candidate
  is admissible only when one *single* global left or right hypothesis passes
  every threat and every tested interval; directions may not be mixed per
  threat.

### Explicitly not claimed

- Continuous-time or inter-sample forward invariance.
- Recursive feasibility under actuator lag or model mismatch.
- Robustness to uncommunicated threat maneuvers.
- A formal 4.5 s guarantee when timestamp alignment leaves less than the full
  advertised horizon.
- Replacement of downstream covariance, MASD, AD, coordination, activation,
  or command-latching logic.

## 3. Runtime Contract

For each candidate and threat trajectory:

1. Reuse the evaluator's timestamp validity and same-time interpolation.
2. At every consecutive interval in the available common future horizon,
   compute horizontal turning-circle clearance
   `h = distance(turn_center, threat_position) - (size_clearance + rho)`.
3. Require an initially nonnegative clearance and
   `h_next - ((1 - gamma) h_current + gamma h_ref) >= 0` at every interval.
4. Intersect results across all threats without mixing left and right
   directions.
5. Only barrier-admissible candidates/combinations may enter PMR/MASD/AD.
6. If no barrier-admissible combination exists, report infeasibility and do
   not revive a barrier-rejected combination through the existing best-unsafe
   AD fallback. Existing active-command latching remains unchanged.

Uncertainty remains exclusively in the existing downstream MASD computation.
It is not added to `h_ref`, avoiding covariance double counting.

## 4. Minimal Implementation

- Extend `ManeuverCombinationEvaluator` with a fixed-size positive-margin
  barrier evaluator and diagnostic result fields.
- Reuse the existing reconstructed means, timestamps, and interpolation code.
- Run the barrier before AD in:
  - heuristic candidate pre-scoring,
  - Current Best + two-alternate joint evaluation,
  - seven-candidate exhaustive test evaluation.
- Add bounded parameters for `gamma`, `h_ref`, and `a_lat_max`; wire `h_ref`
  from DSD (10 m by default) and `a_lat_max` from the existing predictor.
- Keep the existing one-worker-per-aircraft architecture and SPSC queues.

## 5. Verification

Unit tests shall cover:

- invalid `gamma`, margin, acceleration, timestamp, and trajectory rejection;
- NED left/right sign mapping;
- a candidate passing every sampled interval;
- first-violation detection at an intermediate interval;
- multi-threat intersection that rejects left/right mixing;
- zero-bank global-direction handling;
- proof that a barrier-rejected combination does not call or participate in
  downstream AD selection;
- preservation of MASD uncertainty and the best-unsafe AD fallback *within*
  the barrier-admissible set;
- both 3-candidate and 7-candidate evaluator paths.

Then run the existing collision-avoidance unit suite, build the package, and
perform one GUI-free fixed-wing SILS smoke run.

## 6. Five-Axis Plan Audit

| Axis | Result | Reason |
|---|---|---|
| Source accuracy | PASS | Direct source equations and project synthesis are explicitly separated. |
| No over-interpretation | PASS | The result is named sampled common-horizon admissibility; formal continuous-time and robustness claims are excluded. |
| Proportional complexity | PASS | The plan reuses fixed trajectories, interpolation, evaluator paths, thread, and uncertainty logic; no MPC/QP/root solver or new node is added. |
| Implementation correctness | PASS | Direction consistency, all-threat/all-interval intersection, parameter validation, and pre-AD ordering are explicit acceptance criteria. |
| Directional alignment | PASS | The filter produces safe candidate sets first and leaves PMR/MASD/AD plus activation as downstream stages, matching the attached note's architecture. |

## 7. Adversarial Acceptance Gate

The implementation is not accepted if any of the following is possible:

- Threat A passes left and Threat B passes right, but the candidate is accepted
  without one direction passing both threats.
- An intermediate interval fails but the endpoint passes and the candidate is
  accepted.
- A barrier-rejected combination is selected by the best-unsafe AD fallback.
- Uncertainty is added to both `h_ref` and MASD.
- A stale, future-dated, non-finite, or less-than-two-sample aligned trajectory
  is accepted.
- A result is described as a continuous-time or exact 4.5 s safety guarantee.

## 8. Implemented Scope

Implementation completed on 2026-08-28:

- `PositiveMarginBarrierEvaluator` evaluates the shifted discrete condition on
  every consecutive sample in the timestamp-aligned common future horizon.
- NED left/right turning centers use the candidate's fixed direction and the
  existing coordinated-turn acceleration limit.
- The Current Best + two-alternate path and the seven-candidate exhaustive
  path cache directional pair evaluations, intersect one direction across all
  threats, and evaluate PMR/MASD/AD only after barrier admission.
- The heuristic pre-selection path applies the same all-threat directional
  intersection before its existing AD score.
- A barrier-rejected candidate cannot be restored by the downstream
  best-unsafe AD fallback. The fallback still works among barrier-admissible
  combinations when all of their AD values are negative.
- The production node wires `h_ref` to the existing DSD and derives
  `a_lat_max` from the existing maximum-roll parameter. The HILS configuration
  explicitly enables the filter.

No new node, worker thread, predictor, covariance propagator, QP, MPC, or root
solver was introduced.

## 9. Verification Result

### Build and tests

- `colcon build --packages-select collision_avoidance --cmake-args -DBUILD_TESTING=ON`:
  PASS.
- Full package regression suite: 74 tests, 0 errors, 0 failures, 0 skipped.
- The maneuver-combination target contains 18 tests, including four direct
  positive-margin evaluator tests and integration tests for static, joint, and
  exhaustive paths.
- `git diff --check`: PASS.

The tests explicitly verify NED direction signs, invalid parameter/time
rejection, reduced timestamp-aligned interval count, intermediate-sample
violation detection, no multi-threat left/right mixing, pre-AD rejection, and
best-unsafe fallback containment.

### GUI-free five-aircraft SILS smoke

Run ID: `tc_cbf_smoke_20260828_01`

- No `ERROR`, `FATAL`, exception, or traceback was found in the run logs.
- Common qualified selection epochs: 9.
- Same tuple over common qualified epochs: 9/9 (100%).
- Peer-ownship assumption mismatches: 0.
- Active candidate switches: 0 for every aircraft.
- Actual minimum 3-D separation: 1.238477525 m.
- Actual minimum horizontal separation: 1.174192481 m.
- Closest pair: aircraft 1 and 3 at 18.8 s.
- DSD 10 m violation: 21 samples, approximately 2.1 s.

Artifacts:

- summary: `px4_ros2/ros2_ws/src/testing_module/maneuver_selection_hils/result/summary/tc_cbf_smoke_20260828_01/summary.json`
- plot: `px4_ros2/ros2_ws/src/testing_module/maneuver_selection_hils/result/plot/tc_cbf_smoke_20260828_01/actual_maneuver_overview.png`
- video: `px4_ros2/ros2_ws/src/testing_module/maneuver_selection_hils/result/video/tc_cbf_smoke_20260828_01/actual_maneuver.mp4`

## 10. Post-Implementation Five-Axis Audit

| Axis | Result | Evidence |
|---|---|---|
| Source accuracy | PASS | The source TC-CBF geometry and discrete inequality are preserved; the shifted margin, fixed-wing radius mapping, and DSD reuse remain labelled as project synthesis. |
| No over-interpretation | PASS | The implementation reports only sampled, timestamp-aligned common-horizon admission. It does not claim continuous-time, exact 4.5 s, actuator-lag, or model-mismatch guarantees. |
| Proportional complexity | PASS | Existing intent reconstruction, interpolation, finite candidate tables, evaluator paths, AD logic, and worker architecture are reused. Directional pair results are cached instead of recomputed for every joint tuple. |
| Implementation correctness | PASS | Unit tests and code tracing confirm all-interval/all-threat intersection, one fixed direction per aircraft, invalid-input rejection, pre-AD ordering, and no revival of barrier-rejected combinations. |
| Directional alignment | PASS | The implementation produces barrier-admissible candidate sets before PMR/MASD/AD/Cost and leaves AMAC activation downstream, matching the attached note's architecture. |

## 11. Adversarial Audit

| Attack | Result | Evidence |
|---|---|---|
| Mix left for one threat and right for another | PASS | Per-aircraft left and right booleans are each intersected across every pair before the signed or zero candidate is admitted. |
| Hide an intermediate failure behind a safe endpoint | PASS | Every consecutive aligned interval is evaluated; a dedicated test places the violation between passing endpoints. |
| Recover a rejected candidate with maximin/best-unsafe AD | PASS | Barrier-rejected tuples exit before any AD evaluation and are absent from the fallback set. |
| Count covariance in both the barrier and MASD | PASS | Barrier clearance contains only geometry, turning radius, and `h_ref`; propagated covariance remains in MASD. |
| Accept stale/future/non-finite or empty aligned input | PASS | Shared timestamp/alignment validation rejects these inputs before barrier admission. |
| Claim formal closed-loop safety from a sampled filter | PASS | Documentation and diagnostics explicitly limit the claim to sampled candidate admission. |

### Independent system-level finding: NOT ACCEPTED as a 10 m safety proof

The SILS smoke proves that the new path is wired, runs without process errors,
and retains distributed tuple agreement. It does **not** prove collision
avoidance performance: the actual 1.238 m minimum separation violates the
10 m DSD. This finding is consistent with the source note's explicit unresolved
boundary between the positive-margin candidate layer and AMAC activation, plus
actuator lag, predictor mismatch, and inter-sample behavior.

Therefore the scoped implementation passes the five-axis and adversarial
conformity audits, but the complete guidance system must not be described as
DSD-safe. Adjusting `gamma`, changing activation timing, or redesigning the
latch from this single smoke run would exceed this task and would turn an
unvalidated tuning seed into a safety claim.
