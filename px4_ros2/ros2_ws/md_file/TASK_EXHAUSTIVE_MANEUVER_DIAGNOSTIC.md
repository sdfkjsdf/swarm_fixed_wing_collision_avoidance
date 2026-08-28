# Exhaustive Maneuver Diagnostic — Five-Aircraft Point Convergence

Status: **implemented and executed; DSD 10 m acceptance failed**

Date: 2026-08-28

## 1. Diagnostic Question

Determine whether the five-aircraft point-convergence failure is primarily
caused by the heuristic `Current Best + two Alternates` pre-selection or by a
later part of the prediction, selection, activation, and aircraft-response
approach.

This is a diagnostic HILS extension. It is not a replacement for the production
three-candidate datalink policy.

## 2. Controlled Comparison

The production path retains three candidates per aircraft and evaluates
`3^5 = 243` joint combinations. The test-only exhaustive path predicts all
seven level-turn candidates per aircraft:

```text
roll candidates = {-45, -30, -15, 0, +15, +30, +45} degrees
joint combinations = 7^5 = 16,807
aircraft pairs per combination = C(5, 2) = 10
```

Both paths use the same:

- ground-kinematic trajectory predictor;
- compact trajectory intent and cubic reconstruction;
- covariance propagation and 95% uncertainty margin;
- PMR, MASD, and AD equations;
- DSD = 10 m;
- common-time maneuver evaluation and stateful ownship-threat activation;
- five-aircraft point-convergence spawn and guidance configuration.

The exhaustive evaluator computes the trajectory-dependent pair results only
once for each aircraft pair and candidate pair:

```text
10 aircraft pairs x 7 x 7 candidates = 490 unique pair evaluations
```

The 16,807 joint combinations then reuse those cached pair results. No thread
is created per candidate, pair, or combination.

## 3. Source and Project Boundary

Turner et al. (2012), Section 4.5 and Wadley et al. (2013), Section III-D
support systematic combination scoring, independent aircraft decisions, and
the `Current Best + two Alternates` operational structure. The public papers
describe up to 81 combinations for four cooperating aircraft.

The seven-candidate, five-aircraft exhaustive search is therefore a
**project diagnostic**, not a claim about the disclosed Auto ACAS production
implementation. Its purpose is to remove heuristic candidate omission as an
experimental confounder.

## 4. Implementation Boundary

- Default `ManeuverSelectionWorkerParams::exhaustive_test_mode` is `false`.
- The guidance-node parameter
  `maneuver_selection_exhaustive_test_mode` defaults to `false`.
- HILS enables the path only with
  `MANEUVER_SEARCH_MODE=exhaustive`.
- Production three-candidate intent QoS depth remains 5.
- Exhaustive test mode uses intent history depth 16 because seven packets are
  published as one timestamped candidate set.
- A decision reports `evaluated_combination_count = 16807` only after all five
  complete seven-candidate sets for the same epoch are available.

## 5. HILS Evidence

| Run | Search | Minimum 3D separation | DSD violation duration | Same-epoch tuple consensus |
|---|---:|---:|---:|---:|
| `point_baseline_smoke_20260828_01` | no override | 0.510 m | 5.4 s | N/A |
| `point_avoidance_best_ad_20260828_01` | heuristic, 243 | 3.505 m | 2.6 s | 98.17% of 219 common epochs |
| `point_avoidance_exhaustive_20260828_02` | exhaustive, 16,807 | 4.271 m | 3.2 s | 94.91% of 216 common epochs |

Every node in the valid exhaustive run produced at least 216 qualified
16,807-combination decisions. The resulting H.264 MP4 and the common-NED
overview plot were generated offline from the finalized rosbag.

The earlier run `point_avoidance_exhaustive_20260828_01` is invalid evidence:
the default SensorDataQoS depth of 5 could not retain a seven-packet burst, so
four nodes never assembled a complete exhaustive candidate set. The test-only
QoS depth correction was verified before the valid `_02` run.

## 6. Finding

Removing heuristic candidate omission did not satisfy the 10 m DSD. The
exhaustive run improved the single minimum-distance statistic by 0.766 m over
the heuristic run, but it still reached only 4.271 m and spent 3.2 s below the
DSD threshold. One run per search mode is insufficient to rank their small
difference statistically, but it is sufficient to reject the claim that the
current failure is caused only by the three-candidate heuristic.

The original evidence pointed to the shared downstream path:

- the original activation began only after the selected full-search minimum AD
  crossed below zero; the first applied exhaustive log entries were already
  around `AD = -2.91 m`;
- the predictor assumed one selected lateral-acceleration command was held over
  the 4.5 s horizon, while the original runtime could select a different
  command at the next 0.25 s epoch;
- the real PX4 roll response and command-coupling delay are not identical to
  the predictor's roll time constant and instantaneous command boundary;
- the public source's detailed switching superiority threshold, hysteresis,
  and complete uncertainty roll-up are not disclosed and were not invented.

The activation-timing and command-persistence defects have since been corrected
and are documented in `TASK_STATEFUL_MANEUVER_ACTIVATION.md`. Predictor-to-PX4
response fidelity and distributed selection agreement remain open rather than
being masked by further heuristic tuning.

## 7. Five-Axis Re-Audit

The audit target is this diagnostic extension, not a claim that the complete
undisclosed Auto ACAS production algorithm has been reproduced.

| Axis | Verdict | Evidence |
|---|---|---|
| Source accuracy | PASS | The paper-backed three-candidate, independent-scoring, timing, PMR/MASD/AD, and activation claims remain separated from the project-only exhaustive search. |
| Interpretation fidelity | PASS | The result does not claim that seven-candidate five-aircraft enumeration is prescribed by the papers, nor that one successful enumeration would certify avoidance. |
| Complexity proportionality | PASS | The existing worker, intent transport, reconstructor, covariance propagator, and pair evaluator are reused. Only 490 trajectory pair evaluations are performed; no node, worker thread, or per-combination trajectory object was added. |
| Implementation correctness | PASS for the diagnostic | Unit and ROS integration tests verify 49 and 16,807 combinations, all five HILS nodes produced exhaustive qualified decisions, and the valid rosbag reports exhaustive counts on every qualified decision. |
| Directional alignment | PASS | Full search removes candidate pre-selection as the controlled variable while leaving the production default unchanged. |

The five-axis result does **not** override the flight-level acceptance result:
the diagnostic implementation passes its conformity audit, while the current
collision-avoidance behavior fails the 10 m DSD requirement. The public papers
do not provide enough detail to infer the exact switching superiority margin,
hysteresis, full uncertainty roll-up, or PX4-specific response compensation;
those items remain evidence gaps rather than invented behavior.
