# V4 Safe-Control Set — Minimal Implementation Plan

## 1. Outcome

Implement the V4 planar fixed-wing safe-control-set path in the existing
per-aircraft guidance node:

\[
\text{current ownship/threat state}
\rightarrow
\mathcal R_L^{safe},\mathcal R_R^{safe}
\rightarrow
\{r_{near},r_{left},r_{right}\}
\rightarrow
\text{existing TPA/PMR/MASD/AD/Cost/activation}.
\]

The core result is the pair of Left/Right intervals. Candidate generation is a
separate adapter. PMR, MASD, AD, Cost, distributed coordination, activation,
trajectory reconstruction, covariance propagation, and PX4 command output
remain downstream.

Governing documents:

- `/home/hmcl/workspace/reference/paper/MD_FILES/auto_acas_v4_safe_control_set_codex_spec.md`
- `/home/hmcl/workspace/reference/paper/MD_FILES/codex_v4_implementation_decisions_v2.md`

The investigation appended to the second document is part of this plan's
evidence baseline.

## 2. Scope controls

### Included

- ROS-independent scalar \(r=\dot\psi\) safe-set core.
- Separate Left and Right physical/safe intervals.
- Current TAS input with explicit `ACTUAL_TAS` versus `TRIM_FALLBACK` status.
- Per-cycle \(r_{\max}^{eff}\) calculation.
- Existing common-NED belief and peer trajectory-intent time alignment.
- One-step local speed freeze when validated air-relative acceleration is
  unavailable.
- Nominal projection and robust Left/Right representatives, maximum 3.
- Minimal dynamic candidate descriptor in the existing compressed intent
  packet.
- Existing worker-thread integration, diagnostics, unit tests, and headless
  five-aircraft SILS shadow validation.

### Excluded

- Wind-aware ground/air conversion.
- 3-D climb/descend avoidance.
- IAS-derivative to TAS-acceleration conversion.
- Covariance/datalink-robust V4 intervals.
- New Track Manager, ROS node, worker thread, QP, MPC, or nonlinear root
  solver.
- PMR/MASD/AD/Cost/activation redesign.
- Formal forward-invariance or 10 m system-safety claim.

## 3. Fixed mathematical contract

For each control update:

\[
r_{\max}^{eff}
=
\min\left(
\operatorname{rad}(\texttt{FW\_Y\_RMAX}),
\frac{g\tan(\operatorname{rad}(\texttt{FW\_R\_LIM}))}{V}
\right).
\]

\[
\mathcal R_L^{phys}=[0,r_{\max}^{eff}],
\qquad
\mathcal R_R^{phys}=[-r_{\max}^{eff},0].
\]

For direction \(d\), with \(\sigma_L=+1,\sigma_R=-1\):

\[
\rho=\frac{V}{r_{\max}^{eff}},
\quad
\mathbf c_d=\mathbf p+\sigma_d\rho\mathbf n_\perp,
\quad
\mathbf q_{j,d}=\mathbf c_d-\mathbf o_j,
\]

\[
h_{j,d}=\|\mathbf q_{j,d}\|-(C_j+\rho).
\]

The baseline margin terms are:

\[
C_j=R_{own,physical}+R_{threat,physical},
\qquad
h_{ref}=DSD=10\,m,
\qquad
\kappa=1/T.
\]

Uncertainty is not part of \(C_j\) or \(h_{ref}\). It remains in the existing
downstream MASD.

Using the derivative adapter from the V4 specification:

\[
\dot h=b_0+a_r r+a_a a_V^{ext}.
\]

The fixed exogenous acceleration is folded into the drift:

\[
b_{eff}=b_0+a_a a_V^{ext},
\qquad
a_r r\ge \kappa(h_{ref}-h)-b_{eff}.
\]

Each threat produces one affine interval constraint per direction. The core
intersects every threat constraint with that direction's physical interval.
Left and Right are never merged across an unsafe gap.

## 4. Data and sign contracts

### State sources

- Ownship position/course/timestamp: existing transformed
  `EstimatorTrajectoryBelief`.
- Ownship current \(V\): fresh `AirspeedValidated.true_airspeed_m_s`.
- TAS fallback: `airspeed_cruise`, mapped from `FW_AIRSPD_TRIM`, with explicit
  fallback status.
- Threat position/velocity/timestamp: peer-selected
  `ReceivedTrajectoryIntent`, sampled at the common evaluation timestamp.
- \(a_V^{ext}\): `0` supplied by
  `LongitudinalDriftSource::LocalOneStepFreeze` for the first baseline.

`airspeed_derivative_filtered` is IAS derivative and shall not be used as TAS
acceleration in this baseline.

### Sign conversion

V4 core:

\[
r_{V4}>0:\ Left,\qquad r_{V4}<0:\ Right.
\]

PX4/NED:

\[
r_{PX4,NED}=-r_{V4},
\qquad
a_{lat,PX4}=-V r_{V4}.
\]

Nominal inverse mapping:

\[
r_{nom,V4}=-a_{lat,nom,PX4}/V.
\]

These conversions live only in the candidate adapter.

## 5. Proposed ROS-independent APIs

### `SafeControlSetV4`

Files:

- `include/collision_avoidance/selection/SafeControlSetV4.hpp`
- `src/selection/SafeControlSetV4.cpp`

Fixed-size PODs:

```cpp
enum class SafeControlDirection : std::uint8_t { Left, Right };

enum class SafeControlSetStatus : std::uint8_t {
    Valid,
    InvalidConfiguration,
    InvalidOwnshipState,
    InvalidAirspeed,
    InvalidThreatState,
    FutureThreatTimestamp,
    StaleThreatTimestamp,
    DegenerateGeometry,
    LeftInfeasible,
    RightInfeasible,
    SearchSetInfeasible,
};

enum class LongitudinalDriftSource : std::uint8_t {
    ValidatedExternal,
    LocalOneStepFreeze,
};

struct HeadingRateInterval {
    double lower_radps;
    double upper_radps;
    bool feasible;
};

struct SafeControlOwnshipState {
    std::uint64_t timestamp_us;
    double north_m;
    double east_m;
    double heading_rad;
    double true_airspeed_mps;
    double longitudinal_acceleration_mps2;
    LongitudinalDriftSource longitudinal_source;
};

struct SafeControlThreatState {
    int vehicle_id;
    std::uint64_t timestamp_us;
    double north_m;
    double east_m;
    double velocity_north_mps;
    double velocity_east_mps;
    double physical_clearance_m;
};

struct SafeControlSetResult {
    SafeControlSetStatus status;
    HeadingRateInterval left_safe;
    HeadingRateInterval right_safe;
    double effective_max_heading_rate_radps;
    double kappa_per_s;
    double gamma_diagnostic;
    std::size_t evaluated_threat_count;
    // Fixed-size per-threat/per-direction diagnostics.
};
```

The core accepts a fixed-size threat array capped by the existing
`kMaximumSelectionAircraft - 1`; it does not allocate or use ROS types.

### `SafeControlCandidateAdapter`

Files:

- `include/collision_avoidance/selection/SafeControlCandidateAdapter.hpp`
- `src/selection/SafeControlCandidateAdapter.cpp`

Candidate roles are stable within one selection epoch:

```cpp
enum class SafeCandidateRole : std::uint8_t {
    NearNominal = 0,
    RobustLeft = 1,
    RobustRight = 2,
};

struct SafeControlCandidate {
    SafeCandidateRole role;
    double heading_rate_v4_radps;
    estimation::PredictInput predictor_input;
};
```

Rules:

1. Project a valid \(r_{nom}\) onto the union of the separate safe intervals.
2. Left representative is `left.upper - delta_r`; Right representative is
   `right.lower + delta_r`.
3. If an interval is narrower than `2 * delta_r`, use its midpoint rather than
   returning an endpoint outside the interval.
4. Suppress duplicates within `eps_interval`.
5. Never return more than three candidates.
6. If nominal is unavailable, omit `NearNominal`; core validity is unaffected.
7. Tie-breaking is deterministic: smaller distance to nominal, then smaller
   absolute rate, then `NearNominal`, Left, Right role order.
8. Convert \(r_{V4}\) to the existing PX4/predictor lateral-acceleration
   command exactly once with `a_lat_cmd = -V * r_v4`.
9. Preserve the current nominal ground-speed command and level
   `h_dot_cmd=0`; no climb candidate is introduced.

## 6. Existing architecture integration

### 6.1 Parameter path

Reuse `convert_px4_params.py` and the existing YAML path.

Add:

- `FW_Y_RMAX -> max_yaw_rate_deg_per_s`.
- `v4_safe_control_enabled`.
- `v4_shadow_only` (default `true` for first SILS).
- `v4_margin_time_constant_s` (default `5.0`).
- `v4_candidate_guard_deg_per_s` (default `0.5`).
- four numerical tolerance parameters.
- maximum TAS/nominal input age, defaulting to the existing 1 s belief-delay
  scale.

Reuse:

- `desired_separation_distance=10.0` as \(h_{ref}\).
- `aircraft_half_wingspan`.
- `max_roll_deg`.
- `airspeed_cruise`.
- `gravity`.

Do not expose independent `kappa` or `gamma`.

### 6.2 Runtime inputs

Extend the existing `DistributedManeuverSelectionRuntime`, not the node count:

- subscribe to `/px4_<id>/fmu/out/airspeed_validated_v1`;
- push timestamped TAS snapshots to `ManeuverSelectionWorker`;
- accept a timestamped nominal `FwSetpoint` snapshot from `FormationMode`;
- reuse the current single ROS executor as the sole SPSC producer.

`FormationMode::updateSetpoint()` runs on the main/executor thread and forwards
the latest nominal output. Do not push worker input directly from
`FormationMode::rt_loop()`, because that would create a second producer for the
existing SPSC queue.

### 6.3 Threat extraction

Expose or extract the current file-local intent interpolation helper so both
the activation monitor and V4 input adapter use one implementation.

For each peer:

1. require a verified peer ownship candidate/role;
2. select the matching current or previous remote intent cache entry;
3. require a consistent selection epoch and source timestamp;
4. interpolate it to the ownship evaluation timestamp;
5. reject future, stale, non-finite, or no-common-horizon input;
6. use its common-NED planar position/velocity.

No new threat velocity estimator is added.

### 6.4 Worker scheduling

- Compute the V4 safe intervals at the existing 20 Hz trajectory refresh.
- Recompute \(r_{\max}^{eff}\) from the newest TAS every update.
- Preserve the core result in worker output even in shadow mode.
- Generate candidate representatives only after a valid core result.
- Keep 4 Hz selection epochs and the existing 0.25 s coordination delay.
- When both families are infeasible, publish explicit
  `SEARCH_SET_INFEASIBLE`; do not manufacture an unsafe candidate.

### 6.5 Dynamic candidate packet

The existing fixed-LUT packet cannot reconstruct state covariance correctly
for per-epoch dynamic candidates. Extend the existing packet/message with the
four `PredictInput` scalar values. Continue sending only the compressed mean,
initial state, and covariance; do not send all 46 points.

Required changes:

- `TrajectoryIntentPacket` carries the actual candidate `PredictInput`.
- ROS `TrajectoryIntent.msg` carries the same four float32 values.
- sender accepts a candidate role plus actual input;
- receiver validates and uses the transmitted input for roll-state and
  covariance propagation;
- legacy fixed-LUT sender populates the same descriptor for compatibility.

Candidate ID in V4 mode identifies `NearNominal/RobustLeft/RobustRight`, not a
fixed roll degree.

### 6.6 Exact snapshot and active-command consistency

Dynamic role IDs make two extra invariants mandatory:

1. A proposal must identify each aircraft's candidate-set source timestamp, not
   only its 4 Hz epoch and role IDs. Same role ID with a different \(r\) is a
   different candidate snapshot.
2. Once activation latches a candidate, later 20 Hz refreshes must retain that
   aircraft's exact latched \(r\) and `PredictInput` for the active role.

Extend the existing proposal/decision message with the per-aircraft candidate
set timestamp/revision used for evaluation. Consensus requires epoch, role
tuple, and revisions to match. If they do not match, retain the previous
coordinated selection.

This prevents:

\[
\text{evaluated role = RobustLeft at }r_1
\quad\text{but executed role = RobustLeft at }r_2.
\]

### 6.7 Downstream cutover

Use two explicit modes:

1. `v4_shadow_only=true`
   - calculate/publish intervals and candidate diagnostics;
   - current V3/legacy candidate execution remains unchanged.
2. `v4_shadow_only=false`
   - V4 adapter supplies the maximum-three candidate set;
   - existing TPA/PMR/MASD/AD/Cost/coordination/activation operates on it;
   - legacy V3 sampled positive-margin gate is disabled for this path to avoid
     applying a second, semantically different barrier filter.

Configuration validation rejects simultaneous V4 cutover and V3 sampled-gate
activation. The V3 implementation remains available as a legacy comparison
during the validation phase and is not deleted in this task.

## 7. Diagnostics

Extend the existing `ManeuverSelectionDecision` data/ROS message rather than
adding a new ROS node or topic. Record at least:

- V4 validity/status.
- TAS source and TAS age.
- longitudinal drift source.
- \(r_{\max}^{eff}\).
- Left/Right feasible flags and bounds.
- generated candidate count, roles, and \(r\) values.
- first infeasible threat/direction and residual.
- candidate-set revisions used in proposal consensus.

These fields must be rosbag-visible for offline verification. They do not
replace PMR/MASD/AD diagnostics.

## 8. Files to change

### New

- `include/collision_avoidance/selection/SafeControlSetV4.hpp`
- `src/selection/SafeControlSetV4.cpp`
- `include/collision_avoidance/selection/SafeControlCandidateAdapter.hpp`
- `src/selection/SafeControlCandidateAdapter.cpp`
- `test/test_safe_control_set_v4.cpp`
- `test/test_safe_control_candidate_adapter.cpp`

### Existing

- `CMakeLists.txt`
- `tools/convert_px4_params.py`
- `config/airframe_spec.yaml`
- `config/flocking_params.yaml`
- `include/.../ManeuverSelectionWorker.hpp`
- `src/.../ManeuverSelectionWorker.cpp`
- `include/.../DistributedManeuverSelectionRuntime.hpp`
- `src/.../DistributedManeuverSelectionRuntime.cpp`
- `include/.../FormationMode.hpp`
- `src/.../FormationMode.cpp`
- `src/nodes/vtol_guidance_main.cpp`
- `include/.../TrajectoryIntent.hpp`
- `src/.../TrajectoryIntent.cpp`
- `include/.../TrajectoryIntentTransport.hpp` if its conversion signature
  changes
- `src/.../TrajectoryIntentTransport.cpp`
- `msg/TrajectoryIntent.msg`
- `msg/ManeuverSelectionDecision.msg`
- affected intent/worker/runtime/evaluator tests
- HILS guidance YAML and bag analyzer for new diagnostics

No files are added under a new ROS package or node.

## 9. Implementation sequence and gates

### Step 1 — Pure V4 core

Implement interval arithmetic, derivative terms, all-threat intersection,
status/diagnostics, parameter validation, and unit tests.

Gate:

- core and boundary tests pass;
- no ROS/PX4 includes in the core;
- Left and Right remain separate;
- no candidate, PMR, MASD, AD, Cost, or activation logic in the core.

### Step 2 — Candidate/sign adapter

Implement projection, guarded representatives, duplicate suppression, and
V4-to-PX4 conversion.

Gate:

- no output outside a safe interval;
- exact sign and round-trip tests pass;
- candidate count is 0..3;
- no climb command is introduced.

### Step 3 — Runtime inputs and shadow wiring

Add TAS and nominal snapshots, reuse intent time alignment, compute V4 at
20 Hz, and publish diagnostics with `v4_shadow_only=true`.

Gate:

- no change to executed maneuver commands;
- invalid/stale data produces explicit invalid status;
- actual TAS and trim fallback are distinguishable in rosbag;
- existing unit suite passes.

### Step 4 — Dynamic intent transport

Carry actual dynamic candidate input, use it at the receiver, add candidate
snapshot revisions, and verify active-input latching.

Gate:

- sender/ROS/receiver round trip preserves the candidate input;
- same role/different input cannot pass proposal consensus;
- active peer trajectory equals its exact latched command;
- 46-point trajectories are still reconstructed locally.

### Step 5 — V4 candidate cutover

Feed the maximum-three V4 candidates into the existing TPA/AD selection path
and disable the legacy V3 gate only for V4 cutover mode.

Gate:

- V4 infeasibility cannot be revived by best-unsafe AD fallback;
- PMR/MASD/AD uncertainty logic is unchanged;
- coordinated tuple/role/revision agreement is exact;
- activation remains downstream and retains the selected input.

### Step 6 — Headless SILS and offline review

Run one five-aircraft point-convergence case first in shadow mode, then one
explicit V4 cutover smoke only after shadow data is valid.

Use the existing script:

`testing_module/maneuver_selection_hils/scripts/run_point_convergence_case.sh`

Record and analyze:

- interval bounds and infeasibility rate;
- actual versus fallback TAS usage;
- generated candidate rates;
- proposal revision agreement;
- selected/latched/executed input equality;
- PMR, MASD, AD, actual minimum separation;
- process errors and queue drops.

The smoke proves wiring and behavior only. It does not prove DSD safety.

## 10. Unit and integration test matrix

### Core

- valid no-threat physical Left/Right intervals;
- one threat trims a lower or upper bound;
- multiple threats intersect the same direction;
- Left safe/Right unsafe and the reverse;
- both families infeasible;
- degenerate \(a_r\) with satisfied and violated constant constraints;
- \(\|\mathbf q\|\) degenerate;
- NaN/Inf, non-positive \(T\), invalid \(V\), and invalid limits;
- current TAS changes \(r_{\max}^{eff}\) on the next cycle;
- yaw-rate cap dominates bank cap and bank cap dominates yaw cap;
- nonzero supplied \(a_V^{ext}\) changes only the drift;
- local one-step freeze status is preserved;
- tolerance boundary behavior.

### Candidate adapter

- nominal already safe;
- nominal projected to nearest family;
- unsafe gap between Left/Right is not filled;
- guarded Left/Right values stay inside intervals;
- narrow interval midpoint fallback;
- duplicate suppression;
- missing nominal still yields valid directional candidates;
- maximum three candidates;
- positive V4 Left maps to negative PX4/NED lateral acceleration;
- inverse nominal mapping round trip.

### Time/data integration

- actual TAS, trim fallback, future TAS, stale TAS;
- future/stale/no-common-horizon peer intent rejection;
- peer-selected intent is used instead of an arbitrary alternate;
- no independent threat velocity estimate;
- same epoch but different candidate revision fails consensus;
- exact active input remains latched across 20 Hz refreshes.

### Transport/TPA

- dynamic `PredictInput` ROS round trip;
- receiver covariance propagation uses transmitted input, not local fixed LUT;
- compressed mean remains 18 floats and reconstructs 46 points;
- downstream uncertainty appears only once in MASD;
- V4-infeasible candidate never reaches AD fallback;
- feasible V4 candidates still use existing best-safe/best-unsafe AD behavior.

## 11. Adversarial acceptance audit

Implementation is rejected if any attack succeeds:

- Threat A constrains Left and Threat B constrains Right, but mixed directions
  produce one accepted interval.
- Left and Right are collapsed into one interval across an unsafe gap.
- A fixed roll ID is reported as the exact V4 representative when its rate is
  outside the safe interval.
- `FW_AIRSPD_TRIM` is reported as actual TAS.
- IAS derivative is silently used as TAS acceleration.
- PX4 positive lateral acceleration is treated as V4 Left.
- Candidate role IDs match but the compared trajectory input/revision differs.
- An active candidate role retains its ID while its actual \(r\) changes.
- V4 uncertainty is added to \(C_j\) and then subtracted again in MASD.
- V4 infeasibility is overwritten by the existing best-unsafe AD fallback.
- A stale/future threat produces a non-empty safe set.
- Shadow mode changes PX4 commands.
- SILS success is described as a formal 10 m or 95% safety guarantee.

## 12. Five-axis plan audit

| Axis | Result | Evidence |
|---|---|---|
| Source accuracy | PASS | The plan implements the scalar control-affine interval from the V4 documents, uses local PX4 source meanings, and labels the `FW_Y_RMAX` heading-rate use as a project cap rather than a PX4 identity claim. |
| No over-interpretation | PASS | no-wind, level, one-step freeze, non-robust intervals, and smoke-test limitations are explicit. No formal safety or uncertainty robustness is claimed. |
| Proportional complexity | PASS | It reuses the existing node, worker, queue, belief, intent reconstruction, covariance propagation, TPA, AD, coordination, activation, and HILS pipeline. Only the pure core, candidate adapter, missing TAS/nominal inputs, and unavoidable dynamic input descriptor are added. |
| Implementation correctness | PASS | Units, sign conversion, timing, all-threat intersection, dynamic packet identity, active latching, invalid-data handling, and tolerance boundaries are explicit test gates. |
| Directional alignment | PASS | The core preserves Left/Right safe intervals first; candidate representatives and existing Auto ACAS scoring/activation remain downstream. The V3 sampled filter is not renamed or mistaken for V4. |

All five axes pass at the plan level. Code implementation must repeat this audit
and the adversarial audit after tests and SILS evidence are available.

## 13. Step 1 implementation result — 2026-08-29

Step 1 is complete. Step 2 is recorded in Section 14. Steps 3 through 6 have
not started.

Implemented:

- ROS-independent `safe_control_set_v4` static library.
- Fixed-size ownship/threat input and diagnostic result PODs.
- Per-cycle effective heading-rate limit from yaw-rate and bank/current-TAS
  limits.
- Separate Left/Right physical intervals.
- Per-threat affine barrier construction and all-threat interval intersection.
- Explicit infeasible, invalid input, timestamp, and degenerate-geometry
  statuses.
- NED-to-internal algebraic sign conversion preserving `r_v4 > 0 = Left`.
- Local-one-step-freeze versus validated-external longitudinal drift status.
- Correct `d(rho)/dV` for both:
  - yaw-limited `rho=V/r_yaw`;
  - bank-limited `rho=V^2/(g*tan(phi_max))`.
- Fixed-size per-threat/per-direction diagnostics.

Not changed:

- ROS subscriptions and runtime worker.
- `FormationMode` and PX4 commands.
- Candidate generation/projection.
- Trajectory intent packet/message.
- TPA, PMR, MASD, AD, Cost, coordination, and activation.
- V3 sampled positive-margin filter.

Verification:

- `test_safe_control_set_v4`: 18/18 passed.
- Full `collision_avoidance` result: 106 tests, 0 errors, 0 failures,
  0 skipped.
- `git diff --check`: passed.
- The ROS runtime test initially could not load the already-built PX4 message
  Fast DDS type-support library because the parent PX4 ROS workspace had not
  been sourced. After sourcing
  `/home/hmcl/workspace/swarm-fixed-wing/ros2_ws/install/setup.bash`, the same
  runtime tests and full suite passed.

Step 1 adversarial findings corrected before acceptance:

1. The first draft used the constant-\(r_{max}\) radius derivative for a
   bank-limited \(r_{max}(V)\). It was corrected to use
   `d(rho)/dV=2/r_max` on the bank-limited branch and `1/r_max` on the
   yaw-limited branch.
2. A tolerance-sized interval inversion was initially collapsed by averaging,
   which could place the result infinitesimally outside the physical domain.
   It now collapses to the existing physical boundary.
3. `LocalOneStepFreeze` paired with nonzero longitudinal acceleration is now
   rejected as an inconsistent input.

### Step 1 five-axis audit

| Axis | Result | Evidence |
|---|---|---|
| Source accuracy | PASS | The V4 affine derivative and the specification's state-dependent-radius warning are both reflected; yaw- and bank-limited radius derivatives have separate tests. |
| No over-interpretation | PASS | The module returns instantaneous planar safe intervals and diagnostics only. It makes no runtime, uncertainty-robustness, forward-invariance, or DSD guarantee. |
| Proportional complexity | PASS | One pure library, one header/source pair, and one test target were added. There is no ROS dependency, allocation, thread, solver, candidate layer, or duplicate TPA path. |
| Implementation correctness | PASS | 18 focused tests cover NED signs, all-threat intersection, no Left/Right mixing, infeasibility, limit selection, speed updates, drift branches, timestamps, degenerate geometry, tolerances, and invalid inputs. |
| Directional alignment | PASS | The output remains the two separate Left/Right intervals; downstream candidate and Auto ACAS layers were not pulled into the core. |

Step 2 may begin only from this accepted core boundary.

## 14. Step 2 implementation result — 2026-08-29

Step 2 is complete. Steps 3 through 6 have not started.

Implemented:

- A separate ROS-independent `safe_control_candidate_adapter` static library.
- Fixed-size output with no allocation and at most three candidates in stable
  role order: `NearNominal`, `RobustLeft`, `RobustRight`.
- Projection of a valid nominal V4 heading rate onto the union of the still
  separate Left/Right safe intervals.
- Deterministic projection tie-breaking by distance, absolute rate, then Left
  before Right.
- Left `upper - delta_r` and Right `lower + delta_r` robust representatives.
- Midpoint fallback when an interval is narrower than `2 * delta_r`.
- Duplicate suppression without manufacturing a replacement candidate.
- Explicit `SEARCH_SET_INFEASIBLE` propagation with zero candidates.
- V4/PX4 sign conversion in the adapter only:
  `a_lat_cmd = -V_TAS * r_v4` and its inverse.
- Preservation of the existing ground-speed and altitude commands with
  `h_dot_cmd=0` for every generated candidate.
- Validation for non-finite commands, airspeed, interval bounds, family signs,
  physical rate bounds, conversion overflow, and adapter parameters.

Not changed:

- ROS subscriptions, worker queues, and runtime scheduling.
- `FormationMode` or any executed PX4 command.
- Trajectory intent packets/messages or dynamic candidate transport.
- TPA, PMR, MASD, AD, Cost, coordination, and activation.
- V3 sampled positive-margin filtering.
- TAS/nominal snapshot acquisition and shadow diagnostics; these remain Step 3.

Verification:

- `test_safe_control_candidate_adapter`: 15/15 passed.
- Full `collision_avoidance` result: 122 tests, 0 errors, 0 failures,
  0 skipped.
- `git diff --check`: passed.
- The adapter header/source contain no ROS or PX4 message include.
- Existing `PredictTypes.hpp` received only its missing direct `<cstddef>`
  include so it remains self-contained when the adapter uses `PredictInput`.

Step 2 adversarial findings corrected before acceptance:

1. The first CMake draft compiled the adapter into the Step 1 core target.
   The adapter is now a separate static library linked to the unchanged core
   target, preserving the core/candidate boundary.
2. A zero robustness guard and tolerance-sized family-sign violations were
   initially accepted. The guard must now be positive and feasible intervals
   must remain strictly inside their Left or Right physical domains.
3. The first inverse sign-conversion implementation could return infinity for
   finite but extreme inputs. Both conversion directions now reject non-finite
   results with `NaN`, and the overflow boundary is tested.

### Step 2 five-axis audit

Audit scope:

- Source: `codex_v4_implementation_decisions_v2.md`, especially the nominal
  projection, maximum-three candidate, guard, and sign-contract sections.
- Derived contract: Sections 4, 5, 9, and 10 of this plan.
- Implementation: `SafeControlCandidateAdapter.hpp/.cpp`, its CMake target,
  and `test_safe_control_candidate_adapter.cpp`.
- Excluded: runtime data acquisition, packet transport, downstream scoring,
  activation, and SILS claims.

| Axis | Result | Evidence |
|---|---|---|
| Source accuracy | PASS | The implementation uses the documented project design of nominal projection plus separate Left/Right representatives, and the locally investigated sign equations `a_lat=-V_TAS*r_v4` and `r_v4=-a_lat/V_TAS`. It does not label this projection as an original Auto ACAS equation. |
| No over-interpretation | PASS | The adapter returns discrete predictor inputs only. It makes no claim of forward invariance, uncertainty robustness, DSD satisfaction, activation behavior, or SILS safety. Missing nominal input omits only `NearNominal`. |
| Proportional complexity | PASS | One pure fixed-size adapter library and one test target were added. It reuses `SafeControlSetV4Result` and the existing `PredictInput`; there is no ROS node, allocation, optimizer, packet path, trajectory generator, or downstream evaluator duplication. |
| Implementation correctness | PASS | Tests cover safe/unsafe-gap projection, deterministic tie handling, guarded endpoints, midpoint fallback, duplicate suppression, one-family and no-family cases, maximum-three count, exact sign/round-trip, command preservation, no climb, NaN fallback, invalid input, and conversion overflow. The complete 122-test package suite passes. |
| Directional alignment | PASS | Left and Right intervals are never merged; every candidate remains within one feasible family, and candidate selection remains an adapter downstream of the V4 core and upstream of the unchanged TPA/AD layers. |

Cross-axis dependencies: none remain. All evidence required for Step 2 is
available, and every applicable axis passes. Runtime correctness and SILS
safety remain explicitly unevaluated until Steps 3 through 6.
