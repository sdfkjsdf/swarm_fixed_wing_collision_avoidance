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

### 6.6 Command identity and owner-canonical snapshot consistency

Dynamic role IDs make two extra invariants mandatory:

1. A proposal identifies each aircraft's candidate-set source timestamp for
   traceability, but command consensus is the 4 Hz epoch, mode, role ID, and
   actual input revision. The aircraft that owns a candidate is authoritative
   for the committed source timestamp of its own slot.
2. Once activation latches a candidate, later 20 Hz refreshes must retain that
   aircraft's exact latched \(r\) and `PredictInput` for the active role.

Extend the existing proposal/decision message with the per-aircraft candidate
set timestamp/revision used for evaluation. Consensus requires epoch, mode,
role tuple, and revisions to match. Different 20 Hz receive snapshots of the
same command do not invalidate consensus; committed timestamps are assembled
from each owner's proposal. If command identity does not match, retain the
previous coordinated selection.

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

Step 1 is complete. Steps 2 and 3 are recorded in Sections 14 and 15.
Steps 4 through 6 have not started.

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

Step 2 is complete. Step 3 is recorded in Section 15. Steps 4 through 6 have
not started.

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

## 15. Step 3 implementation result — 2026-08-29

Step 3 is complete. Steps 4 through 6 have not started.

Implemented:

- The existing distributed runtime conditionally subscribes to ownship PX4
  `AirspeedValidated` only when V4 is enabled and forwards timestamped TAS,
  PX4 source, and validity to the existing worker queue.
- `FormationMode::updateSetpoint()` forwards the latest pre-avoidance nominal
  ground-speed, altitude, and lateral-acceleration command from the main ROS
  executor thread. The RT loop remains a single producer of its existing
  control-output queue and does not push the worker queue.
- The worker classifies TAS and nominal snapshots as Missing, Valid, Invalid,
  Future, or Stale. Invalid/non-current TAS uses the configured trim fallback
  with a distinct source status; invalid/non-current nominal input omits only
  `NearNominal`.
- At the existing 20 Hz trajectory refresh, the worker resolves every verified
  peer's selected fixed-LUT candidate from the current/previous complete intent
  cache, reuses the existing interpolation helper to align it to current
  ownship time, and rejects missing, future, stale, or invalid peer data with an
  explicit shadow status.
- The aligned common-NED planar state is passed to the Step-1 core with
  `LOCAL_ONE_STEP_FREEZE` and `a_V_ext=0`. Current TAS recomputes the effective
  heading-rate limit every update.
- The Step-2 adapter produces at most three diagnostic candidates after a
  valid/infeasible core evaluation. These candidates are not supplied to the
  legacy TPA or command path in Step 3.
- The existing `ManeuverSelectionDecision` ROS message carries rosbag-visible
  V4 validity, input source/age, core intervals, effective limit, first
  infeasible residual, and candidate role/rate diagnostics.
- The existing point-convergence bag analyzer summarizes the new fields. No
  new node, topic family, worker thread, threat estimator, coordinate frame, or
  trajectory evaluator was added.
- `FW_Y_RMAX` is mapped through the existing PX4-export converter and is used
  only as the documented project upper cap. Existing DSD, half-wingspan,
  gravity, roll, trim-airspeed, and YAML paths are reused.

Not changed:

- Executed `FwSetpoint`/PX4 commands, activation, and legacy candidate choice.
- Fixed-LUT candidate IDs or trajectory-intent packet/message contents.
- Trajectory reconstruction, covariance propagation, PMR, MASD, AD, Cost, and
  uncertainty placement.
- V3 sampled positive-margin filtering.
- Wind-aware air/ground conversion, TAS acceleration estimation, robust V4
  covariance margins, or vertical avoidance.
- Dynamic candidate input/revision transport (Step 4), V4 candidate cutover
  (Step 5), and headless SILS evidence (Step 6).

Verification:

- `test_maneuver_selection_worker`: 18/18 passed, including actual/trim TAS,
  stale/future inputs, invalid nominal, selected-peer alignment, 20 Hz
  TAS-dependent limit refresh, and rejection of command-cutover configuration.
- `test_distributed_maneuver_selection_runtime`: ROS TAS input and serialized
  V4 decision diagnostics passed; the complete binary passed 10/10 repeated
  executions after the timing fixture correction below.
- Full `collision_avoidance` result: 128 tests, 0 errors, 0 failures, 0 skipped.
- Bag analyzer `py_compile`: passed.
- `git diff --check`: passed.
- Runtime tests require the parent PX4 ROS workspace to be sourced before this
  overlay so the PX4 Fast DDS type-support library is available. This is a
  test environment dependency, not a V4 fallback.

Step 3 adversarial findings corrected before acceptance:

1. The first wiring subscribed/pushed TAS and nominal inputs even when V4 was
   disabled. Both paths are now conditional on `v4_safe_control_enabled`.
2. The initial tests covered stale TAS but not future TAS or the next-cycle
   TAS-dependent limit update. Explicit future/invalid classification and
   20 Hz `r_max_eff` recomputation checks were added.
3. The first ROS integration fixture stopped belief updates immediately after
   coordination. In some schedules, the verified peer decision arrived after
   the last 20 Hz evaluation, so the last diagnostic correctly remained
   `MISSING_PEER_DECISION`. The fixture now supplies two post-consensus refresh
   ticks; 10 repeated runs pass.
4. Step-3 rosbag diagnostics require the first infeasible residual, but the
   pure core previously exposed only the imposed bound. A fixed-size positive
   constraint-shortfall diagnostic was added without altering interval results.
5. The Step-2 adapter target was linkable inside the build tree but absent from
   the install target list. Runtime wiring now links it explicitly and installs
   the same existing library target.

### Step 3 five-axis audit

Audit scope:

- Source/decision contract:
  `reference/paper/MD_FILES/codex_v4_implementation_decisions_v2.md`, especially
  the investigated TAS, one-step-freeze, nominal, threat-alignment, physical
  clearance, and downstream uncertainty decisions.
- Derived contract: Sections 6.1 through 6.4, 7, 9 Step 3, 10, and 11 of this
  plan.
- Implementation: worker/runtime/Formation wiring, V4 decision diagnostics,
  parameter mapping, analyzer, and affected tests.
- Excluded from a PASS claim: dynamic candidate identity/transport, command
  cutover, SILS behavior, formal invariance, uncertainty robustness, and 10 m
  separation assurance.

| Axis | Result | Evidence |
|---|---|---|
| Source accuracy | PASS | The runtime uses `true_airspeed_m_s` with PX4 source/timestamp, identifies trim fallback separately, does not use IAS derivative, supplies explicit one-step freeze, reuses the investigated nominal sign adapter, selected reconstructed intent, physical half-wingspan sum, and existing DSD parameter. Uncertainty remains downstream in MASD. |
| No over-interpretation | PASS | Results are named shadow diagnostics and are never described as a safety proof. The documented no-wind/level threat approximation, non-robust core, trim fallback, and one-step acceleration freeze remain limitations. Dynamic candidates and SILS claims are explicitly deferred. |
| Proportional complexity | PASS | The change reuses the existing node, runtime, worker thread, SPSC queue, common belief, remote intent cache, interpolation, decision topic, YAML converter, and analyzer. Conditional input wiring avoids V4-disabled overhead; no new node/thread/topic family/estimator/evaluator was created. |
| Implementation correctness | PASS | Focused tests cover valid, stale, future, missing, and invalid inputs; actual versus trim TAS; nominal omission; selected-peer time alignment; per-cycle limit recomputation; ROS serialization; shadow-only enforcement; and repeated runtime scheduling. The full 128-test suite passes. |
| Directional alignment | PASS | Step 3 preserves the separate Left/Right core intervals and only publishes maximum-three adapter candidates. It does not feed them into TPA, alter legacy commands, move uncertainty into V4, rename V3 as V4, or bypass downstream PMR/MASD/AD/activation. |

All five axes pass for the bounded Step-3 shadow-wiring scope. Step 4 must add
exact dynamic input/revision identity before any V4 candidate can enter the
selection or activation path.

## 16. Step 4 implementation result — 2026-08-29

Step 4 is complete. Steps 5 and 6 have not started.

Implemented:

- The existing compact `TrajectoryIntentPacket` and ROS message now carry the
  actual four-float `PredictInput` descriptor
  `[V_cmd, h_cmd, h_dot_cmd, a_lat_cmd]` and a deterministic input revision.
- The legacy fixed-LUT sender uses the same dynamic-packet builder, so legacy
  and future V4 candidates share one transport and reconstruction path.
- The sender predicts from the transmitted float32 representation. This keeps
  the compressed mean and the receiver's propagated input semantics identical
  across the ROS boundary.
- The receiver no longer owns or consults a fixed candidate lookup table. It
  validates the transmitted input and its revision, reconstructs the existing
  46-point mean locally from the unchanged 18-float compressed mean, and uses
  that input for roll-state and covariance propagation.
- Distributed proposals now identify, for every participating aircraft:
  candidate ID, candidate-input revision, and the exact trajectory source
  timestamp used by the evaluator.
- Proposal consensus requires the role and input-revision arrays plus the
  selection epoch to match. Source timestamps remain visible and are
  canonicalized from each candidate owner's proposal. A same-role proposal
  with a different input revision retains the last coordinated selection.
- A committed selection stores the same revision/timestamp arrays in the
  existing decision message, making snapshot agreement rosbag-visible.
- Activation now latches the selected candidate's input revision together
  with its actual `PredictInput`. Active ownship and peer trajectories are
  resolved by ID plus revision; a later same-ID/different-input trajectory is
  not silently substituted.
- The V4 Step-3 shadow path also resolves a peer's selected intent by ID plus
  the peer's committed input revision.

The revision is a deterministic integrity identity for the transmitted
candidate ID and float32 input values. It is not a cryptographic authentication
mechanism or a safety guarantee.

Not changed:

- V4 diagnostic candidates are not yet supplied to TPA or activation.
- `FormationMode`, `FwSetpoint`, and every executed PX4 command are unchanged.
- The 4 Hz selection and existing 20 Hz trajectory-refresh schedule are
  unchanged.
- Compressed-mean contents, 4.5 s horizon, 0.1 s grid, local 46-point
  reconstruction, covariance equations, PMR, MASD, AD, Cost, DSD, and
  best-unsafe fallback are unchanged.
- No node, topic family, worker thread, estimator, evaluator, optimizer, or
  46-point network payload was added.
- V4 candidate cutover and headless SILS remain Steps 5 and 6.

Verification:

- `test_trajectory_intent`: 5/5 passed, including a dynamic input whose ID is
  the legacy zero-roll ID, 46-point reconstruction, and rejection after input
  tampering without a matching revision.
- `test_trajectory_intent_transport`: ROS conversion preserves all four input
  scalars and the revision.
- `test_maneuver_activation_controller`: the exact input revision and command
  remain latched across later selection changes.
- `test_maneuver_selection_worker`: 18/18 passed, including rejection of the
  same candidate role with a changed input revision while preserving the
  previous committed epoch and active-ownship latching across a new epoch.
- Two- and five-aircraft worker/runtime tests confirm identical committed role,
  input-revision, and source-timestamp arrays on every participant.
- Full `collision_avoidance` result: 129 tests, 0 errors, 0 failures,
  0 skipped.
- `git diff --check`: passed.
- ROS runtime tests require the parent PX4 ROS workspace overlay for the PX4
  Fast DDS type-support library, as already recorded for Step 3.

Step 4 adversarial findings corrected before acceptance:

1. The old receiver reconstructed a candidate's input from the local fixed
   roll table. The table dependency was removed from the receiver, so an ID can
   no longer overwrite the transmitted dynamic input.
2. Candidate revision is computed after float32 wire quantization, and the
   sender uses that same quantized input for prediction. This prevents the
   sender mean and receiver covariance path from using subtly different input
   values.
3. Role tuple equality alone was insufficient. Consensus compares the
   per-aircraft input revisions. Step 6 later corrected the initially
   over-conservative requirement for all observers' 20 Hz source timestamps
   to match exactly and uses owner-canonical timestamps instead.
4. Activation previously looked the command up again by fixed candidate ID.
   It now obtains the input from the selected reconstructed intent and latches
   that intent's revision and input together.
5. One old unit fixture described a peer as already coordinated while changing
   only that peer's own candidate ID to an uncommitted proposal. The new peer
   consistency validation correctly rejected this impossible state; the
   fixture now labels it as proposal-before-commit.

### Step 4 five-axis audit

Audit scope:

- Source/decision contract:
  `reference/paper/MD_FILES/codex_v4_implementation_decisions_v2.md`, Section 5,
  which requires a four-scalar dynamic descriptor rather than 46 transmitted
  points.
- Derived contract: Sections 6.5, 6.6, 9 Step 4, 10, and 11 of this plan.
- Implementation: compact intent sender/receiver, ROS intent and decision
  transport, proposal consensus identity, activation latch, and affected tests.
- Excluded from a PASS claim: V4-to-TPA cutover, actual V4 command execution,
  SILS behavior, formal invariance, DSD assurance, and cryptographic integrity.

| Axis | Result | Evidence |
|---|---|---|
| Source accuracy | PASS | The existing source investigation explicitly requires `timestamp + candidate role + x0 + P0 + compressed mean + PredictInput`, receiver-side covariance propagation from the actual input, and no 46-point payload. The implementation preserves each listed field and does not claim that this project transport detail is a disclosed Auto ACAS wire format. |
| No over-interpretation | PASS | Step 4 establishes transport and identity consistency only. It makes no collision-avoidance, DSD, 95% containment, forward-invariance, security, command-execution, or SILS claim. |
| Proportional complexity | PASS | The existing packet, ROS messages, sender, receiver, runtime subscriptions, worker proposal, and activation controller are extended in place. The revision is the command identity and timestamps remain trace evidence; no parallel transport or coordination path was added. |
| Implementation correctness | PASS | Sender and receiver use the same float32 input, receiver rejects an input/revision mismatch, consensus compares epoch/mode/ID/revision, owner-canonical source times are committed, active lookup requires ID/revision, and activation latches the intent input. Focused, distributed, ROS, and full-suite tests pass. |
| Directional alignment | PASS | The change enables the documented dynamic-candidate contract while leaving V4 cutover downstream, preserving local 46-point reconstruction and the existing TPA/PMR/MASD/AD/activation architecture. |

Cross-axis dependencies: none remain for the bounded Step-4 transport and
identity scope. Step 5 may begin only from this accepted boundary and must use
this dynamic builder rather than reintroducing a role-to-fixed-roll lookup.

## 17. Step 5 implementation result — 2026-08-29

Step 5 is complete. Headless SILS remains Step 6 and has not been run as part
of this step.

Implemented:

- `v4_shadow_only=false` now enables an explicit V4 cutover mode. The worker
  uses the existing 20 Hz V4 core/adapter output as the candidate source for
  the existing trajectory reconstruction, joint PMR/MASD/AD scoring,
  coordination, and activation path.
- V4 candidate IDs retain their role meaning
  `NearNominal/RobustLeft/RobustRight`; the transmitted input and revision are
  still the executable identity. No role is converted back through the fixed
  roll lookup table.
- The compact intent packet carries two set-level fields: candidate-set kind
  (`legacy` or `V4`) and unique set size. This is required because duplicate
  suppression produces one to three V4 candidates rather than a fixed three.
- The existing joint evaluator has a candidate-count overload. It evaluates
  the product of the actual per-aircraft counts while using the unchanged
  pair evaluator and unchanged PMR/MASD/AD equations.
- Proposal/selection decisions identify whether the tuple is a V4 cutover
  tuple. Consensus requires that mode flag together with the existing epoch,
  role IDs, and input revisions; source timestamps are committed from the
  aircraft that owns each selected slot.
- Before the first verified peer selection exists, cutover mode retains the
  existing legacy candidate exchange only as a non-executing bootstrap. A
  legacy bootstrap tuple is never eligible for activation in cutover mode.
  Once one valid V4 set has been generated, the worker never falls back to
  legacy candidates.
- A V4 `SEARCH_SET_INFEASIBLE` result supplies zero new candidates. Therefore
  the downstream evaluator cannot revive it through its best-unsafe AD
  fallback. A previously coordinated Current Best continues to be repredicted;
  if active, its exact latched role/input/revision is additionally constrained,
  as required by the existing hold semantics.
- During the legacy-to-V4 handoff, the remote cache retains the exact
  peer-selected trajectory input until the peer publishes a coordinated V4
  selection. This prevents the 20 Hz V4 refresh from erasing the trajectory
  that the peer decision still names.
- Configuration validation rejects V4 cutover together with the legacy V3
  sampled positive-margin filter. It also rejects cutover together with the
  exhaustive seven-roll test mode; both legacy paths remain available when
  cutover is not enabled.

Not changed:

- Shadow mode remains the default in both checked-in YAML files.
- The V4 scalar core, candidate projection/sign conversion, ground-kinematic
  predictor, 4.5 s horizon, 0.1 s grid, compressed 18-float mean, local
  46-point reconstruction, and covariance propagation are unchanged.
- PMR window search, physical-size margin, DSD, 95% covariance projection,
  MASD, AD, reciprocal cost, and the feasible-candidate best-unsafe fallback
  are unchanged.
- No ROS node, topic family, worker thread, estimator, Track Manager,
  optimizer, or parallel evaluator was added.
- This step does not establish formal invariance, robust uncertainty
  guarantees, 10 m separation assurance, 95% trajectory containment, or SILS
  behavior.

Verification:

- `test_maneuver_selection_worker` now covers:
  - legacy bootstrap consensus without activation;
  - transition to V4 role/input/revision consensus;
  - downstream activation only after a V4 tuple is committed;
  - exact active V4 input revision retention across a changed nominal input;
  - explicit V4 infeasibility with zero emitted candidate packets and no
    legacy/best-unsafe revival;
  - rejection of V4 cutover plus the legacy positive-margin gate.
- `test_maneuver_combination_evaluator` verifies mixed per-aircraft candidate
  counts and the exact product-sized combination space.
- `test_trajectory_intent_transport` verifies candidate-set size/kind through
  the ROS conversion together with the dynamic input and revision.
- Full `collision_avoidance` result: 132 tests, 0 errors, 0 failures,
  0 skipped.
- `git diff --check`: passed.
- As in Steps 3 and 4, ROS runtime tests require the parent PX4 ROS workspace
  overlay to be sourced so the PX4 Fast DDS type-support library is visible.

### Step 5 adversarial audit

| Attack | Result | Evidence |
|---|---|---|
| Core returns `SEARCH_SET_INFEASIBLE`, but legacy or AD best-unsafe candidates reappear | PASS | After the first V4 set, invalid/infeasible generation clears current selection candidates and emits no packets. The dedicated three-aircraft worker test verifies zero candidate count, zero packet count, no proposal, and no activation. |
| A legacy bootstrap tuple commands the aircraft before V4 is ready | PASS | Cutover activation resets/returns unless the committed tuple carries `selected_v4_cutover=true`; the two-aircraft test commits a legacy bootstrap tuple and observes no activation. |
| Two nodes agree on IDs but one evaluates legacy semantics and the other V4 roles | PASS | Candidate-set kind is transported, all evaluated sets must have one kind, and proposal consensus compares `proposed_v4_cutover`. |
| Duplicate suppression silently restores a fixed three-candidate assumption | PASS | Packet set size is validated, remote completion uses that size, and the joint evaluator decodes mixed-radix slots from per-aircraft counts. |
| An active role keeps its ID but changes its actual input at 20 Hz | PASS | V4 set construction replaces the regenerated role input with the activation controller's exact latched input; the test changes nominal input and observes the original selected revision. |
| V3 sampled filtering and V4 interval filtering both gate the same cutover | PASS | Parameter validation rejects the simultaneous configuration. The legacy V3 code remains available only outside V4 cutover. |
| V4 uncertainty is added and then subtracted again in MASD | PASS | Step 5 does not alter `evaluatePairImpl()` or the core constraints; covariance remains only in the existing MASD calculation. |
| A malformed V4 packet claims more than three roles or a non-V4 role ID | PASS | Remote packet validation requires set size 1..3 and role ID 0..2 before reconstruction/cache admission. |

### Step 5 five-axis source audit

Audit scope:

- Project-authoritative decision source:
  `reference/paper/MD_FILES/codex_v4_implementation_decisions_v2.md`,
  especially the investigated existing TPA interface, dynamic descriptor
  contract, sign decision, uncertainty placement, and final implementation
  boundary.
- Derived requirements: Sections 6.4 through 6.7, 9 Step 5, 10, and 11 of
  this plan.
- Implementation: compact intent set metadata, worker cutover/readiness/cache
  logic, variable-count joint-evaluator overload, decision transport, and
  focused/full regression tests.
- Exclusions: Step-6 SILS, formal or robust safety proofs, disclosed Auto ACAS
  wire-format equivalence, DSD guarantee, and 95% containment claims.

Evidence baseline:

1. The decision source requires maximum-three safe candidates to enter the
   existing TPA/AD/activation hierarchy through the actual dynamic
   `PredictInput`, while keeping 46-point reconstruction local and uncertainty
   in downstream MASD.
2. The derived Step-5 contract requires V4 infeasibility to remain terminal
   for candidate generation, disables the V3 sampled gate in cutover, and
   preserves exact role/input/revision coordination plus active input latching.
3. The implementation supplies only validated V4 sets to the existing joint
   evaluator, preserves feasible-set best-unsafe AD behavior, blocks fallback
   when no V4 set exists, and retains the exact active input.

| Axis | Status | Source evidence | Implementation evidence | Reason / impact | Confidence |
|---:|---|---|---|---|---|
| 1. Source accuracy and traceability | PASS | The decision source Section 5 states `timestamp + candidate role + x0 + P0 + compressed mean + PredictInput`; its final boundary requires existing worker/TPA/AD integration. Plan Sections 6.4–6.7 make the cutover and infeasibility gates explicit. | Intent packets retain the compact mean and add only set metadata; V4 roles use the Step-4 dynamic input builder; existing evaluator/activation objects are reused. | The implementation follows the project-authoritative contract and does not misrepresent these project-specific transport details as a published Auto ACAS packet format. | High |
| 2. Interpretation fidelity | PASS | The source and plan limit the baseline to non-robust safe intervals upstream of uncertainty-aware MASD and require explicit infeasibility rather than a guarantee. | Cutover diagnostics retain `v4_shadow_only`; documentation excludes proofs/SILS/DSD guarantees; infeasible sets emit no new candidates, while feasible candidates still use existing AD policy. | No example or qualitative source statement is promoted to a safety guarantee. The non-executing legacy bootstrap is identified as a project readiness mechanism, not a source requirement. | High |
| 3. Complexity proportionality | PASS | Variable 0..3 candidates, exact dynamic identity, selected-peer time alignment, and retained active input are named integration constraints. | Existing packet/message/cache/worker/evaluator are extended in place. No new node, topic, thread, estimator, optimizer, or duplicate PMR/MASD/AD path exists. Set size/kind and one evaluator overload are the minimum additions needed to avoid fixed-three and semantic-alias errors. | The added state is justified by cold-start readiness, variable candidate count, and exact active-command consistency; no materially separate architecture was introduced. | High |
| 4. Correct implementation under aligned direction | PASS | The gate requires no V3/V4 double filtering, exact command role/revision agreement, unchanged uncertainty logic, and no infeasible fallback. | Parameter validation enforces mutual exclusion; cache/evaluator use actual counts; consensus compares V4 mode plus ID/revision and owner-canonicalizes source timestamps; selected and active refresh retain the chosen revision; focused tests and the full suite pass. | Candidate identity, candidate-count indexing, fallback, and activation boundary conditions have direct regression evidence. PMR/MASD/AD formulas were not modified. | High |
| 5. Directional alignment | PASS | The governing direction is continuous Left/Right V4 safe-set generation first, then candidate representation, existing TPA/PMR/MASD/AD/Cost, coordination, and activation. | `SafeControlSetV4` remains upstream; its adapter supplies V4 role/input packets; the existing downstream evaluators and activation controller remain authoritative. | The implementation neither relabels V3 as V4 nor replaces the source-directed hierarchy with a new optimizer or trajectory-level uncertainty gate. | High |

Cross-axis dependencies: none remain for the bounded Step-5 code cutover.
All applicable axes pass. The next admissible step is Step 6 headless SILS and
offline evidence collection; a passing Step 5 does not prejudge that result.

## 18. Step 6 headless SILS result — 2026-08-29

Step 6 is complete as a cutover wiring and behavior smoke. It is not a DSD
safety acceptance: the final run violated the 10 m DSD and therefore opens a
separate performance-remediation step.

### Harness and offline evidence changes

- The existing point-convergence runner now accepts `V4_MODE=shadow|cutover`,
  rejects cutover with the legacy exhaustive mode, sources the current
  collision-avoidance overlay, and passes the V4/V3 gate parameters
  explicitly. The simulator remains headless.
- The bag processor sources the same overlay before deserialization.
- The existing analyzer reads decision and trajectory-intent topics, reports
  V4 mode/role/revision identity, distinguishes command agreement from exact
  20 Hz snapshot agreement, edge-detects activation, checks active role and
  revision retention, validates variable-size V4 packet metadata, and verifies
  every selected owner intent against the transmitted packet.
- `FormationMode` override logs are cross-checked against the exact selected
  packet's candidate ID, `V_cmd`, and `a_lat_cmd`. This proves the command
  handed to the existing PX4 setpoint adapter; it does not prove PX4 plant
  tracking or datalink delivery beyond that adapter call.

### Failures found and corrected during SILS

1. `step6_v4_cutover_20260829_01` never reached V4 cutover. One full
   `previous` cache was serving both selected-intent retention and previous
   4 Hz epoch retention, so a legacy selected set displaced the V4 set needed
   at an asynchronous epoch boundary. The roles were separated: current and
   previous full sets remain coordination caches, while only one selected
   intent is retained. A regression test advances the peer across the boundary
   first.
2. `step6_v4_cutover_20260829_02` committed V4 tuples but never activated.
   The next 20 Hz refresh removed the exact selected input revision before the
   activation monitor could resolve it. The selected Current Best is now
   repredicted with the same input/revision even while activation is not yet
   active. The existing infeasible-set test still proves zero *new* candidates
   and no legacy/best-unsafe revival.
3. `step6_v4_cutover_20260829_03` activated only after the closest pass in that
   run. Of 64 common proposal epochs, 59 agreed on mode/IDs/input revisions but
   only 25 also had identical observer-side 20 Hz source timestamps. Requiring
   the latter was an unsupported exact-snapshot expansion. Consensus now uses
   executable command identity and commits each source timestamp from the
   aircraft that owns that slot. A test injects different observer-side source
   timestamps and verifies one owner-canonical committed array.
4. Diagnostic publications could repeat a prior `activation_just_started` or
   `activation_just_ended` flag. V4 diagnostic output now clears these
   one-output events, and offline summaries count real false/true edges rather
   than raw repeated flags.

### Final cutover smoke

Final run:
`step6_v4_cutover_20260829_04`

Artifacts:

- rosbag: `result/rosbag/step6_v4_cutover_20260829_04`
- summary: `result/summary/step6_v4_cutover_20260829_04/summary.json`
- plot: `result/plot/step6_v4_cutover_20260829_04/actual_maneuver_overview.png`
- video: `result/video/step6_v4_cutover_20260829_04/actual_maneuver.mp4`

Wiring/identity evidence:

- All five aircraft were V4-cutover selected for every post-gate decision
  record (`814..867` records per aircraft); every record also had a valid V4
  proposal.
- All 138 common committed epochs had the exact same mode/ID/revision and
  owner-canonical source-timestamp tuple.
- 138/146 common proposal epochs agreed on command identity. Exact observer
  snapshot agreement was only 73/146, which is retained as a latency
  diagnostic rather than used as command consensus.
- Selected intent references were missing in 0 records for all five aircraft;
  malformed V4 packet metadata count was 0.
- Every aircraft activated before the 17.5 s closest pass; first activation
  occurred at 6.06..6.95 s. Active candidate switch count and active input
  revision switch count were both 0 for all aircraft.
- Throttled Formation override evidence matched the exact selected intent in
  `14/14`, `17/17`, `13/13`, `13/13`, and `14/14` records. Missing or inactive
  references were 0.
- No queue-full/drop, process error, exception, segmentation, or terminate
  diagnostic was present in the five guidance logs.

Behavior evidence, intentionally not promoted to a safety claim:

- Actual minimum 3D separation: `1.881194 m`.
- Actual minimum horizontal separation: `1.314832 m`.
- Closest pair: aircraft `0-3` at `17.5 s`.
- DSD-violation estimate: `1.8 s` at the analyzer's 10 Hz grid.
- Therefore the V4 cutover wiring smoke passes, but the 10 m DSD performance
  criterion fails. The next step must diagnose model/control/activation
  adequacy from these recorded trajectories; increasing margins or changing
  the V4 equations is not part of this Step-6 wiring correction.

Verification:

- Full `collision_avoidance` suite: 132 tests, 0 errors, 0 failures,
  0 skipped.
- Worker regressions cover asynchronous epoch-cache rollover, inactive
  Current-Best revision retention, owner-canonical timestamp consensus,
  active latching, and infeasible zero-new-candidate behavior.
- Analyzer `py_compile`, runner/processor `bash -n`, and `git diff --check`
  pass.

### Step 6 adversarial audit

| Attack | Result | Evidence |
|---|---|---|
| A selected legacy intent permanently occupies the previous-epoch V4 cache | PASS | Selected retention is one independent intent; the previous full-set cache always rolls with candidate epochs, and the peer-first boundary regression passes. |
| A selected but not-yet-active V4 input disappears at the next 20 Hz refresh | PASS | Current Best is retained by role/input/revision before activation; the regression disables activation and observes the same revision on a later refresh. |
| V4 infeasibility revives a new legacy or best-unsafe candidate | PASS | The existing three-aircraft regression still emits zero packets when no Current Best exists; only an already-coordinated Current Best may be repredicted. |
| Same role but different executable input reaches consensus | PASS | Command consensus includes per-aircraft input revisions; a mismatch remains rejected. |
| Same command is rejected only because observers received adjacent 20 Hz snapshots | PASS | Consensus excludes observer-side source time and owner-canonicalizes the committed timestamp; 59/64 pre-fix command agreements motivated the correction. |
| An active role or input silently changes | PASS | Final bag reports zero active candidate switches and zero active revision switches for every aircraft. |
| Selected intent and Formation override differ | PASS | All 71 throttled override records match the owner-selected packet's candidate, ground-speed command, and lateral acceleration within the log's 0.01 precision. |
| A successful smoke is called a 10 m safety proof | PASS | The 1.881 m final minimum and 10 m DSD failure are explicit; the accepted claim is limited to wiring and observed behavior. |

### Step 6 five-axis source audit

Audit scope:

- Sources: the reviewed Auto ACAS timing/current-best evidence in
  `reference/paper/auto_acas_maneuver_selection_study_reviewed_v26.html`
  (Current Best + alternates, 20 Hz trajectory refresh, approximately 0.25 s
  delayed selection), and the project decisions in
  `reference/paper/MD_FILES/codex_v4_implementation_decisions_v2.md`.
- Derived requirements: Sections 6.4 through 6.7 and Step 6 of this plan.
- Implementation/evidence: worker cutover/cache/consensus/latching code,
  runner, bag processor/analyzer, unit/integration tests, and final SILS bag.
- Excluded from a PASS claim: formal invariance, robust uncertainty safety,
  10 m DSD assurance, 95% containment, and PX4 closed-loop tracking accuracy.

| Axis | Status | Source evidence | Implementation evidence | Reason / impact | Confidence |
|---:|---|---|---|---|---|
| 1. Source accuracy and traceability | PASS | The reviewed source separates held maneuver commands from 20 Hz trajectory refresh and identifies delayed selection as synchronization, while the project decision source defines dynamic `PredictInput` transport and existing downstream AD/activation reuse. | The implementation keeps command revision distinct from trajectory source time, retains Current Best, updates compact trajectories, and reuses the existing downstream path. | No project-specific packet field or V4 formula is claimed to be a disclosed Auto ACAS implementation detail. | High |
| 2. Interpretation fidelity | PASS | The sources require nearly identical coordinated information, not bit-identical observer receive timestamps or guaranteed DSD performance from one smoke. | The earlier exact-source-time gate was removed after bag evidence; the final report explicitly records the 10 m failure and limits success to wiring. | Neither timing prose nor a SILS run is expanded into a formal or empirical safety guarantee. | High |
| 3. Complexity proportionality | PASS | The named constraints are asynchronous epoch rollover, one retained Current Best, variable one-to-three candidates, and offline traceability. | One single-intent retention slot is used instead of another full trajectory-set cache; the existing worker, messages, topics, runner, logs, and analyzer are extended without a new node/thread/evaluator. | Every added field/check maps to an observed failure or Step-6 acceptance item; the first full-cache attempt was removed after a stack-overflow regression. | High |
| 4. Correct implementation under aligned direction | PASS | Command identity must remain stable while its trajectory is refreshed and selected peer data must be time-aligned before downstream evaluation. | 132 tests pass; final bag has exact committed identity on 138/138 epochs, zero selected-intent misses, zero active role/revision switches, and 71/71 Formation override matches. | Cache timing, command identity, latching, packet metadata, and execution handoff have static and runtime evidence. This status covers wiring correctness, not DSD performance. | High |
| 5. Directional alignment | PASS | V4 safe intervals remain upstream of candidate representation and the existing TPA/PMR/MASD/AD/coordination/activation hierarchy. | No alternative optimizer, uncertainty subtraction, vertical maneuver, or new command path was introduced; infeasible new-set behavior remains explicit. | The governing architecture remains the agreed source-directed hierarchy. | High |

Cross-axis dependencies: none remain for the bounded Step-6 wiring claim.
All five axes pass for that claim. The observed 10 m separation-performance
failure is a separate, explicit next-step blocker and is not relabeled as a
wiring failure or hidden by the audit scope.
