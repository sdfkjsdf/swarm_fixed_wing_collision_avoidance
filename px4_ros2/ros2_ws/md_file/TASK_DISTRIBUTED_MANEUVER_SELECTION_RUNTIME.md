# Distributed Maneuver-Selection Runtime — Next Implementation Scope

Status: **Phases A-C implemented; ROS shadow transport integration verified;
two-PX4 non-GUI SILS acceptance pending**
Baseline commit: **`c14268e`**
Runtime ownership: **one aircraft = one onboard ROS 2 node instance**

## 1. Target Pass State

The next implementation shall connect the existing static trajectory-intent
and 3 x 3 evaluator into each aircraft's onboard node without introducing a
central swarm maneuver selector.

```text
Aircraft A / onboard node A                    Aircraft B / onboard node B
----------------------------------             ----------------------------------
own state + received B intents                 own state + received A intents
              |                                             |
local candidate prediction                                  local candidate prediction
local combination scoring                                   local combination scoring
local coordination check                                    local coordination check
              |                                             |
command only PX4 A                                          command only PX4 B
```

Each aircraft independently reaches its own conclusion from nearly identical
shared information. A node shall never issue another aircraft's PX4 command.

### 1.1 Source baseline and interpretation boundary

Authoritative source baseline:

- Turner et al. (2012), *Automatic Aircraft Collision Avoidance Algorithm
  Design for Fighter Aircraft*, Section 4.5;
- Wadley et al. (2013), *Development of an Automatic Aircraft Collision
  Avoidance System for Fighter Aircraft*, Section III-D.

The sources directly support independent aircraft scoring, new maneuver
generation at 4 Hz, trajectory refresh at 20 Hz, and combination selection
approximately 0.25 s after alternate generation so participating aircraft have
time to receive updated trajectories.

The five stages below, the C++ worker boundary, SPSC channels, exact timeout
states, and thread ownership are project implementation decisions. The reviewed
HTML presents the five-stage parallel diagram as a learning-oriented
reconstruction; it is not treated as disclosed Auto ACAS software architecture.

The source use of "independent" or "parallel" aircraft lanes does **not** imply
one CPU thread per aircraft candidate, trajectory pair, or combination inside
one onboard node.

## 2. Passing Invariants to Preserve

The next step must preserve all behavior already verified at `c14268e`:

- the predictor remains a ground-kinematic model;
- PX4 EAS conversion remains at the PX4 command boundary;
- lateral candidate IDs remain the shared preflight lookup contract;
- no vertical avoidance candidate is introduced;
- A transmits the compact intent, not all 46 points;
- B reuses `TrajectoryIntentReceiver` for cubic mean reconstruction and
  covariance/cone propagation;
- evaluation reuses the existing 46 synchronous points;
- DSD remains exactly 10.0 m while aircraft size and uncertainty remain
  separate MASD terms;
- valid delayed trajectories are aligned to a common time;
- when all valid AD values are nonpositive, the largest AD remains the best
  available combination;
- the existing `SpscQueue` implementation and thread-ownership naming
  convention are reused rather than replaced.

## 3. Current Baseline: Implemented and Missing

### 3.1 Already implemented

- seven lateral lookup candidates: roll `0`, `+/-15`, `+/-30`, `+/-45` degrees;
- compact `TrajectoryIntentPacket` and ROS message conversion;
- A-side prediction and packet construction;
- B-side 46-point mean reconstruction and covariance/cone propagation;
- ROS publisher/subscription wrappers;
- static two-aircraft 3 x 3 PMR/MASD/AD evaluator;
- deterministic result table and focused unit tests;
- reusable fixed-size `SpscQueue`;
- proven main-thread/worker-thread lifecycle pattern in `FormationMode` and
  `TrajectoryReplayMode`.

### 3.2 Implemented in this change

- runtime `Current Best + two Alternates` state management;
- `selection_epoch` in the compact packet and ROS message;
- deterministic 4 Hz candidate refresh, 20 Hz trajectory refresh, and 0.25 s
  qualification boundary;
- per-sender staging cache that atomically promotes only a complete
  `(selection_epoch, source_timestamp)` three-candidate set and retains the
  previous complete set while a newer refresh is incomplete;
- compact packet reconstruction and existing 3 x 3 evaluator invocation in a
  dedicated computation worker;
- one worker and one ROS adapter owned by each existing `vtol_guidance_node`;
- independent local selection, previous-best fallback, and separate activation
  decision;
- local-only Formation command arbitration behind a default-on shadow switch;
- ROS topic integration test with two independent runtime instances.

### 3.3 Still pending

- two real `vtol_guidance_node` instances connected to two PX4 SITL aircraft in
  non-GUI Gazebo;
- rosbag evidence for measured 20 Hz/4 Hz cadence and epoch grouping in that
  two-PX4 run;
- closed-loop acceptance of the local avoidance override; shadow mode remains
  the default;
- generalized three- or four-aircraft combination scoring.

## 4. Required Five-Stage Runtime Behavior

The five stages are implemented inside every onboard node. They are not five
central services and do not require five worker threads.

### Stage 1 — Candidate-set refresh

- Run the alternate-refresh event at 4 Hz.
- Retain one `Current Best` and choose two different alternates from the
  configured eligible lateral candidate IDs.
- Increment a local `selection_epoch` whenever the held three-candidate set
  changes.
- Keep the three candidate IDs fixed until the next 4 Hz refresh event.
- The first runtime test uses a configured eligible list. Threat isolation and
  the paper's undisclosed detailed pre-selection rules are not invented here.

### Stage 2 — Coordination waiting interval

- Record the generation time of each `selection_epoch`.
- Do not qualify a new best from that epoch until approximately 0.25 s has
  elapsed.
- Continue using the previous best during the waiting interval.
- Do not block or sleep the ROS executor for 0.25 s; represent the wait as a
  timestamped worker state.

### Stage 3 — Trajectory refresh and sharing

- Run at 20 Hz while keeping the same three maneuver IDs.
- Use the latest complete ownship state as the new predictor initial state.
- Build and publish one compact intent for each of the three held candidates.
- Include `selection_epoch` in the packet/message so three packets can be
  grouped without confusing adjacent candidate generations.
- Identify the sending aircraft through the node/topic namespace already used
  by the multi-agent deployment; do not duplicate a vehicle ID in every packet
  unless transport tests show that namespacing is insufficient.
- On receipt, retain only the latest complete candidate set for each sender and
  reconstruct it with the existing `TrajectoryIntentReceiver`.

### Stage 4 — Combination evaluation and best selection

- At the selection event, read immutable snapshots of the latest complete
  ownship and threat candidate sets.
- For the first runtime milestone, call the existing two-aircraft 3 x 3
  evaluator without changing its PMR/MASD/AD equations.
- Keep evaluation as deterministic fixed-size loops in one worker thread.
- Do not create one thread per candidate, PMR window, pair, or combination.
- Record the selected combination, local selected candidate ID, AD, validity,
  evaluated timestamps, and epoch IDs for diagnosis.

The phrase "approximately 4 Hz selection cadence" remains an interpretation of
the disclosed 4 Hz generation plus 0.25 s delayed-selection timing. It shall not
be documented as an independently disclosed source frequency.

### Stage 5 — Coordination check and activation output

- A newly evaluated best becomes `qualified_best` only after the coordination
  waiting condition is satisfied and the required remote candidate set is
  complete and valid.
- Otherwise retain `previous_best`.
- Emit an activation decision separately from the best-candidate decision.
- For the first HILS integration, collision avoidance is active when the
  selected combination's AD is negative and all required inputs are valid.
- When active, the local selected lateral candidate may override the local
  formation lateral command; it shall never command another aircraft.
- Maintain the existing level-flight restriction and ground-speed command
  semantics.
- Before closed-loop HILS acceptance, expose the full decision in shadow mode
  so timing, coordination, and chosen candidate can be checked without applying
  the avoidance override to PX4.

## 5. Thread and Data-Ownership Design

### 5.1 One worker per onboard node

```text
ROS/main thread
  - ownship/PX4 callbacks
  - remote intent callbacks
  - ROS publish
  - PX4 setpoint update
              |
              | immutable input snapshot (existing SpscQueue)
              v
ManeuverSelectionWorker thread
  - five-stage timing state
  - candidate set ownership
  - prediction/reconstruction cache ownership
  - 3 x 3 evaluator
              |
              | packets + selection result (existing SpscQueue)
              v
ROS/main thread
```

The existing `FormationMode` RT worker continues to own flocking computation.
The selection worker is a second, independent worker inside the same onboard
node; it does not take ownership of ROS entities or the PX4 mode lifecycle.

### 5.2 Reuse boundary

Reuse directly:

- `collision_avoidance/common/SpscQueue.hpp`;
- the member suffix convention `_mt`, `_worker`, `_mt2worker`, and
  `_worker2mt`, adapted from the current `_mt`/`_rt` convention;
- `TrajectoryIntentSender` and `TrajectoryIntentReceiver`;
- `TrajectoryIntentPublisher` and `TrajectoryIntentSubscription`;
- `ManeuverCombinationEvaluator`;
- the existing `FormationMode` thread start/stop/join pattern.

Do not reuse by coupling:

- do not call `FormationMode::rt_loop()` from the selection worker;
- do not share mutable `TrajectoryIntentReceiver` or spline instances across
  threads;
- do not make ROS publishers/subscriptions owned by the pure computation
  worker;
- do not introduce a generic thread pool or a new queue implementation.

## 6. Implemented Artifact Changes

Implemented artifacts:

1. Extend the existing compact intent contract:
   - `TrajectoryIntentPacket`;
   - `TrajectoryIntent.msg`;
   - `TrajectoryIntentTransport` conversion and tests;
   - add only `selection_epoch` unless a demonstrated transport requirement
     justifies another field.
2. Add a ROS-independent runtime worker:
   - `selection/ManeuverSelectionWorker.hpp`;
   - `selection/ManeuverSelectionWorker.cpp`;
   - fixed-size input/output snapshot types in the same header initially.
3. Add focused unit tests:
   - fake/explicit clock inputs rather than real unit-test sleeps;
   - 4 Hz refresh and 20 Hz prediction events;
   - 0.25 s qualification boundary;
   - complete/incomplete epoch grouping;
   - previous-best retention;
   - AD-negative activation decision;
   - independent state across two worker instances.
4. Integrate one worker instance into each `vtol_guidance_node` through
   `FormationMode` or a small node-owned adapter after the worker tests pass.
5. Added a two-runtime ROS shadow integration test. The two-node, two-PX4,
   non-GUI SILS smoke test remains the next acceptance step before enabling the
   avoidance-command override by default.

No additional ROS node executable is planned. Launching the existing
`vtol_guidance_node` once per vehicle remains the deployment model.

## 7. Dependency-Ordered Remediation Sequence

### Phase A — Contract and source boundary

`audit_axis`: 1, 2, 5
`root_cause`: the static packet has no epoch field, while runtime grouping and
coordination cannot safely infer a complete three-candidate generation from
candidate ID and source timestamp alone.
`affected_artifacts`: `TrajectoryIntentPacket`, `TrajectoryIntent.msg`,
transport conversion, this plan.
`change`: add `selection_epoch`; keep source timestamp semantically separate.
`preserve`: four-time compact mean and `P0`; no 46-point datalink payload.
`defer/remove`: acknowledgements, new network protocol, and vehicle ID field.
`verification`: packet round-trip and adjacent-epoch grouping tests.
`pass_criterion`: packets from adjacent generations cannot form a false complete
candidate set.
`regression_guard`: existing packet fields remain bit-for-bit preserved through
ROS conversion.

### Phase B — Pure runtime state machine

`audit_axis`: 3, 4, 5
`root_cause`: the five timing stages exist only in documentation; current
prediction and evaluation are static calls.
`affected_artifacts`: new `ManeuverSelectionWorker` and unit tests.
`change`: implement one deterministic state machine driven by explicit input
timestamps and immutable snapshots.
`preserve`: existing predictor, receiver, evaluator, DSD, and uncertainty
equations.
`defer/remove`: thread pool, per-combination tasks, generalized scheduler
framework, and wall-clock sleeps in tests.
`verification`: deterministic cadence, waiting, cache, selection, and activation
tests.
`pass_criterion`: the five stage transitions occur at specified boundaries and
produce repeatable outputs from identical snapshots.
`regression_guard`: existing static evaluator tests remain unchanged and pass.

### Phase C — Existing-node thread integration

`audit_axis`: 3, 4, 5
`root_cause`: no onboard runtime currently feeds received intents into the
selection calculation.
`affected_artifacts`: `vtol_guidance_main.cpp`, `FormationMode`, and CMake only
as required to own and connect one worker.
`change`: reuse the existing SPSC and lifecycle pattern for main-to-worker
snapshots and worker-to-main outputs.
`preserve`: one agent per node and existing Formation RT ownership.
`defer/remove`: central swarm selector, extra node executable, shared mutable
spline state, and ROS calls from the computation worker.
`verification`: two independent worker instances, clean start/stop/join,
ThreadSanitizer where supported, and queue-overflow diagnostics.
`pass_criterion`: two onboard node instances keep independent candidate epochs
and terminate without race, deadlock, or leaked thread.
`regression_guard`: formation guidance remains unchanged when shadow mode is
enabled.

### Phase D — Two-aircraft shadow SILS

`audit_axis`: 1 through 5
`root_cause`: unit tests cannot establish distributed timing behavior through
ROS transport.
`affected_artifacts`: testing-module launch/config/result pipeline only.
`change`: run two existing onboard nodes, exchange three intents at 20 Hz,
evaluate after the coordination delay, and log complete decisions without
overriding PX4.
`preserve`: non-GUI execution and rosbag-first offline analysis.
`defer/remove`: Monte Carlo, four-aircraft combinations, and activation command.
`verification`: bag timestamps, candidate epochs, 20 Hz/4 Hz cadence,
coordination qualification, symmetric local decisions, and evaluator values.
`pass_criterion`: both agents independently reach compatible results from the
same complete candidate epochs and no stale/incomplete set is selected.
`regression_guard`: the existing predictor/cone HILS plots and tests remain
reproducible.

### Phase E — Local activation wiring

`audit_axis`: 2, 4, 5
`root_cause`: shadow selection does not yet affect the local PX4 command.
`affected_artifacts`: the local Formation/collision-avoidance command arbiter.
`change`: after shadow acceptance, allow a valid AD-negative qualified candidate
to override only the local lateral command and preserve the existing ground to
EAS adapter boundary.
`preserve`: level-flight avoidance and local-node-only authority.
`defer/remove`: vertical maneuvers, AMAC terrain logic, and commands to remote
aircraft.
`verification`: non-GUI two-aircraft SILS activation and symmetric turn cases.
`pass_criterion`: activation occurs only for valid qualified AD-negative output,
and deactivation/fallback never emits an uninitialized setpoint.
`regression_guard`: shadow mode remains available and is the default until the
activation test passes.

## 8. Verification Matrix

| Axis | Required evidence for PASS |
|---|---|
| 1 — Source accuracy | Every timing and independent-scoring claim cites the applicable 2012/2013 section; project thread details are labeled as project policy. |
| 2 — Interpretation fidelity | 4 Hz generation, 20 Hz refresh, and approximately 0.25 s delayed selection remain distinct; no claim that the paper mandates CPU threads or a 4 Hz evaluator. |
| 3 — Complexity proportionality | One worker per onboard node, existing SPSC, fixed-size loops, and no new thread pool, queue, reconstructor, or covariance propagator. |
| 4 — Implementation correctness | Contract, fake-clock timing, epoch completeness, two-instance isolation, lifecycle, transport, static evaluator, and SILS shadow tests pass. |
| 5 — Directional alignment | Each aircraft scores independently, commands only itself, retains previous best until qualified, and separates best selection from activation. |

Deadline evidence is measured rather than assumed. The 20 Hz update path must
finish within its 50 ms period and the delayed selection event within its 250 ms
window on the target HILS machine. These are project deadline criteria, not
claims of certified real-time behavior.

## 9. Risks and Explicit Non-Goals

Risks requiring measurement or a later decision:

- clock synchronization quality between simulated/onboard nodes;
- queue overflow if the ROS/main consumer is delayed;
- incomplete candidate epochs caused by packet loss;
- command arbitration between flocking and collision avoidance;
- exact published production behavior for nonpositive-AD cost and switching
  hysteresis remains undisclosed.

Explicit non-goals for this next step:

- a central selector node;
- one thread per candidate, pair, window, or combination;
- five-aircraft or arbitrary-N combination evaluation;
- the paper's undisclosed multi-threat pre-selection rule;
- vertical avoidance maneuvers;
- terrain-avoidance AMAC;
- continuous cubic minimum-distance root solving;
- Monte Carlo coverage calibration;
- replacing the existing formation RT worker or SPSC implementation.

The papers' disclosed multi-aircraft example reaches four participating
aircraft and up to `3^4 = 81` combinations. A five-aircraft `3^5 = 243`
evaluator would be a separately approved project extension, even though the
current state container can hold more agents.

## 10. Re-Audit and Completion Gate

Implementation of this plan is complete only when:

1. Phases A through D pass with shadow mode enabled;
2. Phase E passes before avoidance override becomes enabled by default;
3. all existing `collision_avoidance` and `trajectory_prediction_hils` tests
   continue to pass;
4. the two-node SILS rosbag demonstrates the required cadence, epoch grouping,
   common-time evaluation, and independent compatible decisions;
5. an independent five-axis re-audit reports `PASS` for every material axis.

Until those gates pass, this file remains a plan and no runtime collision-
avoidance authority is claimed.

## 11. Implementation and Verification Record — 2026-08-28

Implemented runtime artifacts:

- `ManeuverSelectionWorker`: pure computation worker, explicit timestamp-driven
  state machine, SPSC input/output, compact intent creation, reconstruction,
  and 3 x 3 evaluation;
- `DistributedManeuverSelectionRuntime`: node-owned ROS adapter for common-NED
  belief input, local intent publication, remote intent subscription, and
  worker output delivery;
- `FormationMode` arbitration input: selected command and activation are
  separate, affect only the local vehicle, and are ignored while shadow mode is
  enabled;
- `TrajectoryIntent.selection_epoch`: generation grouping without transmitting
  the 46-point reconstructed trajectory.

Verification performed:

- `collision_avoidance` and `trajectory_prediction_hils` build succeeded;
- `colcon test-result` reported 54 result records (including CTest wrappers),
  with zero errors, failures, or skips;
- the two-runtime ROS integration test exchanged three compact intents in each
  direction, reconstructed/evaluated both sides independently, and produced
  compatible qualified epoch-1 decisions;
- the deterministic worker tests cover 20 Hz/4 Hz boundaries, adjacent
  incomplete epoch isolation, independent state, activation output, and clean
  worker lifecycle.

This record is **not** evidence of a completed two-PX4 Gazebo SILS flight. It
verifies the runtime computation and ROS transport in shadow mode. Phase D bag
evidence and Phase E closed-loop HILS/SILS acceptance remain open.
