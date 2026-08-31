# Draft Plan: AD Activation, Dynamic Best Coordination, and Non-Closing Deactivation

Status: **IMPLEMENTED AND HILS-OBSERVED — SAFETY CALIBRATION PENDING**

Date: 2026-08-31

This document is intentionally editable. It replaces Draft 0 through Draft 3.
The immediate implementation scope is deliberately narrow:

- restore the Lockheed AD activation boundary;
- continue periodic best-maneuver evaluation while avoidance is active;
- permit an active maneuver change only after a defined superiority test and
  peer-awareness coordination;
- use the project's non-closing range-rate condition for deactivation;
- remove elapsed time from the deactivation decision; and
- do not add a Flocking recovery prediction.

## 1. Agreed State Transition

```text
INACTIVE / FLOCKING
        |
        | selected-best AD < 0
        v
ACTIVE AVOIDANCE
        |
        +-- 4 Hz: new clearly-superior best
        |          + peer-awareness confirmed
        |              -> coordinated active maneuver switch
        |
        | all affected pairs have R_dot >= 0
        v
INACTIVE / FLOCKING
```

The complete decision rule is:

```text
while inactive:
    if selected-best AD < 0:
        activate the selected best avoidance command
    else:
        remain in Flocking

while active:
    at a 4 Hz selection epoch:
        evaluate current best and alternates
        if a new best is clearly superior and peer-awareness is confirmed:
            switch to the new coordinated best
        else:
            keep executing the previous best

    if monitor data is invalid:
        remain active
    else if every affected pair has R_dot >= 0:
        deactivate and return to Flocking
    else:
        remain active
```

Elapsed time is not part of this state transition:

\[
\boxed{t_{\mathrm{active}}\not\Rightarrow\text{deactivation}}
\]

No Flocking trajectory is predicted before deactivation:

\[
\boxed{AD_{\mathrm{Flocking}}\text{ is not computed or used}}
\]

## 2. Source Boundary and Project Modification

This plan must not be reported as completely identical to the public Lockheed
baseline.

### 2.1 Source-backed parts

- Candidate maneuver combinations are evaluated periodically.
- The selected trajectory is refreshed from the current flight state between
  selection frames.
- PMR and MASD define:

  \[
  AD=PMR-MASD.
  \]

- An inactive controller requests avoidance when the selected-best trajectory
  satisfies:

  \[
  \boxed{AD_{\mathrm{best}}<0}.
  \]

- The public baseline monitors range/closure rate after activation.
- The previous best remains the first-choice candidate until another maneuver
  is clearly identified as superior.
- Maneuver selection is periodic. On non-selection frames, maneuver commands
  remain fixed while their trajectories are refreshed from current state.
- Before a newly selected maneuver is performed, the algorithm checks whether
  cooperating aircraft are expected to be aware of the change. If not, the
  previous best is performed.

The public papers do not disclose a numerical `clearly superior` margin or an
explicit active-avoidance `M1 -> M2` state-machine example. Therefore active
switching is the source-aligned project interpretation, while its exact
superiority threshold and acknowledgment protocol must be labeled as project
integration choices.

### 2.2 Explicit project modifications

- The project uses code range rate `R_dot >= 0 m/s` as the deactivation
  boundary. The disclosed Lockheed baseline used a substantially stronger
  separating-rate condition corresponding to approximately
  `R_dot >= 30.48 m/s` in the current code convention.
- The project removes the disclosed 4.5 s active-maneuver timeout from the
  deactivation decision.
- The project returns directly to the current Flocking controller when the
  non-closing condition is met; it does not first evaluate a predicted
  Flocking trajectory.
- DSD remains 10 m inside MASD.
- The current project aircraft model, covariance model, roll candidates,
  distributed coordination, PX4 adapter, and five-aircraft HILS topology are
  retained.
- The project uses an explicit proposal/awareness handshake instead of the
  paper's undisclosed transmission-delay estimator.

Therefore the accurate name for this policy is:

```text
Lockheed-derived AD activation
+ periodic dynamic-best evaluation
+ explicit peer-awareness coordination
+ project non-closing deactivation
+ no-timeout continuation
```

It must not be named `exact Lockheed Auto ACAS`.

## 3. AD Boundary

With DSD = 10 m already included in MASD:

\[
AD < 0
\iff
PMR < MASD.
\]

The activation parameter must be:

```text
amac_activation_threshold_m = 0.0
```

The boundary behavior is strict:

| Selected-best AD | Inactive-state action |
|---:|---|
| `AD < 0` | Activate selected avoidance maneuver |
| `AD == 0` | Remain in Flocking |
| `AD > 0` | Remain in Flocking |

No additional positive AD margin is permitted in this policy. In particular,
DSD = 10 m does not imply `AD < 10 m` activation because DSD has already been
subtracted through MASD.

## 4. Range-Rate Meaning

The current worker computes:

\[
\dot R =
\frac{(\mathbf p_B-\mathbf p_A)^T(\mathbf v_B-\mathbf v_A)}
     {\|\mathbf p_B-\mathbf p_A\|}.
\]

Its sign is:

| Code range rate | Meaning |
|---:|---|
| `R_dot < 0` | Range decreasing; pair still closing |
| `R_dot == 0` | Instantaneously non-closing |
| `R_dot > 0` | Range increasing; pair separating |

For every peer included in the activation's affected-threat set:

\[
\boxed{
\text{deactivate}
\iff
\forall i\in\mathcal A:\dot R_i\ge0
}
\]

If even one affected peer still has `R_dot < 0`, avoidance remains active.
Invalid or unavailable range-rate data cannot satisfy the deactivation gate.

## 5. Timing Contract

The following time quantities are independent:

| Quantity | Meaning | Required behavior |
|---|---|---|
| 4.5 s | Future prediction horizon | Keep predicting every candidate 4.5 s into the future. |
| 0.1 s | Prediction sample spacing | Keep the existing 46 samples from 0.0 through 4.5 s inclusive. |
| 20 Hz / 0.05 s | Selected/executed trajectory and activation monitoring | Recompute current AD and range rate. Do not run a hidden full combination search. |
| 4 Hz / 0.25 s | Candidate generation and full combination selection | Keep the best planning maneuver updated periodically. |
| active elapsed time | Diagnostic only | It may be logged, but it cannot deactivate avoidance. |

The numerical equality between the 4.5 s prediction horizon and the former
4.5 s active timeout must not be used to recreate a timed command hold or a
timed deactivation.

## 6. Planning and Execution Boundary

The latest 4 Hz selected combination and the command currently applied to PX4
must remain distinguishable:

```text
4 Hz full combination evaluation
        |
        v
latest coordinated planning best
        |
        | activation commit when AD < 0
        v
executed ownship avoidance command
        |
        +--> 20 Hz AD and R_dot monitoring
```

For this minimal step:

- the planning best continues to update at 4 Hz;
- the active ownship command remains unchanged while a new proposal is being
  evaluated and coordinated;
- an active ownship command may change before deactivation only when a new
  best passes the project superiority gate and peer-awareness barrier;
- a 20 Hz monitor update cannot replace the executed candidate;
- each peer's published actual command remains the source of truth when
  evaluating that peer.

The active command is therefore not an unconditional activation-to-deactivation
latch. It is a `previous best` that remains executable until a coordinated
replacement is accepted.

Acceptance of that replacement must be an atomic command handoff, not an
activation-state transition:

```text
ACTIVE(M1)
    |
    | coordinated M2 commit
    v
ACTIVE(M2)
```

The handoff updates only the executed candidate ID, candidate-input revision,
and `PredictInput`. It must preserve `active == true`, the original activation
episode timestamp, and the affected-threat set. It must not emit a deactivate,
reactivate, or one-cycle Flocking command between `M1` and `M2`.

## 7. Dynamic Best and Peer-Awareness Gap

### 7.1 What already exists

The current project already contains most of the required communication path:

- candidate generation and selection epochs continue at 4 Hz;
- `HeuristicCandidateSelector` keeps the current best in the three-candidate
  set and fills the other two positions with alternates;
- the joint evaluator compares complete multi-aircraft combinations;
- every aircraft publishes
  `/common/px4_<id>/maneuver_selection_decision`;
- every other aircraft subscribes to that decision topic;
- the decision message already carries the selected tuple, proposed tuple,
  candidate input revisions, proposal epoch, ownship candidate, activation
  state, `proposal_consensus_confirmed`, `new_best_accepted`, and
  `previous_best_retained`; and
- a missing or mismatched same-epoch proposal currently retains the previous
  best.

No new ROS topic or node is required.

### 7.2 What currently prevents active switching

The 4 Hz worker loop still runs while avoidance is active, but two code paths
make a new active ownship best impossible:

1. `constrainActiveAircraftCandidates()` replaces every candidate slot of an
   active aircraft with its already-executed candidate before joint
   evaluation. The evaluator therefore cannot compare that aircraft's
   alternates.
2. `finalizePendingCoordination()` rejects a proposal when the active
   ownship candidate ID or input revision differs from the activation latch.

The unit test
`ActiveAircraftLatchesOnlyItsOwnCandidateAcrossAConfirmedNewEpoch` explicitly
asserts this fixed-candidate behavior. Consequently, the current implementation
does not perform:

\[
M_1\text{ active}
\rightarrow M_2\text{ becomes better}
\rightarrow\text{coordination}
\rightarrow M_2\text{ execution}.
\]

### 7.3 Current communication is partial, not absent

The current proposal exchange does transmit the proposed new tuple and wait
until every peer publishes the same proposal epoch, candidate IDs, and input
revisions. This is a real peer communication/consensus path.

However, the current code commits as soon as matching proposals have been
observed locally. It does not separately wait until every peer publishes that
it has also observed the full matching proposal set. Although
`proposal_consensus_confirmed` is already present in the message and received
from peers, it is not used as a second pre-execution readiness barrier.

Therefore the accurate current assessment is:

| Capability | Current state |
|---|---|
| Publish and subscribe peer decisions | Implemented |
| Share proposal epoch, tuple, and input revisions | Implemented |
| Retain previous best on missing/mismatched proposal | Implemented |
| Re-evaluate active ownship alternatives | Not implemented; candidates are collapsed |
| Accept a different active ownship best | Not implemented; commit is rejected |
| Confirm every peer is ready before executing changed best | Partial; matching proposals are checked, but no second readiness barrier is enforced |
| Numerical `clearly superior` rule | Not implemented; current `1e-12` evaluator tolerance is only a floating-point comparison tolerance |

### 7.4 Target active-switch coordination

The target sequence reuses the existing decision message:

```text
M1 remains executed
    |
    | 4 Hz evaluation finds candidate M2
    v
M2 passes clearly-superior project rule
    |
    v
publish M2 proposal(epoch, tuple, input revisions)
    |
    v
receive exact matching proposals from every participant
    |
    v
publish local proposal_consensus_confirmed=true
while still executing M1
    |
    v
receive proposal_consensus_confirmed=true from every participant
for the same proposal epoch and exact tuple
    |
    v
commit M2 and switch the ownship executed command
```

If any proposal is missing, stale, mismatched, or not confirmed, `M1` remains
executed. A 20 Hz trajectory refresh can update trajectories and safety
diagnostics but cannot advance this state machine or switch the command.

This two-stage readiness barrier is a project implementation of the paper's
peer-awareness principle. It must not be described as the undisclosed exact
Lockheed delay-estimation algorithm.

### 7.5 Explicit design gate: “clearly superior”

The public source does not disclose the numerical switching margin. The
current evaluator selects any improvement larger than a numerical tolerance of
approximately `1e-12`; this is not a meaningful superiority/hysteresis rule.

Before active switching is enabled, the project must approve an explicit rule
for both evaluator cases:

```text
both combinations feasible:
    Cost_new < Cost_current - Delta_cost_switch

both combinations unsafe:
    minimum_AD_new > minimum_AD_current + Delta_AD_switch

new combination feasible and current combination unsafe:
    new combination is superior
```

The initial HILS calibration records the project values in
`testing_module/maneuver_selection_hils/config/amac_dynamic_best.yaml`:

```text
Delta_cost_switch = 0.01
Delta_AD_switch = 1.0 m
```

These values are provisional engineering choices, not claimed Lockheed
values. HILS results must be used to retain or recalibrate them.

## 8. Five-Axis Audit Baseline

| Axis | Baseline | Finding addressed by this draft |
|---|---|---|
| 1. Source accuracy | PASS | The papers support periodic best selection, retention until a clearly superior alternative, trajectory refresh between selection frames, and checking peer awareness before a new selection is performed. The exact active-switch state machine and numerical margin are explicitly marked undisclosed. |
| 2. Interpretation fidelity | FAIL | The current absolute active latch turns an undisclosed detail into a prohibition on active best changes. Conversely, an immediate switch on any tiny cost improvement would over-interpret `clearly superior`. |
| 3. Complexity proportionality | PASS | The existing proposal message, subscriptions, peer cache, and consensus fields are reusable. No new node, topic, or worker is required. |
| 4. Implementation correctness | NOT_APPLICABLE | Active-switch direction and the unresolved superiority contract must be corrected before local implementation details can receive an Axis 4 verdict. |
| 5. Directional alignment | FAIL | Active candidates are collapsed and changed active ownship proposals are rejected, so periodic evaluation cannot produce a coordinated active best change. |

## 9. Remediation Sequence

### Phase 1 — Remove unintended policy expansion

**Audit axes:** 1, 2, 3, and 5

**Root cause:** `AD_Flocking` was introduced as an additional safety check even
though it was not requested and is not required for the agreed experiment.

**Affected artifacts:**

- this plan;
- comments, tests, or code added later from this plan;
- runtime configuration for the Lockheed-derived AMAC policy.

**Required change:**

1. Do not create `AD_Flocking`, `recovery_minimum_ad_m`, a recovery predictor,
   or a recovery state.
2. Do not add a positive activation threshold.
3. Keep DSD = 10 m only inside MASD.
4. Describe `R_dot >= 0` and timeout removal as project choices.

**Pass criterion:** Static search and review find no Flocking recovery AD or
time-based release dependency in this policy.

### Phase 2 — Restore the AD activation boundary

**Audit axes:** 1, 2, and 4

**Affected artifacts:**

- `include/collision_avoidance/selection/ManeuverActivationController.hpp`
- `src/selection/ManeuverActivationController.cpp`
- `test/test_maneuver_activation_controller.cpp`
- `src/testing_module/maneuver_selection_hils/scripts/run_execution_policy_comparison.sh`
- other HILS launch paths that override `amac_activation_threshold_m`

**Required change:**

1. Set the policy's effective activation threshold to `0.0 m`.
2. Activate only for finite selected-best `AD < 0` with at least one unsafe
   affected peer.
3. Keep `AD == 0` and `AD > 0` inactive.
4. Ensure result metadata records the effective zero threshold.

**Pass criterion:** Unit and runtime-parameter tests prove the strict boundary
and show no `AD < 10 m` early trigger.

### Phase 3 — Replace timeout/separation termination with non-closing release

**Audit axes:** 2, 4, and 5

**Affected artifacts:**

- `include/collision_avoidance/selection/ManeuverActivationController.hpp`
- `src/selection/ManeuverActivationController.cpp`
- `test/test_maneuver_activation_controller.cpp`
- worker integration and distributed runtime tests

**Required change:**

1. Set the project deactivation threshold to `0.0 m/s` in the current
   range-rate convention.
2. Require every affected peer to have finite `R_dot >= 0`.
3. Remove `maximum_active_duration_us` from the deactivation decision.
4. Remove `Timeout` as a maneuver-deactivation reason if it has no remaining
   semantic use. Active elapsed time may remain only as a diagnostic.
5. Preserve active state on invalid monitor data.
6. Keep the active ownship command unchanged unless either the non-closing
   deactivation condition is satisfied or Phase 4 commits a coordinated,
   clearly-superior replacement.

**Content to remove or defer:** `rearm_required`, clear-before-rearm,
Flocking-recovery AD, timeout recovery, and coordinated timeout recovery.

**Pass criterion:** An active maneuver remains uninterrupted at 4.5 s, 10 s,
or any later time while at least one affected pair has `R_dot < 0`, and it
deactivates on the first valid monitor sample where every affected pair has
`R_dot >= 0`.

### Phase 4 — Enable coordinated dynamic-best replacement

**Audit axes:** 1, 2, 4, and 5

**Dependencies:**

- Phases 1 through 3 pass their unit checks.
- `Delta_cost_switch = 0.01` and `Delta_AD_switch = 1.0 m` are recorded as
  provisional project parameters in the HILS policy YAML.

**Root cause:** Active candidate sets are currently collapsed to the executed
candidate, and a changed active ownship proposal is rejected even when every
peer has published the same proposal.

**Affected artifacts:**

- `include/collision_avoidance/selection/ManeuverActivationController.hpp`
- `src/selection/ManeuverActivationController.cpp`
- `include/collision_avoidance/selection/ManeuverSelectionWorker.hpp`
- `src/selection/ManeuverSelectionWorker.cpp`
- `include/collision_avoidance/selection/HeuristicCandidateSelector.hpp`
- `src/selection/HeuristicCandidateSelector.cpp`, only if the approved
  superiority rule belongs at candidate pre-selection rather than tuple commit
- `src/selection/ManeuverCombinationEvaluator.cpp`, only for exposing the
  current-tuple and proposed-tuple comparison needed by the switch gate
- `msg/ManeuverSelectionDecision.msg`, only if an existing field proves
  insufficient after static review
- `src/communication/DistributedManeuverSelectionRuntime.cpp`
- `test/test_maneuver_activation_controller.cpp`
- `test/test_maneuver_selection_worker.cpp`
- `test/test_distributed_maneuver_selection_runtime.cpp`

**Required change:**

1. Keep all eligible `Current Best + Alternate 2` candidates available in the
   4 Hz planning evaluator even when the aircraft is active.
2. Do not use the unrestricted planning set as the 20 Hz executed-trajectory
   monitor input; that path must continue to use the actual published command.
3. Evaluate the current coordinated tuple and proposed tuple under the same
   timestamp, candidate revisions, and cost/AD ordering.
4. Retain the current executed best unless the proposal passes the approved
   `clearly superior` project rule.
5. Remove the unconditional active-ownship change rejection. Replace it with
   the two-stage same-epoch proposal/readiness barrier in Section 7.4.
6. Reuse `proposal_consensus_confirmed` as the local readiness advertisement
   if its current semantics can be separated from commit. Add no new topic.
7. Commit a changed active ownship candidate only after every participant has
   advertised readiness for the same proposal epoch, exact tuple, and exact
   input revisions.
8. Route that approved commit into the activation controller as an explicit
   active-command replacement. Atomically replace
   `latched_candidate_id`, `latched_candidate_input_revision`, and
   `latched_input` while preserving `active`, `activation_timestamp_us`, and
   `affected_threat_mask`. The replacement must not set `just_deactivated` or
   `just_activated` and must not pass through the Flocking command path.
9. Until commit, keep sending the previous best to PX4. On missing, stale, or
   mismatched readiness, keep the previous best.
10. After commit, publish the new selected tuple and actual ownship candidate so
   all peers' 20 Hz monitors use the new executed command.

**Content to remove:**

- Candidate-slot replication that prevents active planning comparison.
- The test contract that an active ownship candidate can never change.
- Any use of the floating-point comparison tolerance as a claimed
  `clearly-superior` margin.

**Pass criterion:** During an active episode, a clearly-superior `M2` remains
only a proposal while any peer is unaware, the executed command remains `M1`,
and `M2` is applied only after the same-epoch all-peer readiness barrier. The
handoff is observed as `ACTIVE(M1) -> ACTIVE(M2)`, with no inactive/Flocking
sample and no reset of the activation episode or affected-threat set. A
non-superior, missing, stale, or mismatched proposal never changes execution.

### Phase 5 — Verify cadence and five-aircraft HILS behavior

**Audit axes:** all

**Dependencies:** Phases 1 through 4 pass static and unit checks.

**Required verification:**

1. Build and run all collision-avoidance package tests, including the sourced
   distributed runtime test.
2. Run the existing five-aircraft headless pentagon/Flocking HILS using this
   policy separately from Horizon-Gated V4.
3. Record in rosbag and analysis output:
   - 4 Hz full combination-selection cadence;
   - 20 Hz selected/executed trajectory monitoring cadence;
   - selected-best AD at every activation;
   - proof that every activation has `AD < 0`;
   - every affected pair's `R_dot` at deactivation;
   - active duration, including intervals longer than 4.5 s;
   - active candidate switches;
   - current and proposed cost/minimum-AD difference at each switch decision;
   - proposal publication, local readiness, all-peer readiness, and commit
     timestamps;
   - previous-best retention reasons for missing, stale, mismatched, or
     non-superior proposals;
   - Flocking/avoidance command-mode transitions;
   - minimum 3-D separation and DSD = 10 m violations.
4. Re-run an independent five-axis audit after implementation and HILS.

**Pass criterion:** Code, tests, and bag evidence all show zero-threshold AD
activation, all-peer non-closing deactivation, no time-based deactivation, and
the intended 4 Hz/20 Hz cadence separation. Every active candidate switch must
have a preceding superiority decision and all-peer same-epoch readiness
evidence.

## 10. Required Tests

### 10.1 Activation boundary

- `AD = -epsilon`: activates.
- `AD = 0`: remains inactive.
- `AD = +epsilon`: remains inactive.
- invalid AD or empty unsafe mask: remains inactive.

### 10.2 Active deactivation

- one affected peer with `R_dot < 0`: remains active.
- all affected peers with `R_dot == 0`: deactivates.
- all affected peers with `R_dot > 0`: deactivates.
- one non-finite affected-peer rate: remains active.
- invalid monitor sample: remains active.
- elapsed time greater than 4.5 s with `R_dot < 0`: remains active.
- elapsed time greater than 10 s with `R_dot < 0`: remains active.
- no test references `AD_Flocking` or timeout deactivation.

### 10.3 Cadence and execution

- full candidate-combination evaluation occurs only at 4 Hz.
- current executed-trajectory monitoring occurs at 20 Hz.
- the 20 Hz path does not perform a hidden full combination search.
- the active candidate cannot switch because only time elapsed.
- a peer is evaluated using its published actual command when available.

### 10.4 Dynamic best and awareness

- Active `M1` does not collapse the 4 Hz planning candidates to `M1` only.
- A better but not clearly-superior `M2` retains `M1`.
- A clearly-superior `M2` is published as a proposal while `M1` remains
  executed.
- Matching proposal contents without all-peer readiness retain `M1`.
- Missing, stale, or mismatched peer readiness retains `M1`.
- All peers ready for the same epoch, candidate IDs, and revisions commits
  `M2`.
- The commit atomically changes the activation controller's latched ownship
  candidate ID, input revision, and input.
- The active flag remains true, the original activation timestamp and affected
  threat mask remain unchanged, and neither `just_deactivated` nor
  `just_activated` is asserted during the `M1 -> M2` handoff.
- No Flocking command sample appears between the last `M1` command and the
  first `M2` command.
- The subsequent decision publication reports `M2` as the actual ownship
  command and peer 20 Hz monitors consume that command.
- No command switch occurs on a 20 Hz refresh.
- The former
  `ActiveAircraftLatchesOnlyItsOwnCandidateAcrossAConfirmedNewEpoch` test is
  replaced by active coordinated-switch and previous-best-retention tests.

## 11. Verification Matrix

| Axis | Observable PASS evidence | Regression guard |
|---|---|---|
| 1. Source accuracy | `AD < 0` activation, periodic best evaluation, previous-best retention, and peer awareness are tied to the public AMAC sources; `R_dot >= 0`, timeout removal, explicit readiness exchange, and switching margins are labeled as project modifications. | No numerical superiority or exact active-switch state machine is attributed to Lockheed. |
| 2. Interpretation fidelity | `AD >= 0` means no activation while inactive. `Clearly superior` is not reduced to floating-point tolerance, and peer awareness is not claimed from proposal publication alone. | DSD = 10 m is not converted into an extra AD threshold; active switching cannot bypass the approved superiority/readiness gates. |
| 3. Complexity proportionality | The existing controller, worker, evaluator, decision message, publisher, subscriber, and peer cache implement the rule without a new predictor, recovery state, node, worker, or topic. | Static search shows no `AD_Flocking` data path or duplicate coordination protocol. |
| 4. Implementation correctness | Boundary, invalid-data, all-peer range-rate, long-duration, cadence, active-switch, atomic latch-replacement, command-continuity, previous-best-retention, readiness, distributed-runtime, and HILS checks pass. | Existing prediction, reconstruction, covariance, time alignment, and peer actual-command tests remain passing; an active replacement cannot reset the activation episode or emit a Flocking pulse. |
| 5. Directional alignment | Runtime follows the agreed state transition: `AD < 0` activates; periodic evaluation may replace an active best only after superiority and awareness; all-peer `R_dot >= 0` deactivates; time does not deactivate. | No unconditional active latch, immediate uncoordinated switch, timeout, Flocking prediction, positive AD trigger, or recovery gate controls execution. |

## 12. Risks and Explicit Non-Goals

### Accepted risks to measure

- `R_dot >= 0` is instantaneous and materially less restrictive than the
  disclosed Lockheed separating-rate threshold.
- A pair can satisfy `R_dot >= 0` while still closer than DSD or while AD is
  negative. This policy intentionally does not add another deactivation gate.
- Returning directly to Flocking can make the pair approach again because the
  Flocking path is not predicted before release.
- If `R_dot < 0` persists because the selected command is ineffective, the
  same avoidance command can remain active for an unbounded time.
- The source does not disclose `Delta_cost_switch` or `Delta_AD_switch`.
  The initial project values can suppress necessary switches or create
  maneuver chatter, so their adequacy remains a HILS calibration question.
- A two-stage explicit readiness barrier is more concrete than the public
  delay-estimation description and may add selection latency. That latency
  must be measured rather than attributed to Lockheed.
- Best-effort decision-message loss must retain the previous best; it may delay
  a beneficial switch but must not cause an uncoordinated switch.
- These behaviors are to be measured, not silently repaired during this
  experiment.

### Non-goals

- No `AD_Flocking` calculation or recovery-trajectory prediction.
- No time-based avoidance termination.
- No timeout recovery/reselection state.
- No positive activation margin such as `AD < 10 m`.
- No uncoordinated active-command switch.
- No claimed Lockheed numerical superiority margin.
- No new ROS topic, node, worker thread, or parallel coordination protocol.
- No change to the trajectory model, covariance model, roll candidates,
  ground-to-EAS adapter, or cubic reconstruction.
- No modification to Horizon-Gated V4 in this step.
- No claim of production-equivalent Lockheed logic or flight-safety
  certification.

## 13. Completion Gate

- [x] User approved Draft 4 implementation.
- [x] Effective AMAC activation threshold is `0.0 m`.
- [x] `AD < 0` activates and `AD >= 0` remains inactive.
- [x] Effective deactivation range-rate threshold is `0.0 m/s`.
- [x] Every affected pair must satisfy `R_dot >= 0` before deactivation.
- [x] No elapsed-time value can deactivate an active maneuver.
- [x] No `AD_Flocking` or Flocking recovery prediction exists.
- [x] Provisional `Delta_cost_switch = 0.01` and
      `Delta_AD_switch = 1.0 m` are recorded as project HILS parameters.
- [x] Active aircraft retain their alternates in the 4 Hz planning evaluator.
- [x] A new active best cannot execute before the all-peer same-epoch
      readiness barrier.
- [x] Missing, stale, mismatched, or non-superior proposals retain the previous
      executed best.
- [x] An approved active replacement atomically updates only the executed
      candidate latch and preserves the active episode, activation timestamp,
      and affected-threat set without a Flocking command gap.
- [x] Every committed active switch is published and becomes the peer actual
      command used by subsequent safety monitoring.
- [x] 4 Hz selection and 20 Hz monitoring have separate code and unit-test
      paths; bag cadence evidence remains pending.
- [x] Package and distributed runtime tests pass.
- [x] Separate five-aircraft HILS and rosbag analysis complete; the run records
      a 10 m DSD violation and is not a safety-performance pass.
- [x] Independent five-axis re-audit returns `PASS` for conformity to the
      agreed policy, with the safety-performance failure reported separately.

## 14. Revision Log

| Revision | Date | State | Change |
|---|---|---|---|
| Draft 0 | 2026-08-31 | Superseded | Initial cadence and timeout-remediation plan. |
| Draft 1 | 2026-08-31 | Superseded | Added an unrequested Flocking recovery AD and timeout recovery policy. |
| Draft 2 | 2026-08-31 | Superseded | Restored strict `AD < 0` activation, retained project `R_dot >= 0` deactivation, removed elapsed-time termination, and removed all `AD_Flocking` behavior. |
| Draft 3 | 2026-08-31 | Superseded | Added periodic active best reevaluation, documented the current hard-latch defect and existing decision transport, and required superiority plus an all-peer readiness barrier before an active maneuver switch. |
| Draft 4 | 2026-08-31 | Proposed | Made active dynamic-best replacement mandatory and defined the coordinated commit as an atomic `ACTIVE(M1) -> ACTIVE(M2)` latch handoff with no deactivation, reactivation, or Flocking command gap. |
| Implementation 1 | 2026-08-31 | Static/unit verified | Implemented Draft 4 and recorded provisional HILS superiority margins (`0.01`, `1.0 m`); five-aircraft bag verification remains pending. |
| HILS 1 | 2026-08-31 | Conformity pass, performance fail | Verified 4 Hz/20 Hz cadence and distributed invariants in a valid five-aircraft bag; minimum separation was 8.279 m against DSD = 10 m, so safety calibration remains open. |
