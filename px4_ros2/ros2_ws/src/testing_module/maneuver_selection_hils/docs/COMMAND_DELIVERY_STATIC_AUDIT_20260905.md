# Executable command delivery: bounded static audit

## Scope and evidence baseline

Baseline: a292fc6. Scope: FormationMode command delivery and the local callback
in DistributedManeuverSelectionRuntime. Excludes candidate evaluation, peer
qualification, activation/deactivation, MASD, predictor dynamics and tuning.

Authoritative requirement: the current user instruction preserves 4 Hz selection,
peer confirmation and AD < 0 activation, while removing avoidable waiting after
execution is authorized. Immediate event delivery is a project implementation
choice, not a claimed verbatim Lockheed implementation rule.

Inspected source paths are relative to `collision_avoidance` unless stated:

- Before this patch, `include/collision_avoidance/modes/FormationMode.hpp`,
  `setManeuverSelectionDecision()` only assigned the held decision.
- `src/modes/FormationMode.cpp`, `updateSetpoint()` was the only avoidance
  setpoint writer. The linked PX4 ROS 2 library's
  `control/setpoint_types/fixedwing/lateral_longitudinal.hpp` requests 30 Hz.
  Therefore delivery between ticks could wait for the next tick (nominally
  up to about 33.3 ms, not an end-to-end latency upper bound).
- `src/selection/ManeuverSelectionWorker.cpp`, `processPending()` calls
  `finalizePendingCoordination()` then `updateActivationState()` in the same
  iteration on commit. No extra 0.25-second post-commit wait was established.
- `src/communication/DistributedManeuverSelectionRuntime.cpp` still drains
  outputs every 10 ms. This patch does not remove that polling interval.

## Implementation

The decision setter calls the shared `publishAvoidanceSetpoint()` immediately
only when ModeBase reports active, the existing shadow/qualification/execution
gates permit override, and authorization is newly present or the applied lateral
acceleration/ground-speed command changed. Unchanged heartbeats do not cause
additional event writes. The 30 Hz refresh uses the same publishing function,
EAS conversion, turn-speed floor, command-history recording and optional tracing.

The runtime continues all peer and diagnostic publications but invokes the
local control callback once with the newest decision from the drained queue.
This prevents immediate execution from replaying superseded queued commands.
Default executor serialization is retained; no worker-thread PX4 publication,
new thread, timer, queue or consensus protocol was introduced.

## Five-axis findings

| Axis | Status | Source evidence | Implementation evidence | Reason / impact | Confidence |
|---|---|---|---|---|---|
| 1. Source accuracy | PASS | User's bounded command-delivery requirement; inspected setter and linked 30 Hz definition | Setter and periodic writer described above | Waiting mechanism is directly visible; no paper-specific implementation claim | High |
| 2. Interpretation fidelity | PASS | Requirement concerns avoidable post-authorization waiting | Existing eligibility gates retained | Does not equate selection commit with activation or promise DSD compliance | High |
| 3. Complexity proportionality | PASS | Reuse existing control path | Shared writer plus latest-only local callback | No duplicated controller or alternate safety path | High |
| 4. Implementation correctness (static scope) | PASS | Original output formula, gates and executor ownership | Active-mode check, same output conversion/history, newest-only delivery | Prevents inactive/shadow/unqualified immediate publication and stale burst replay; requires flight validation below | Medium |
| 5. Directional alignment | PASS | Preserve selection/awareness/AD contract | No worker policy, threshold or MASD changes | Changes delivery timing only after authorization, not safety decision criteria | High |

No dependency requires NOT_APPLICABLE. Overall static-scope verdict: PASS.
This does not constitute a flight-safety or latency-effectiveness verdict.

## Verification and remaining evidence gaps

- Local Release (-O3 -DNDEBUG) guidance executable and runtime build passed.
- Existing worker tests: 47/47 passed; existing ROS runtime tests: 2/2 passed
  on isolated ROS domain 91. These are regression checks, not direct proof of
  immediate setpoint timing in an activated PX4 mode.
- `git diff --check` passed.
- Pi deployment and post-change 200-second hybrid SILS have not been performed.
- Measure the existing commit/activation-to-publication traces after deployment
  to quantify the removed wait; do not count inactive preselection waiting as
  avoidable command delay. Physical response and cross-host clock uncertainty
  remain separate.
- DSD causality and improvement remain unverified. The patch removes a concrete
  timing mechanism, not a proven sole cause of the 7.835 m minimum separation.
