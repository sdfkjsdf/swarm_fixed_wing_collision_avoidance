# Stateful Maneuver Activation Remediation

Status: **implemented, tested, and five-axis audited**

Date: 2026-08-28

## 1. Scope

This task corrects one specific source-conformity defect: the runtime formerly
used each 0.25 s maneuver-selection result as both a new proposal and the
immediately applied PX4 command. It therefore had no independent activation
lifecycle and could replace or stop the predicted 4.5 s maneuver at the next
selection epoch.

This task does not claim to reproduce the complete undisclosed Auto ACAS
implementation and does not attempt to solve the remaining distributed
selection-consensus or 10 m DSD performance failures.

## 2. Source-Backed Contract

The implementation boundary is based on:

- Turner et al. (2012), Sections 4.5 and 4.6: selection is periodic, the
  retained maneuver commands are not changed on non-selection frames, and the
  avoidance trajectory continues to be transmitted while avoidance is active.
- Wadley et al. (2013), Sections III-D and III-E: maneuver selection and AMAC
  monitoring are separate functions; negative AD requires activation; the
  disclosed baseline deactivation conditions are sufficient separation rate
  or a 4.5 s timeout.
- The reviewed study HTML, Sections 6 and 7: the 0.25 s interval is a
  coordination/selection interval, not an execution-command replacement
  guarantee.

The public sources do not disclose the exact superiority margin, complete
hysteresis, PX4 lifecycle gate, or rearm implementation.

## 3. Implemented State Boundary

```text
20 Hz intent/AD monitoring ──> activation controller ──> PX4 override
           ^                         |
           |                         +-- inactive
4 Hz selection proposals             +-- active(command latched)
                                     +-- deactivated
```

- Selection continues to evaluate candidates at its existing cadence.
- `AD < 0` may activate only after the point-convergence execution gate opens.
- The selected ownship candidate and input are latched on activation.
- Later 0.25 s selection results cannot replace the active PX4 command.
- Active execution ends only when all affected threats satisfy the disclosed
  separating-rate criterion or when 4.5 s elapses.
- Invalid or missing monitor samples cannot interrupt an active command.

## 4. Explicit Project Policies

The following are minimal project policies and are not represented as exact
published Auto ACAS logic:

- The existing 20 Hz trajectory refresh is also the AMAC monitoring cadence.
- In this fixed-wing HILS, activation is armed only after all five vehicles
  finish fixed-wing transition and stabilization and enter the
  `point_convergence` execution phase.
- After deactivation, the cone must first clear `AD >= 0` before a later
  negative-AD sample can start a new conflict episode. This prevents a single
  conflict from being retriggered every 50 ms.
- A positive range-rate threshold of 30.48 m/s is the sign-converted form of
  the disclosed approximately 100 ft/s separating criterion.

## 5. Test Evidence

- Collision-avoidance package: 76 tests, 0 failures.
- Unit tests verify command latching, separating and timeout deactivation,
  invalid-sample retention, and clear-before-rearm behavior.
- Worker tests verify activation between 0.25 s selection epochs and verify
  that negative AD cannot activate before the execution gate opens.
- ROS runtime tests verify intent exchange and stateful decision publication.
- Headless five-aircraft exhaustive HILS:
  `point_avoidance_exhaustive_stateful_20260828_03`.

The final HILS analyzer uses the latest of the five point-convergence activation
times as evaluation time zero. All preflight and transition records are
excluded from metrics, plots, decision diagnostics, and video.

Observed activation-state results:

- all five vehicles entered fixed-wing point-convergence before activation;
- first activations occurred about 5.9 s or later after the common evaluation
  epoch;
- active candidate switches: 0 for every vehicle;
- the earlier kilometer-scale preflight AD activations were eliminated;
- immediate 50 ms reactivation after deactivation was eliminated.

## 6. Five-Axis Re-Audit

Audit target: the selection-versus-activation remediation in this document.

| Axis | Verdict | Evidence |
|---|---|---|
| Source accuracy | PASS | Selection, activation, negative-AD trigger, separating termination, and 4.5 s timeout are mapped to the cited sections without claiming undisclosed thresholds. |
| Interpretation fidelity | PASS | The 0.25 s interval is used only for proposals. The 20 Hz monitor, fixed-wing gate, active-command latch, and clear-before-rearm rule are explicitly labeled as project policies. |
| Complexity proportionality | PASS | One ROS-independent state controller was added and wired into the existing worker, runtime, and Formation lifecycle. No new ROS node or worker thread was introduced. |
| Implementation correctness | PASS | Unit, worker, ROS runtime, and five-aircraft HILS evidence agree: preflight cannot activate, active commands do not switch, and only defined state exits stop execution. |
| Directional alignment | PASS | Candidate selection and execution activation are now separate states; a periodic selection event no longer directly toggles the PX4 override. |

## 7. Separate Open Failures

The conformity PASS above is not a flight-safety acceptance result. In the
same final HILS run, the point-convergence evaluation still reported:

- minimum actual 3D separation: approximately 1.19 m;
- DSD requirement: 10 m;
- same-epoch distributed tuple agreement: approximately 38.3%.

These remain explicit follow-up problems in distributed selection coordination
and avoidance effectiveness. They are not hidden by the activation-state
audit.
