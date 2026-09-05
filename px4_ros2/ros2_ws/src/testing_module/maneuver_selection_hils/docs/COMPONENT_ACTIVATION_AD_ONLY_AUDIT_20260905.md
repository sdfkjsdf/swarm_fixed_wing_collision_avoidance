# Component activation delivery and AD-only selection

## Scope and source boundary

Authoritative requirement: the current user instruction to fix component
activation-event loss and remove the Formation/rejoin objective from maneuver
selection. Preserve AD < 0, 4 Hz candidate selection, peer qualification, MASD,
Formation guidance and the existing termination/post-release safety checks.
This patch is a project repair, not a claim that Lockheed disclosed this ROS
event protocol. Baseline inspected: a292fc6. Earlier uncommitted immediate
Formation publication changes are preserved and not counted as this repair.

## Evidence baseline

In the baseline run `hybrid_formation_a292fc6_input_history_200s_20260905_01`,
the 3–4 minimum separation was 7.835 m at 101.7 s. Central decision records show
execution requests at about 99.360/99.784 s and same-host publication traces at
99.369/99.789 s. These observations distinguish activation request skew from
post-request publication wait; central receipt is not the exact worker event time.

Baseline `ManeuverSelectionWorkerActivation.cpp::updateActivationState()`
unconditionally cleared every `activation_start_pending` even when
`buildActivationSample()` returned false. `ManeuverActivationController::update()`
rejects invalid samples. Thus one evaluation could both reject activation and
erase the only edge needed to retry. At the relevant time, vehicle 4's pair
monitor traces include incomplete rounds. That is consistent with this defect,
but the bag does not record the exact vehicle-4 event consumption branch;
attribution of the entire 0.42 s remains unproven.

The same run's 99.780 s switch reduced minimum AD from 1.999 to 0.686 m and
increased reciprocal cost from 0.747 to 1.760. Its rejoin-objective-applied flag
was true. Baseline `ManeuverSelectionWorkerTrajectory.cpp::evaluateCurrentSet()`
explicitly allowed `rejoin_improves` independently of `clearlySuperior()`.

## Implemented change

1. `RemoteDecisionCache` retains the activation edge's original epoch, tuple
   and valid mask. Later heartbeats no longer redefine the event identity.
2. An invalid activation sample preserves the event. A valid sample consumes
   it under the existing qualification/epoch/tuple/component conditions.
   A newer peer decision reporting inactive cancels pending activation, so a
   completed avoidance is not replayed after local data recovery.
3. Removed the rejoin-objective construction, isolated nominal-nearest choice,
   nominal-error scoring, rejoin-based superiority exception, and the dedicated
   `ManeuverRejoinObjective` API from worker, component and exhaustive evaluators.
4. Safe tuples use reciprocal AD cost; unsafe fallback and existing superiority
   margins are unchanged. No added positive activation threshold or distance
   margin. Formation control and termination/post-release checks are unchanged.
5. Historical message/diagnostic fields remain readable. They no longer enable
   a selection objective; objective-applied is false and objective cost is NaN.
   No dormant runtime switch can restore the removed objective.

## Five-axis static audit

All source paths below refer to the collision_avoidance package.

| Axis | Status | Source / requirement evidence | Implementation evidence | Reason and impact | Confidence |
|---|---|---|---|---|---|
| 1. Source accuracy | PASS | Current bounded user instruction; baseline branches and bag above | Worker activation/cache and selector files | Separates inspected defects from unproven full-encounter causality | High |
| 2. Interpretation fidelity | PASS | Preserve qualification and AD activation, remove rejoin override | Original eligibility conditions retained; rejoin scoring deleted | Does not interpret every commit as permission to activate or promise simultaneous execution | High |
| 3. Complexity proportionality | PASS | Direct repair, no bypass architecture | Small saved-event identity; removal of objective API and computations | No new timer, thread, consensus phase or margin | High |
| 4. Implementation correctness | PASS within bounded static/regression scope | Invalid samples cannot activate; ended peers must not replay | Deferred-event/heartbeat and ended-peer regression fixtures; shared AD evaluator tests | Repairs loss without treating incomplete data as valid, and removes nominal preference | High |
| 5. Directional alignment | PASS | AD-based selection and valid coordinated activation | `clearlySuperior()` and peer gates retained; no `rejoin_improves` or objective API remains | Both requested changes are implemented without changing the safety boundary | High |

No axis dependency is unresolved within this bounded code scope. Overall static
and regression verdict: PASS. General DSD safety is outside that verdict; the
single hybrid run below provides bounded performance evidence only.

## Verification / limitations

- Release guidance executable built with the updated worker and runtime.
- Worker: 49/49 tests passed, including invalid-sample recovery across a newer
  heartbeat, ended-peer cancellation, matching/different component behavior,
  and the existing input-history tests.
- Pairwise evaluator: 7/7 passed; combination evaluator: 23/23 passed.
- No calibration, firmware configuration or runtime cadence changed.
- Runtime integration: 2/2 tests passed (81/81 total related tests).
- Pi deployment and the 200-second hybrid run below completed successfully.
- The 0.275 s stale threshold appears only in the regression fixture to create
  an invalid sample deterministically; it is not a new runtime tuning value.
- Remaining evidence needed for performance attribution: new hybrid traces
  quantifying activation skew and DSD, with the same common-time analysis basis.

## 200-second hybrid verification

Run: `hybrid_formation_event_adonly_200s_20260905_01`.
Pi vehicle 0 and local vehicles 1–4 used the same updated source. Release Pi
image: `collision-avoidance:event-adonly-20260905` (image SHA256
`95ed5bec3748c26683594d16509aca354f32ea6d7511cbdae27f92aa715da557`).
This run includes the command-delivery fix described in the companion audit.
AD < 0, DSD 10 m, communication margin 0 m, and roll-response constant 0.5 s
were unchanged from the preceding comparison run.

- Formation pentagon recording: 200 s; common Formation evaluation: 181.0 s.
- Minimum sampled 3D separation: 11.097392226 m (pair 1–4, time 13.1 s).
- DSD violation: 0 samples / 0 estimated seconds at 10 Hz analysis.
- Previous input-history run: 7.835221571 m minimum / 0.6 s violation.
- Final fleet position/velocity standard deviation: 54.008983043 m /
  0.587893346 m/s (previous: 38.705361565 m / 0.313411311 m/s).
- Each node evaluated 723 consecutive selection epochs at 0.25 s spacing.
- Pi evaluation median/p95/max: 5.448 / 7.936 / 8.960 ms; all-node max 9.947 ms.
  These are instrumented evaluator durations, not end-to-end application delay.
- Common component tuple agreement: 723/723; proposal tuple agreement: 688/688.
- First observed activation transitions spanned 92.2 ms in central bag arrival
  time. This is not actuator application skew or proof of simultaneity.
- Pi-minus-PC clock offset was approximately +57.33 ms before and +55.04 ms
  after; no clocks were adjusted.

Artifacts are in the ignored HILS `result/summary/<run>` and `result/plot/<run>`
directories: summary, CSV, PNG, clock probes, source patch and run manifest.
The single sampled run does not establish repeatability, continuous-time safety,
or which individual bundled change caused the improvement. Final fleet spread
increased despite the improved minimum separation. No video was generated.
