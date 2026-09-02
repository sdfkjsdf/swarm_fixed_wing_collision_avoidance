# Draft: AMAC Known-Delay Rollout and Residual Communication Margin

Status: **PHASE 1 IMPLEMENTED — NUMERICAL `B_comm` CALIBRATION PENDING**

Date: 2026-09-02

Return anchor before this document:

```text
289e9a5 feat: anchor exact delayed AMAC candidate rollout
```

That anchor contains the recovered delayed-rollout implementation with the
historical 200 s SILS result:

- minimum 3-D separation: `8.56639369821651 m`;
- estimated time below the 10 m DSD: `1.0 s`;
- common qualified-epoch candidate-tuple agreement: `605 / 605`;
- unit and integration tests at the anchor: `211 / 211` passed.

This document first recorded the design boundary. Implementation was later
authorized in the 2026-09-02 task and the implemented scope is recorded in
Section 12. A nonzero communication margin remains deliberately disabled until
the actual command-publication timing boundary is measured.

## 1. Problem Statement

The current trajectory cone propagates estimator covariance and diagonal
process noise:

\[
P_{k+1}=F_kP_kF_k^T+Q\Delta t.
\]

This represents estimator and predictor-model uncertainty. It does not, by
itself, represent variation in distributed proposal, communication, commit,
or command-publication latency.

The anchor also propagates the trajectory mean and covariance with a fixed
candidate-application delay of `0.25 s`:

```text
current command for 0.25 s
        -> candidate command for the remaining horizon
```

Consequently, the generated cone is conditional on that fixed delay being the
correct switch schedule. It does not explicitly bound residual latency error.

## 2. Public-Source Boundary

The public 2013 Lockheed Martin paper states that:

- new maneuvers are generated four times per second;
- maneuver combinations are selected 0.25 s after alternate generation;
- during the intervening frames, trajectories are updated from current flight
  state but maneuver commands do not change;
- all maneuver trajectories are updated 20 times per second; and
- MASD rolls up multiple uncertainties, including pilot inputs and
  transmission-delay uncertainty.

Source:

```text
/home/hmcl/workspace/reference/paper/Lockheed_Martin_collision_avoidance/
Development of an Automatic Aircraft Collision Avoidance.pdf
Section III-D, PDF page 9
```

The paper does **not** disclose:

- a distance value assigned to the 0.25 s decision delay;
- the numerical transmission-delay distribution or bound;
- a function converting transmission delay into a spatial margin; or
- the internal uncertainty roll-up used in MASD.

Therefore every numerical communication-margin equation and parameter below
is an explicit project design, not a recovered Lockheed formula.

## 3. Required Semantic Separation

Three quantities must remain distinct.

### 3.1 Estimator and predictor uncertainty

\[
U_{95,PQ}(\mathbf n,t)
=
\sqrt{\chi^2_{3,0.95}\,\mathbf n^TP_{\mathrm{rel,pos}}(t)\mathbf n}.
\]

This is the existing 95% relative-position covariance support in the
instantaneous line-of-sight direction. It must not be removed or replaced by a
communication margin.

### 3.2 Known algorithmic delay

Let

\[
t_{\mathrm{epoch}}
\]

be the candidate-generation epoch and

\[
t_{\mathrm{select,expected}}
=t_{\mathrm{epoch}}+0.25\;\mathrm{s}.
\]

If the public timing is used as the project timing contract, a trajectory
refreshed at packet time \(t_{\mathrm{packet}}\) must use the remaining known
delay:

\[
\tau_{\mathrm{remaining}}
=
\max\left(0,
t_{\mathrm{select,expected}}-t_{\mathrm{packet}}
\right).
\]

The known delay belongs in the trajectory mean and in covariance propagation
along that same mean schedule.

### 3.3 Residual communication and execution delay

Let the actual new-command publication time be

\[
t_{\mathrm{apply,actual}}
=t_{\mathrm{apply,expected}}+\delta\tau.
\]

Only the unmodeled residual \(\delta\tau\), or a defined conservative bound on
it, belongs in a separate communication-delay safety margin.

The residual covers project-defined latency between the expected selection
event and actual command publication, for example:

```text
local proposal generation
    -> ROS 2 delivery
    -> peer proposal reception
    -> all-participant tuple cross-check
    -> distributed commit
    -> PX4 command publication
```

Aircraft roll dynamics after command publication remain the responsibility of
the existing roll-rate-limited predictor. They must not be silently counted a
second time in the communication margin.

## 4. Current-Implementation Timing Concern

At the anchor, every 20 Hz intent rebuild passes the full configured
`amac_candidate_application_delay_s = 0.25` from the packet's new source
timestamp. If that parameter is intended to represent Lockheed's known
candidate-generation-to-selection interval, the absolute switch time moves
forward at every refresh.

Example:

```text
t = 0.00 s  candidate epoch begins
t = 0.20 s  trajectory is refreshed
t = 0.25 s  expected selection event
```

The source-aligned remaining known delay at `t = 0.20 s` is `0.05 s`, not a
new `0.25 s`. Restarting a full delay would instead predict candidate
application at `t = 0.45 s`.

This was the anchor's semantic concern. The approved Phase 1 implementation
resolved it by assigning the existing parameter the first meaning below:

1. the fixed candidate-generation-to-selection wait; or
2. a separate estimated packet-time-to-command-application latency.

The packet-time-to-command-publication residual remains separate and is not
encoded by this parameter.

Relevant anchor locations:

```text
collision_avoidance/src/selection/ManeuverSelectionWorker.cpp
  buildCurrentIntentSet()

collision_avoidance/include/collision_avoidance/estimation/
trajectory_prediction/TrajectoryPredict.hpp
  predictWithCommandDelay()

collision_avoidance/src/estimation/trajectory_prediction/
TrajectoryUncertainty.cpp
  propagateAlongMeanWithCommandDelay()
```

## 5. Minimal Project Design

The approved first implementation keeps the existing
covariance term and add one separately named bounded communication term:

\[
\boxed{
MASD
=
D_{\mathrm{aircraft}}
+DSD
+U_{95,PQ}
+B_{\mathrm{comm}}
}.
\]

The corresponding avoidance distance remains:

\[
AD=PMR-MASD.
\]

For the first experiment, \(B_{\mathrm{comm}}\) may be a constant distance in
meters derived offline from an explicitly selected residual-delay bound:

\[
B_{\mathrm{comm,const}}
=
v_{\mathrm{rel,design}}\Delta\tau_{\mathrm{design}}
+\frac{1}{2}
a_{\mathrm{rel,design}}\Delta\tau_{\mathrm{design}}^2.
\]

This is a conservative project safety budget. It is not a probabilistic 95%
covariance term and must not be labeled as one.

The units must be explicit:

| Quantity | Unit | Meaning |
|---|---:|---|
| `known_delay_s` | s | Deterministic delay represented in the mean rollout |
| `residual_delay_bound_s` | s | Unmodeled latency beyond the expected application time |
| `relative_speed_bound_mps` | m/s | Design bound used for delay-to-distance conversion |
| `relative_acceleration_bound_mps2` | m/s^2 | Design bound used for delay-to-distance conversion |
| `communication_delay_margin_m` | m | Pairwise MASD increment |

If relative speed and acceleration are already pairwise relative quantities,
the resulting margin is added once per pair. It must not also be added once per
aircraft.

## 6. Why the Two Uncertainty Terms Must Coexist

Replacing \(U_{95,PQ}\) with \(B_{\mathrm{comm}}\) would remove estimator and
model uncertainty from MASD. Adding the full known 0.25 s motion to both the
mean trajectory and \(B_{\mathrm{comm}}\) would count the same delay twice.

The required accounting identity is:

```text
known deterministic delay
    -> mean trajectory and its covariance linearization schedule

estimator/model stochastic uncertainty
    -> U95,PQ

unmodeled residual latency bound
    -> B_comm
```

No physical effect may appear in more than one branch of this accounting.

## 7. Initial Scope Limit

The first approved implementation must not introduce:

- a communication state into the seven-state predictor;
- online latency-distribution estimation;
- a time-varying or direction-varying communication ellipsoid;
- a replacement for the existing \(P/Q\) propagation;
- a change from `AD < 0` activation;
- a change to the 10 m DSD;
- a change to the seven-candidate exhaustive diagnostic mode;
- a V4/Mode-B behavior change; or
- an undocumented addition of the same margin to both cone support and MASD.

The minimal experimental artifact is one named scalar pairwise MASD increment
plus diagnostics that expose its contribution.

## 8. Measurement Boundary

The constant cannot be justified from the public paper. It must be derived
from this system's rosbag evidence.

Required event timestamps are:

```text
candidate epoch generated
local best proposal emitted
last required peer proposal received
tuple committed
ownship command published
```

For each successful active command change, record:

\[
\tau_{\mathrm{actual}}
=
t_{\mathrm{command\ published}}
-t_{\mathrm{timing\ reference}}.
\]

The timing reference must be chosen before measuring. Candidate epoch,
expected selection time, and proposal time are not interchangeable.

The design must report at least sample count, median, p95, maximum, missing
event count, and the exact definition of the selected bound. A p95 value is a
calibration choice; a hard safety bound requires a justified upper bound.

## 9. Future Acceptance Conditions

No implementation is complete until all of the following are demonstrated:

1. A 20 Hz trajectory refresh does not restart a known absolute selection
   delay when the intended switch time is unchanged.
2. Sender mean, receiver reconstruction, roll propagation, and covariance
   propagation use the same switch schedule.
3. `communication_delay_margin_m = 0` reproduces the anchor's AD/MASD results.
4. Increasing the margin by \(b\) meters increases MASD by exactly \(b\) and
   decreases AD by exactly \(b\), with no change to PMR or \(U_{95,PQ}\).
5. The margin is applied exactly once per evaluated aircraft pair.
6. V4/Mode-B behavior is unchanged.
7. Unit and integration tests remain fully passing.
8. A repeated five-aircraft 200 s SILS run reports minimum separation, DSD
   violation duration, activation count, proposal/commit latency, and nuisance
   activation changes against anchor `289e9a5`.

## 10. Five-Axis Checklist for the Implementation

| Axis | Pass condition |
|---|---|
| Source accuracy | Public Lockheed statements and project-specific equations are labeled separately. |
| Interpretation fidelity | The paper is not claimed to disclose a numerical delay margin or conversion formula. |
| Complexity proportionality | The first implementation adds only the minimum scalar margin and required diagnostics. |
| Implementation correctness | Timing references, units, one-time accounting, and zero-margin equivalence are tested. |
| Directional alignment | Known motion is predicted; only residual uncertainty is added to the safety budget. |

## 11. Decisions Still Open

The following values are intentionally not selected in this document:

- the timing reference for the actual-latency measurement;
- the expected application-time model after selection;
- p95 versus a hard upper-bound policy;
- \(v_{\mathrm{rel,design}}\);
- \(a_{\mathrm{rel,design}}\);
- the first numerical value of \(B_{\mathrm{comm}}\); and
- whether a later phase should replace the scalar bound with rollout-difference
  support.

These are calibration and design decisions. Guessing them in the first code
change would erase the distinction this document is intended to preserve.

## 12. Implemented Phase 1 and Measured Boundary

The approved first implementation makes two minimal changes.

### 12.1 Absolute known-delay schedule

The existing parameter remains named
`amac_candidate_application_delay_s`, but its implemented meaning is now:

```text
candidate epoch generation
    + configured known delay
    = one absolute expected application time
```

Every 20 Hz packet carries only the remaining delay to that absolute time.
For example, a packet refreshed `0.20 s` after an epoch that uses a `0.25 s`
known delay carries `0.05 s`; it does not start a new `0.25 s` delay.

Sender mean generation, transmitted `command_delay_s`, receiver mean
reconstruction, roll-rate-limited propagation, and covariance propagation all
continue to use the same transmitted switch schedule.

### 12.2 Independent residual margin path

The evaluator now exposes:

```text
amac_communication_delay_margin_m
```

and computes:

\[
MASD=D_{aircraft}+DSD+U_{95,PQ}+B_{comm}.
\]

The value is validated as finite and nonnegative, added exactly once per pair,
published in `ManeuverSelectionDecision`, and reported by the offline analyzer.
Its configuration default is `0.0 m`, so this phase does not claim that a
numerical communication bound has already been calibrated.

### 12.3 Existing-bag measurement

The historical anchor bag was measured using the following available event
definition:

```text
first local rosbag record containing a valid proposal
    -> first local rosbag record containing new_best_accepted
       for the same epoch, proposal timestamp, and candidate-ID tuple
```

Results over `1,675` matched commits were:

| Statistic | Measured value |
|---|---:|
| median | `0.029754793 s` |
| p95 | `0.049835249 s` |
| maximum | `0.060342894 s` |
| commit without matching proposal | `0` |

This is a proposal-observation-to-commit-observation metric. It is **not** yet
the required expected-selection-to-PX4-command-publication residual, because
the historical message did not record the latter event. Consequently these
numbers were not converted into a nonzero `B_comm` value.

### 12.4 Verification completed

- package build: passed;
- all `19` CTest targets: passed;
- `193` reported test cases: `0` failures;
- 20 Hz refresh regression: `0.10 s -> 0.05 s` remaining delay for the same
  absolute `0.50 s` switch time;
- zero-margin behavior: retained by the existing zero default;
- nonzero-margin unit regression: adding `2.5 m` changes only
  `MASD += 2.5 m` and `AD -= 2.5 m`, while PMR and `U95_PQ` remain unchanged.

### 12.5 Still required before enabling a nonzero value

1. instrument or otherwise identify actual ownship PX4 command publication;
2. measure residual latency from the selected expected timing reference;
3. choose and document p95 versus hard-bound policy and the relative-motion
   bounds;
4. configure a nonzero `B_comm` only after that calibration; and
5. repeat the five-aircraft 200 s SILS comparison against anchor `289e9a5`.

### 12.6 Zero-margin 200 s SILS result

The first valid five-aircraft pentagon run kept
`amac_communication_delay_margin_m = 0.0` and therefore isolated the absolute
known-delay correction:

| Metric | Anchor packet-relative 0.25 s | Absolute remaining-delay schedule |
|---|---:|---:|
| minimum 3-D separation | `8.566393698 m` | `9.270009854 m` |
| estimated time below 10 m DSD | `1.0 s` | `0.2 s` |
| activation starts | `30` | `13` |
| starts with a safe combination available | `20` | `9` |
| common qualified-epoch tuple agreement | `100%` | `100%` |

The timing correction improved this sample but did not satisfy the 10 m DSD;
the safety-performance result therefore remains a fail.

The new bag measured `1,450` matched proposal-to-commit observations:

| Statistic | Value |
|---|---:|
| median | `0.029705306 s` |
| p95 | `0.049772330 s` |
| maximum | `0.060591934 s` |

For the next HILS-only sensitivity run, a provisional hard-bound experiment
uses:

\[
\Delta\tau_{design}=0.10\,s,
\quad v_{rel,design}=2V_{max}=50\,m/s,
\quad a_{rel,design}=2a_{lat,max}=23.374\,m/s^2,
\]

which gives:

\[
B_{comm}=50(0.10)+\frac12(23.374)(0.10)^2
=5.11687\,m.
\]

The test value is rounded upward to `5.2 m`. The `0.10 s` budget combines the
observed `0.0606 s` proposal-to-commit maximum with one nominal 30 Hz
Formation setpoint-update period and rounding. This is a project sensitivity
bound, not a source-recovered Lockheed value and not yet a flight-qualified
hard latency bound. Production configuration remains at `0.0 m`.

### 12.7 Rejected 5.2 m sensitivity result

The 200 s pentagon run with the HILS-only `5.2 m` override did not improve
closed-loop safety or flocking:

| Metric | `B_comm = 0.0 m` | `B_comm = 5.2 m` |
|---|---:|---:|
| minimum 3-D separation | `9.270009854 m` | `8.823756289 m` |
| estimated time below 10 m DSD | `0.2 s` | `0.3 s` |
| final position standard deviation | `43.6255 m` | `159.0386 m` |
| final velocity standard deviation | `0.2380 m/s` | `18.0015 m/s` |
| common qualified-epoch tuple agreement | `100%` | `100%` |

The large scalar margin caused substantially more activation and switching,
and the resulting closed-loop geometry was worse despite the more conservative
instantaneous AD value. Therefore `5.2 m` is rejected as a configured value.
This also demonstrates that scalar-margin magnitude is not monotonically
related to multi-agent closed-loop safety and must be calibrated by SILS rather
than enabled directly from the kinematic upper-bound calculation.

### 12.8 Rejected 1.0 m sensitivity result

A smaller `1.0 m` HILS-only override was also rejected:

| Metric | `B_comm = 0.0 m` | `B_comm = 1.0 m` |
|---|---:|---:|
| minimum 3-D separation | `9.270009854 m` | `1.038924862 m` |
| estimated time below 10 m DSD | `0.2 s` | `3.7 s` |
| final position standard deviation | `43.6255 m` | `37.5327 m` |
| final velocity standard deviation | `0.2380 m/s` | `0.3813 m/s` |
| activation starts | `13` | `27` |
| active candidate switches | `7` | `26` |
| common qualified-epoch tuple agreement | `100%` | `100%` |

This run shows that selecting a scalar from the observed `0.73 m` DSD
shortfall is invalid: the margin changes the distributed closed-loop maneuver
sequence, not only the instantaneous separation number. No tested nonzero
value is accepted. Production and default HILS configuration therefore remain
at `0.0 m`.

## 13. Five-Axis Audit After Implementation

| Axis | Status | Finding |
|---|---|---|
| Source accuracy | PASS | The 4 Hz generation, 0.25 s post-generation selection, 20 Hz state refresh, and MASD delay-uncertainty statements are directly present in the 2013 source. |
| Interpretation fidelity | PASS | Absolute remaining delay and all `B_comm` equations and numbers are explicitly identified as project design, not disclosed Lockheed mathematics. |
| Complexity proportionality | PASS | The implementation adds one scalar evaluator parameter, one diagnostic field, timing correction, analyzer summary, and focused tests; it does not add predictor states or a second uncertainty model. |
| Implementation correctness | PASS for wiring; calibration INDETERMINATE | Unit tests prove one-time MASD accounting and zero-margin behavior, and all package tests pass. Actual PX4 command-publication timing is not directly instrumented, and both nonzero SILS values failed. |
| Directional alignment | PASS | Known motion remains in the mean rollout, estimator/model uncertainty remains in `P/Q`, and only residual latency is allocated to the separate MASD term. |

Overall verdict:

```text
implementation infrastructure: PASS
nonzero B_comm calibration: INDETERMINATE / not accepted
closed-loop 10 m DSD performance: FAIL for every tested run
```
