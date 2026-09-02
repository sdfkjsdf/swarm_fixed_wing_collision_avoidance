# Draft: AMAC Known-Delay Rollout and Residual Communication Margin

Status: **DESIGN DOCUMENT ONLY — NOT IMPLEMENTED**

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

This document records the next design question. It does not authorize code,
parameter, activation-threshold, candidate-set, or HILS changes.

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

This is a semantic concern, not yet an implemented correction. Before any code
change, the project must decide whether the existing parameter means:

1. the fixed candidate-generation-to-selection wait; or
2. a separate estimated packet-time-to-command-application latency.

Those meanings cannot share the same parameter without an explicit timing
reference.

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

The first implementation, if later approved, should keep the existing
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

## 10. Five-Axis Checklist for a Later Implementation

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
