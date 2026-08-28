# Static Maneuver-Combination Evaluation — Step Scope

Status: **implemented and statically verified (2026-08-28)**  
DSD for this step: **10.0 m**

## 1. Objective

Implement one static evaluation cycle for two cooperating aircraft:

```text
Aircraft A: Current Best + Alternate 1 + Alternate 2
Aircraft B: Current Best + Alternate 1 + Alternate 2
                         ↓
              3 × 3 = 9 combinations
                         ↓
       PMR → MASD → AD → feasibility/cost table
```

This step evaluates an already prepared three-candidate set. It does not select
the three candidates from the seven lateral maneuver candidates.

### 1.1 Evidence and project-policy boundary

The authoritative source baseline for this step is:

- Turner et al. (2012), *Automatic Aircraft Collision Avoidance Algorithm
  Design for Fighter Aircraft*, Sections 4.1, 4.4, and 4.5;
- Wadley et al. (2013), *Development of an Automatic Aircraft Collision
  Avoidance System for Fighter Aircraft*, Threat SWIM and Sections III-B
  through III-E.

The papers directly support common-time threat propagation, a 4.5 s TPA
horizon, 46 local points at 0.1 s, three trajectories per cooperating aircraft,
nine two-aircraft combinations, PMR/AD-based scoring, and MASD containing
half-wingspans, DSD, and an uncertainty roll-up.

The compact four-time trajectory packet, the three PMR reporting windows, the
covariance-to-distance equation in Section 5.2, and nonpositive-AD numerical
handling are explicit project decisions. They are not presented as undisclosed
Auto ACAS production details.

### 1.2 Five-axis audit corrections applied before implementation

The pre-implementation audit found three ways the earlier draft exceeded or
diverged from the agreed step. They were corrected as follows:

- **Over-interpretation:** the draft treated continuous cubic stationary-point
  solving as though it were required by the papers. The public papers disclose
  the local 46-point representation and the search for minimum distance in time
  and space, but not that numerical method. This step therefore reuses the 46
  existing synchronous samples and labels the three-window split as project
  policy.
- **Unnecessary new development:** a new spline minimizer, union-breakpoint
  builder, worker pool, trajectory reconstructor, and covariance propagator
  were not justified for only `9 x 46 = 414` range samples. The implementation
  reuses `TrajectoryIntentReceiver`, its `reconstructed_mean`, and its existing
  propagated `cone`; only the pairwise evaluator is new.
- **Over-conservative directional drift:** exact timestamp equality and
  rejecting every nonpositive-AD combination would conflict with the source
  direction. Valid delayed data are aligned to a common evaluation time. If all
  valid combinations have `AD <= 0`, the evaluator still selects the largest
  (least-negative) AD instead of returning no maneuver.

The source/project-policy boundary above and the deferred runtime lifecycle in
Section 3 prevent source claims from being expanded into undisclosed production
behavior.

## 2. A-to-B Data Contract

A does **not** transmit all 46 trajectory points. For each candidate, A sends
the existing compact trajectory-intent packet:

- source timestamp;
- candidate ID;
- initial predictor state `x0`;
- initial covariance `P0`;
- compact nominal trajectory samples:
  - position and velocity at `t = 0.0 s`;
  - position at `t = 1.5 s`;
  - position at `t = 3.0 s`;
  - position and velocity at `t = 4.5 s`.

B performs the following locally for each received candidate:

1. validate the timestamp;
2. reconstruct the nominal trajectory with the existing cubic spline;
3. create the local 46-point, 0.1 s trajectory representation;
4. propagate `P0` along the reconstructed mean;
5. create the 46-point trajectory cone used by the evaluator.

The 46 points are a local algorithm representation, not an inter-aircraft
datalink payload.

## 3. Common-Time Alignment

Different valid timestamps are not, by themselves, a rejection condition.
Received cooperating-aircraft data shall be propagated to B's current ownship
time before combination evaluation:

```text
A data at t_A
      ↓
timestamp validity check
      ↓
propagate/reconstruct to B evaluation time t_eval
      ↓
common-time A candidate representation
```

The evaluator compares A and B positions only at the same absolute future
time. The following concepts must remain separate:

- `source_timestamp`: time at which a candidate trajectory was generated;
- `selection_epoch`: candidate-set generation cycle;
- `evaluation_time`: B's current ownship time used as the common reference.

For this static step, valid delayed data will be aligned to `evaluation_time`.
Future timestamps and data older than the three-second static staleness limit
will be reported as invalid input.

The stateful runtime behaviors below are not implemented in this step:

- caching and propagating the last good dataset;
- detecting abnormal timestamp jumps between successive packets;
- recovery hysteresis after a failed threat;
- removing a threat three seconds after the last good dataset.

The alignment result shall expose a status so those behaviors can be added
without changing the cost evaluator interface.

## 4. Static 3 × 3 Evaluation

For every pair of candidates `(A_i, B_j)`, calculate the synchronous 3D
nominal separation:

\[
R(t) = \left\|p_A(t)-p_B(t)\right\|_2.
\]

### 4.1 Segment-wise PMR reporting using the existing 46 points

Reuse the 46 reconstructed mean/cone points already produced by
`TrajectoryIntentReceiver`. Group their synchronous nominal-distance samples
into the three intervals associated with the compact trajectory knots:

\[
I_1=[0.0,1.5],\qquad
I_2=[1.5,3.0],\qquad
I_3=[3.0,4.5]\ \mathrm{s}.
\]

Use a half-open ownership convention for the first two windows so a knot is
evaluated once:

```text
PMR_1: indices  0..14  (0.0 <= t < 1.5 s)
PMR_2: indices 15..29  (1.5 <= t < 3.0 s)
PMR_3: indices 30..45  (3.0 <= t <= 4.5 s)
```

After common-time alignment shortens the available overlap, evaluate only the
valid samples in each window. A window with no valid sample is reported as
unavailable and is not allowed to win the PMR comparison.

For every available window, calculate:

\[
PMR_s=\min_{t_k\in I_s}\left\|p_A(t_k)-p_B(t_k)\right\|_2,
\qquad
t_s^*=\arg\min_{t_k\in I_s}\left\|p_A(t_k)-p_B(t_k)\right\|_2.
\]

This produces three local results for one trajectory combination:

\[
(PMR_1,t_1^*),\qquad
(PMR_2,t_2^*),\qquad
(PMR_3,t_3^*).
\]

Select the global nominal PMR by comparing the three segment results:

\[
s^*=\arg\min_{s\in\{1,2,3\}}PMR_s,
\]

\[
PMR=PMR_{s^*},\qquad t_{PMR}=t_{s^*}^*.
\]

The first implementation uses deterministic fixed-size loops. It does not add
a polynomial root solver or a thread pool. Continuous spline minimization may
be studied later as a separately validated refinement, including its required
trajectory-minimization uncertainty.

For two aircraft with three candidates each, one static evaluation contains

\[
9\ \text{trajectory combinations}\times3\ \text{PMR windows}
=27\ \text{window PMR results}.
\]

### 4.2 PMR-first evaluation order

The evaluator must compare `PMR_1`, `PMR_2`, and `PMR_3`; it must not calculate
three independently ranked AD values. Only after the global nominal PMR and
its associated time have been selected shall the evaluator calculate
uncertainty, MASD, and AD:

```text
segment 1 -> PMR_1, t_1*
segment 2 -> PMR_2, t_2*
segment 3 -> PMR_3, t_3*
              |
              v
       global PMR and t_PMR
              |
              v
     MASD(t_PMR) and final AD
```

Therefore the final Predicted Minimum Range is:

\[
PMR = \min(PMR_1,PMR_2,PMR_3).
\]

Record each segment's local PMR/time for diagnosis, as well as the selected
global `PMR`, PMR-window ID, and absolute/time-offset value associated with
`t_PMR`.

## 5. MASD Components

The requested 10 m value applies only to DSD. Uncertainty is not removed, and
the total MASD is not fixed to 10 m.

\[
MASD(t_{PMR}) = R_{aircraft} + \underbrace{10.0}_{DSD}\; + U_{95}(t_{PMR}).
\]

### 5.1 Aircraft size

Use the sum of the two aircraft half-wingspans:

\[
R_{aircraft}=\frac{b_A}{2}+\frac{b_B}{2}.
\]

The values shall be evaluator parameters rather than hidden constants. For the
Gazebo `standard_vtol` static test, the local SDF collision width is 2.144 m;
two identical vehicles therefore produce:

\[
R_{aircraft}=1.072+1.072=2.144\ \mathrm{m}.
\]

### 5.2 Current uncertainty contribution

At the nominal PMR time, combine the two propagated position covariances
under an independence assumption:

\[
P_{rel}=P_{p,A}(t_{PMR})+P_{p,B}(t_{PMR}).
\]

As an explicit project policy, project the relative covariance onto the
nominal line of sight and use the 95% three-dimensional confidence convention
currently used for the trajectory-cone representation:

\[
\hat r=\frac{p_B-p_A}{\|p_B-p_A\|},
\qquad
U_{95}=\sqrt{\chi^2_{3,0.95}\,\hat r^T P_{rel}\hat r},
\]

where `chi_squared(3, 0.95) ≈ 7.815`.

This is the project's first explicit covariance-to-distance roll-up. It is not
claimed to be the undisclosed exact Auto ACAS implementation. Navigation and
predictor/process uncertainty represented by the current cone are included.
Additional published categories such as spline resolution, datalink
resolution, delay, and coupler-model uncertainty are not numerically modeled
in this step.

## 6. AD, Feasibility, and Cost

Use the paper-nearest ordering documented for this step: find the nominal PMR
first, then subtract the associated MASD at that PMR sample time.

\[
AD = PMR - MASD(t_{PMR}).
\]

For the two-aircraft static evaluator:

- `AD > 0`: feasible, `cost = 1 / AD`;
- `AD <= 0`: separation budget violated and reciprocal cost is reported as
  undefined rather than used for ranking;
- select the valid combination with the largest AD;
- when positive-AD combinations exist, this is equivalent to selecting the
  lowest positive reciprocal cost;
- if every AD is nonpositive, still select the largest (least-negative) AD so
  the avoidance system retains the best available maneuver;
- return `no best` only when every combination is invalid because required
  trajectory data are unavailable;
- an exact AD tie retains the first combination in deterministic row order for
  this static report only; operational switching hysteresis remains out of
  scope.

The handling of `AD <= 0` is an explicit project policy because the public
papers do not disclose the production singular/negative-cost handling. The
fallback preserves the paper's stated objective of choosing the trajectory
combination with the largest predicted minimum AD.

## 7. Required Result Table

The static runner shall print all nine rows with at least these columns:

```text
Combination
A candidate ID
B candidate ID
PMR [m]
PMR window
PMR sample/time
PMR_1 / PMR_2 / PMR_3 [m]
Aircraft-size margin [m]
DSD [m]
95% uncertainty margin [m]
MASD [m]
AD [m]
Feasible
Cost
Selected best
```

Intermediate values must remain visible so an unsafe result cannot be hidden
inside a single aggregate cost.

## 8. Explicitly Out of Scope

This implementation step does not include:

- threat isolation or threat ranking;
- pre-selection from the seven lateral maneuver candidates;
- 4 Hz alternate generation;
- 20 Hz runtime trajectory refresh and broadcast;
- ROS batch transport changes;
- continuous spline root solving or parallel execution infrastructure;
- last-good-data lifecycle and the three-second removal timer;
- recovery hysteresis;
- current-best switching hysteresis;
- transmission-delay coordination before accepting a new best;
- AMAC activation;
- three-or-more-aircraft combinations;
- Monte Carlo or HILS validation.

Those behaviors will be connected only after the static nine-combination
calculation and its complete result table have been verified.

## 9. Implemented Reuse Boundary

The implementation is deliberately split at the existing A-to-B intent output:

```text
existing TrajectoryIntentSender
        -> existing compact packet
        -> existing TrajectoryIntentReceiver
        -> existing 46-point reconstructed mean + propagated cone
        -> new ManeuverCombinationEvaluator
        -> 9-row static result table
```

Implemented files:

- `collision_avoidance/selection/ManeuverCombinationEvaluator.hpp` — fixed-size
  input/result contract and evaluator interface;
- `selection/ManeuverCombinationEvaluator.cpp` — common-time sampling,
  three-window PMR, MASD, AD, best-selection, and result-table formatting;
- `test/test_maneuver_combination_evaluator.cpp` — six focused static tests;
- `trajectory_prediction_hils/src/tools/static_maneuver_combination_main.cpp` —
  non-ROS deterministic 3 x 3 snapshot runner using the existing sender and
  receiver.

No second trajectory reconstruction, covariance propagation, cone generation,
ROS node, thread pool, or continuous root solver was added.

## 10. Verification Record

Commands verified on 2026-08-28:

```bash
colcon build --packages-select collision_avoidance trajectory_prediction_hils
colcon test --packages-select collision_avoidance trajectory_prediction_hils
ros2 run trajectory_prediction_hils static_maneuver_combination
```

Results:

- both packages built successfully;
- 46 tests passed with 0 failures, including all six evaluator tests;
- the static runner printed all nine combinations and all intermediate margins;
- in the deterministic head-on snapshot every AD was nonpositive, and the
  largest AD (`-11.581 m`) was still selected as the best available result.

This is a static algorithm-wiring result, not yet evidence of HILS performance
or calibrated 95% coverage.
