# Five-Axis Implementation Audit: AD Activation and Dynamic Best

Date: 2026-08-31

Scope: static implementation, unit/distributed-runtime tests, and the valid
five-aircraft HILS run `amac_dynamic_best_draft4_20260831_03`.

Overall five-axis conformity verdict: **PASS**

Safety-performance verdict for this single HILS run: **FAIL**. The minimum
3-D separation was 8.279 m against DSD = 10 m for approximately 0.4 s. Source
conformity therefore must not be reported as a demonstrated 10 m safety
guarantee.

## Source Baseline

- The [2012 Auto ACAS paper](</home/hmcl/workspace/reference/paper/Lockheed_Martin_collision_avoidance/[95]Automatic Aircraft Collision Avoidance Algorithm Design for Fighter Aircraft.pdf>) Section 4.5 states that the previous best remains first choice until another maneuver is clearly superior, selection is periodic, and trajectories are updated on non-selection frames.
- The [2013 Auto ACAS paper](</home/hmcl/workspace/reference/paper/Lockheed_Martin_collision_avoidance/Development of an Automatic Aircraft Collision Avoidance.pdf>) Sections III-D and III-E state that candidates are generated at 4 Hz, trajectories are updated at 20 Hz, the previous best is performed when peers are not expected to know a new selection, and avoidance is required when `AD = PMR - MASD < 0`.
- The public baseline's numerical superiority margin and exact delay estimator are not disclosed. The explicit two-stage readiness exchange and the HILS values `Delta_cost_switch = 0.01`, `Delta_AD_switch = 1.0 m` are project choices.
- `R_dot >= 0` deactivation and removal of elapsed-time termination are also explicit project modifications, not exact Lockheed baseline behavior.

## Axis Results

### Axis 1 - Source accuracy: PASS

The implementation uses strict `AD < 0` activation, keeps 4 Hz combination
evaluation separate from 20 Hz trajectory refresh, retains the previous best
until superiority and awareness conditions pass, and does not attribute the
project margins or readiness protocol to Lockheed Martin.

### Axis 2 - Interpretation fidelity: PASS

DSD = 10 m remains inside MASD and is not added again as `AD < 10 m`.
`clearly superior` is implemented with positive project margins rather than
the evaluator's floating-point tolerance. The plan and YAML label those values
as provisional project calibration values.

### Axis 3 - Complexity proportionality: PASS

The implementation reuses the existing worker, controller, decision topic,
peer cache, proposal tuple, and `proposal_consensus_confirmed` field. It adds no
predictor, recovery state, ROS node, worker thread, or coordination topic. The
only protocol extension is switch evidence in the existing decision message.

### Axis 5 - Directional alignment: PASS

The AMAC path no longer collapses every active candidate set to the executed
candidate. A changed active proposal is evaluated against the current tuple,
is withheld when non-superior, remains a proposal while any participant is not
ready, and is committed through an atomic active-command replacement. V4's
existing fixed-candidate behavior remains isolated from this AMAC change.

### Axis 4 - Implementation correctness: PASS for the agreed policy

Evidence:

- Package build passed.
- All 142 package tests passed with zero errors, failures, or skips.
- Focused controller tests prove strict zero-AD activation, all-affected-pair
  non-closing release, invalid-sample retention, no 4.5 s/10 s time release,
  and an atomic replacement that preserves the activation episode.
- Focused worker tests prove 4 Hz/20 Hz separation, non-superior retention,
  first-stage readiness without execution, all-peer readiness commit, and no
  intermediate inactive/Flocking sample.
- The valid five-aircraft bag measured exactly 4.0 Hz full 243-combination
  evaluation and approximately 20.0 Hz intent refresh for every aircraft.
- All observed activation starts had `AD < 0`; no timeout deactivation reason
  exists.
- All 48 common qualified selected epochs had the same tuple, with zero peer
  ownship-assumption mismatch, zero active selected-slot mismatch, and zero
  unauthorized active proposal.

Coverage limitation: the valid HILS scenario produced no active candidate
replacement, so `ACTIVE(M1) -> ACTIVE(M2)` was exercised by deterministic unit
tests but not by this bag. This is a coverage limitation, not contrary runtime
evidence.

## HILS Performance Finding Outside the Five-Axis Verdict

The policy did not maintain DSD = 10 m in this run. Vehicle 3 also showed rapid
deactivate/reactivate cycles near the end because the agreed release condition
is instantaneous `R_dot >= 0` and there is intentionally no rearm or recovery
gate. This outcome is consistent with the documented accepted risks, but it
means the current parameter/policy combination is not yet a validated safety
solution.

Evidence artifacts:

- Summary: `testing_module/maneuver_selection_hils/result/summary/amac_dynamic_best_draft4_20260831_03/summary.json`
- Plot: `testing_module/maneuver_selection_hils/result/plot/amac_dynamic_best_draft4_20260831_03/actual_maneuver_overview.png`
- Video: `testing_module/maneuver_selection_hils/result/video/amac_dynamic_best_draft4_20260831_03/actual_maneuver.mp4`
- Bag: `testing_module/maneuver_selection_hils/result/rosbag/amac_dynamic_best_draft4_20260831_03`

