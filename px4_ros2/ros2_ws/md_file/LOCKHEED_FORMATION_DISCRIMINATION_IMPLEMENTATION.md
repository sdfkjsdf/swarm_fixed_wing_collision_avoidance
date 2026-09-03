# Lockheed Baseline Formation Discrimination Layer

## Scope

This module is a ROS-independent activation-inhibit layer for the Lockheed
baseline path. It does not classify a collision, generate a maneuver, invoke a
CBF, or terminate an active avoidance episode.

The per-threat processing path is:

```text
synchronized ownship/threat state
  -> timestamp and finite-state validation
  -> 3-D slant range, range rate, and closure
  -> entry/exit boundary lookup
  -> stateful OUTSIDE / FDZ / STANDBY transition
  -> per-threat FormationResult
```

The multi-threat decision is a separate, explicitly selected aggregation
policy. No global OR or AND policy is hidden inside the classifier.

## Evidence boundary

- Source-direct structure: range and closure, FDZ, Standby Region, hysteresis,
  and inhibition of a new Auto ACAS activation while classified formation.
- Implementation reconstruction: the three-state enum, piecewise-linear table
  API, fail-open handling of invalid/stale data, per-threat result schema, and
  selectable aggregation policies.
- UAV calibration: every boundary value, state-age limit, range relationship,
  and aggregation-policy selection.

No runtime profile file is shipped in this worktree. A calibrated profile must
be supplied explicitly and reviewed before this optional layer is enabled.

## Runtime integration

`ManeuverSelectionWorker` retains one `FormationDiscriminator` and converts the
already common-time-aligned selected threat intent at the evaluation timestamp
into one `FormationUpdateInput` per threat. The configured aggregation policy
is applied only to the threats for which the existing PMR/MASD/AD evaluator
reported `AD < 0`.

Only the transition from inactive to active is gated:

```cpp
if (!avoidance_status.active && collision_activation_requested &&
    aggregation.allow_new_activation) {
    // forward the existing activation request
}
```

The aggregation output must not be sent to `reset()`, deactivation, CPA
termination, or active-command replacement logic. Outside formation only means
that the existing PMR/MASD/AD logic is allowed to continue.

The ROS parameters are present but `formation_discrimination_enabled` defaults
to `false`. Enabling it with an incomplete template is rejected by worker
parameter validation; a reviewed calibrated profile must supply all scalar and
piecewise-linear table values first.

## Calibration workflow

`FormationDecisionCsvFormatter` provides the structured row required for
replay. `FormationCalibrationHarness` replays the same C++ classifier across
profiles and calculates entry/exit time, state toggles, nuisance-activation
opportunities, missed inhibit cases, and configurable boundary dwell time.

`tools/plot_formation_range_closure.py` consumes those rows and plots the
encounter with the entry/exit FDZ and Standby boundaries. The plotting tool
does not duplicate or reimplement classification logic.
