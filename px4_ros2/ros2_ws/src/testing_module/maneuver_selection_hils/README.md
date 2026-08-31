# Five-aircraft maneuver-selection HILS

This test asset intentionally drives five fixed-wing vehicles toward one common
NED point. It is not a formation controller. Its purpose is to compare the
actual minimum pair separation with and without the distributed maneuver
selection override.

The production default remains `test_guidance_mode=flocking`. This scenario
selects `point_convergence` only through its test YAML.

## One case

```bash
# Apply the selected maneuver tuple.
scripts/run_point_convergence_case.sh avoidance

# Keep the selector in shadow mode for a baseline bag.
scripts/run_point_convergence_case.sh baseline

# Test-only: evaluate all 7^5 = 16,807 roll combinations.
MANEUVER_SEARCH_MODE=exhaustive scripts/run_point_convergence_case.sh avoidance
```

Run the two execution policies as separate SILS cases with the same scenario
configuration. The optional first argument is the positive AMAC AD activation
threshold in metres. The comparison default is the agreed 10 m experimental
reserve, matching the numerical DSD value while remaining separate from MASD.
The production AMAC default remains the disclosed `AD < 0` baseline.

```bash
scripts/run_execution_policy_comparison.sh       # Delta_act = 10 m
scripts/run_execution_policy_comparison.sh 5.0   # explicit override
```

The generated run IDs include the explicit threshold, for example
`_amac_ad_5p0m`, and end in `_continuous_v4` for the V4 case.
The first case uses legacy AMAC selection with `AD < threshold`; the second
uses coordinated V4 candidates continuously after coordination, without the
AMAC AD activation gate. An empty V4 safe set remains an explicit infeasible
diagnostic and is not reported as a safety guarantee.

The runner starts five headless PX4 SITL instances, one DDS agent per instance,
the shared-coordinate transformer, and one guidance node per aircraft. It
records first and performs all analysis after the bag has been finalized.
The run is rejected unless all five vehicles enter the mode lifecycle that
executes `point_convergence` after fixed-wing transition and stabilization.

The evaluation epoch is the latest of those five activation timestamps. All
odometry, decisions, plots, metrics, and video before that common
point-convergence epoch are excluded. The `FormationMode` class name is only
the existing execution container; this HILS explicitly verifies that its
configured guidance is `point_convergence`, not flocking.

Generated artifacts are grouped by run ID:

- `result/rosbag/<run_id>`
- `result/summary/<run_id>/summary.json`
- `result/summary/<run_id>/separation_history.csv`
- `result/plot/<run_id>/actual_maneuver_overview.png`
- `result/video/<run_id>/actual_maneuver.mp4`

To rebuild the plot and video without rerunning Gazebo:

```bash
scripts/process_point_convergence_bag.sh <run_id>
```

The heuristic candidate selector is deliberately replaceable. It retains the
current best candidate and ranks two alternates by worst pairwise AD, using a
deterministic right-turn tie break before useful remote information exists.
The actual joint evaluator still evaluates all `3^5 = 243` combinations and all
ten aircraft pairs per combination.

`MANEUVER_SEARCH_MODE=exhaustive` is a diagnostic HILS mode, not the production
datalink policy. It publishes all seven lateral candidates per aircraft and
evaluates all `7^5 = 16,807` combinations. Pair trajectory calculations are
cached as ten aircraft pairs times `7 x 7 = 490` unique pair evaluations before
the joint combinations are reduced to the best result.

Selection and execution are separate. Selection proposals are refreshed at
4 Hz, while the existing 20 Hz trajectory refresh monitors ownship-versus-
threat AD. Under `amac_ad_threshold`, once activated the ownship command
remains latched until the separating-rate criterion or the 4.5 s timeout ends
that conflict episode. Under `continuous_v4`, AMAC activation and latching stay
disabled; every newly coordinated V4 tuple is eligible for execution. V4
cutover uses a two-phase bootstrap: every aircraft first advertises that it can
generate a non-empty V4 candidate set while legacy intents remain available,
then V4 intents enter the existing distributed proposal/confirmation path.
Local readiness alone never enables command execution.
