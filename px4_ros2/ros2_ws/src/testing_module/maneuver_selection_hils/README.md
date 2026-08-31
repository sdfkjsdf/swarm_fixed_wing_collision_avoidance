# Five-aircraft maneuver-selection HILS

This test asset drives five fixed-wing vehicles through intersecting paths. It
is not a formation controller. The original pattern uses one common NED point;
the `opposite_edge_crossing` pattern assigns each vehicle a different
destination on the opposite edge of the initial pentagon. Both compare actual
minimum pair separation with and without the distributed maneuver-selection
override.

The production default remains `test_guidance_mode=flocking`. This scenario
selects `point_convergence` only through its test YAML.

## One case

```bash
# Apply the selected maneuver tuple.
scripts/run_point_convergence_case.sh avoidance

# Keep the selector in shadow mode for a baseline bag.
scripts/run_point_convergence_case.sh baseline

# Cross the center and continue to five opposite-edge destinations.
scripts/run_opposite_edge_crossing_case.sh avoidance

# Use the production FlockingGuidance with the formation HILS spawn/config.
scripts/run_flocking_case.sh avoidance

# Reuse the 250 m-radius pentagon spawn and run FlockingGuidance only.
FLOCKING_LAYOUT=pentagon scripts/run_flocking_case.sh baseline

# The HILS default evaluates all 7^5 = 16,807 roll combinations.
scripts/run_point_convergence_case.sh avoidance

# Explicit comparison only: restore Current Best + two alternates.
MANEUVER_SEARCH_MODE=heuristic scripts/run_point_convergence_case.sh avoidance
```

The opposite-edge mapping is cyclic: vehicle 0 targets the midpoint of the
initial positions of vehicles 2 and 3, vehicle 1 targets 3 and 4, and so on.
The resulting common-NED targets are stored in each run's summary and rendered
as five color-matched stars in the PNG and MP4.

The flocking case reuses `testing_module/formation_hils/config/spawn_config.yaml`,
production `collision_avoidance/config/flocking_params.yaml`, and
`test_guidance_mode=flocking`. It does not add a test-only flocking controller.
The first smoke case starts the five vehicles in a 35 m-spaced line with a
common northbound course and checks whether the normal 30 m flocking spacing
causes any unnecessary V4 override.
`FLOCKING_LAYOUT=pentagon` keeps that same production controller and northbound
initial course but reuses the existing 250 m-radius pentagon spawn and common-
NED transform. In `baseline` mode, collision avoidance remains shadow-only so
the measured separation is attributable to flocking guidance alone.

Run the two execution policies as separate SILS cases with the same scenario
configuration. The AMAC case uses the strict `AD < 0` activation boundary;
the 10 m DSD remains inside MASD and is not added again as an AD threshold.

```bash
scripts/run_execution_policy_comparison.sh
```

The generated run IDs end in `_amac_ad_0m` and `_horizon_gated_v4`.
The first case uses legacy AMAC selection with `AD < 0`; the second
opens the coordinated V4 command gate only when the minimum 95% robust
near-nominal cone clearance over the aligned 4.5 s horizon reaches the 10 m
DSD threshold. Candidate combinations must first pass the same-time robust
positive-margin cone filter; AD cost ranks only the remaining combinations.
An empty V4 safe set remains an explicit infeasible diagnostic and is not
reported as a safety guarantee.

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

HILS validation publishes all seven lateral candidates per aircraft and
evaluates all `7^5 = 16,807` combinations by default. Pair trajectory
calculations are cached as ten aircraft pairs times `7 x 7 = 490` unique pair
evaluations before the joint combinations are reduced to the best result.

`MANEUVER_SEARCH_MODE=heuristic` is retained only for an explicit comparison.
It keeps the current best candidate and ranks two alternates by worst pairwise
AD, then evaluates `3^5 = 243` joint combinations. It is not used implicitly by
the HILS runner.

Selection and execution are separate. Selection proposals are refreshed at
4 Hz, while the existing 20 Hz trajectory refresh monitors ownship-versus-
threat AD. Under `amac_ad_threshold`, once activated the ownship command
does not end on CPA clearance alone. Each node also predicts the actual
Flocking command for 4.5 s, evaluates all ten post-handoff pairs, and requires
`AD_post >= 0`. The five nodes exchange readiness and confirmation before the
coordinated return to Flocking. There is no elapsed-time termination or
reactivation lockout. After return, activation supervision follows the actual
Flocking tuple that was verified; a negative or unavailable post-handoff AD
releases that state immediately and permits avoidance activation in the same
update. Under `continuous_v4`, AMAC activation and latching stay
disabled; every newly coordinated V4 tuple is eligible for execution.
Under `horizon_gated_v4`, AMAC activation remains separate and V4 execution is
controlled by the 4.5 s robust-cone horizon gate described above. V4
cutover uses a two-phase bootstrap: every aircraft first advertises that it can
generate a non-empty V4 candidate set while legacy intents remain available,
then V4 intents enter the existing distributed proposal/confirmation path.
Local readiness alone never enables command execution.

Active AMAC best replacement uses the HILS-only project policy in
`config/amac_dynamic_best.yaml`. The initial calibration values are
`Delta_cost_switch = 0.01` and `Delta_AD_switch = 1.0 m`. They are provisional
project values because the public Lockheed sources do not disclose numerical
`clearly superior` margins. The old command continues through proposal
agreement and an all-peer readiness exchange; the new command is applied as
one `ACTIVE(M1) -> ACTIVE(M2)` handoff without a Flocking sample or activation
timestamp reset.

```bash
scripts/run_flocking_case.sh avoidance
```

Use `AMAC_POLICY_CONFIG=/absolute/path/to/another.yaml` to run a separately
recorded calibration. The public sources do not disclose those two numerical
margins, so every override remains a project parameter rather than a Lockheed
Martin value.
