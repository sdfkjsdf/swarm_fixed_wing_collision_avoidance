# MASD budget measurement

This is an observation-only audit for the five-aircraft Gazebo-classic Formation
pentagon fixture. It does not change activation, termination, candidate selection,
Q, or a MASD margin. It is not a general-purpose certification tool.

## Runtime observation

Build `collision_avoidance` in Release on PC and Pi because the optional diagnostic
message and trace hooks are new. The normal launcher leaves tracing disabled.
The measurement run enables:

```bash
export MASD_DIAGNOSTICS_ENABLED=true
export ROLL_RESPONSE_TIME_CONSTANT_S=0.5
```

The second override preserves the recorded `67ebf73` baseline. There were already
uncommitted 0.415-s roll-model edits before this task; those edits are preserved,
not used to change this comparison. Use the actual recorded parameter for any
other run. Both local and remote guidance launchers forward these variables.

`/common/px4_N/maneuver_budget_trace` has four event types:

| Event | Location | Meaning |
|---|---|---|
| 1 | `ManeuverSelectionWorkerTrajectory.cpp` | A valid proposal has finished computation |
| 2 | `ManeuverSelectionWorkerCoordination.cpp` | The coordinated tuple has committed |
| 3 | `ManeuverSelectionWorkerActivation.cpp` | An existing pair-monitor calculation has completed |
| 4 | `FormationMode.cpp` | Actual ROS fixed-wing setpoint publication is bracketed |

The worker uses a separate optional bounded SPSC trace queue. It does not enlarge
the control-output queue, block on the recorder, recompute AD, or consume trace
messages as control inputs. `dropped_trace_count` records worker-queue overflow;
it does not prove that DDS lost no trace messages. Publication is best effort.
Instrumentation and recording have nonzero scheduling/network overhead, so two
independent closed-loop runs are not a deterministic causal A/B experiment.

Use `steady_ns` only on the same machine. For cross-machine timestamps, take
read-only clock probes before and after the experiment:

```bash
python3 analysis/measure_clock_offset.py agent0@192.168.50.73 \
  --output /absolute/existing/report/directory/clock_before.json
```

The command does not adjust either clock. The interval bounds include network
asymmetry at the probe; interpolation between probes is an estimate, not a clock
accuracy guarantee. A shared ROS domain or NTP-enabled flag alone is insufficient.

The hybrid runner is `scripts/run_hybrid_1rpi_4local_case.sh avoidance`. For this
measurement it used `RUN_DURATION_SECONDS=200`, `GENERATE_VIDEO=false`, the normal
Formation pentagon setup, and a separate Pi image
`collision-avoidance:masd-measurement`. The original `:distributed` image and
`collision-avoidance-dev` container were not modified. All results stay on PC.

## Offline reconstruction

Use an isolated Python environment with ROS Humble `rosbag2_py`, `numpy`,
`matplotlib`, and `pyulog`. A C++ compiler and Eigen are required. Source both
ROS and the workspace containing the new message before running the analyzer.

```bash
source /opt/ros/humble/setup.bash
source /home/hmcl/workspace/swarm-fixed-wing/ros2_ws/install/setup.bash

python3 analysis/analyze_masd_budget.py \
  --bag /absolute/path/to/bag \
  --output /absolute/path/to/report_directory \
  --start-us COMMON_FIXED_WING_START_EPOCH_US \
  --tau-phi 0.5 \
  --clock-probe /absolute/path/clock_before.json /absolute/path/clock_after.json \
  --ulog /absolute/path/node0.ulg /absolute/path/node1.ulg \
         /absolute/path/node2.ulg /absolute/path/node3.ulg /absolute/path/node4.ulg
```

`actual_evaluation_start_ns` in the normal HILS summary supplies the common
fixed-wing evaluation start. Do not include takeoff/transition in coverage.
`--reuse-analysis` refreshes latency and plots from an existing report; it must
not be used after changing coverage definitions or clock/truth reconstruction.

`masd_native.cpp` compiles the production trajectory receiver, uncertainty
propagator and cubic reconstruction into an offline shared library. The analyzer
uses the recorded packet's state, P0, command, revision and compressed mean.
Replayed global AD is cross-checked against the recorded graph result.

This fixture fixes DSD=10 m, two half-wingspans totaling 2.144 m, Bcomm=0,
chi-square(3)=7.814727903251179, 46 points at 0.1 s, roll-rate limit=70 deg/s,
and the production default Q. A different configuration must be made explicit;
do not silently reuse these constants for another airframe or controller.

## Time and ground-truth boundary

- Production received position **and covariance** use source age plus horizon.
- A DDS timestamp is HRT minus **filtered estimated** timesync offset.
- ULog physical event times here use HRT minus **raw observed** timesync offset.
  The raw estimate still has RTT/sampling limitations. Filter lag is reported
  separately; it must not be mislabeled transport latency.
- Simulator ground truth comes from `vehicle_global_position_groundtruth`, not
  EKF `trans_vehicle_odometry`. Geographic coordinates are inverted using this
  Gazebo-classic fixture's projection: radius 6,353,000 m; home latitude
  47.397742 deg, longitude 8.545594 deg, altitude 488 m. Those values follow
  `sitl_gazebo-classic/include/common.h` and `launch_5vtol.sh`; using WGS84 ECEF
  here would silently introduce a different map scale.
- Ground-truth interpolation rejects gaps over 150 ms. A 10-ms separation grid
  is an interpolated estimate, not additional independent simulator samples.

## Interpreting the outputs

- `coverage_samples.csv`, `coverage_groups.csv`: one-sided separation-error
  coverage by horizon, maneuver, high bank, state-opposed roll, and PMR instant.
- `operational`: proposed/assembled trajectories versus what actually happened;
  unexecuted hypotheses are **not** Q-calibration samples.
- `lateral_command_matched`: recorded PX4 lateral input matches from source to
  target; old bags cannot additionally verify the ground-speed command.
- `speed_and_lateral_matched`: also requires active avoidance and matching
  published ground-speed/lateral commands. This is still not certification of
  longitudinal/EAS/altitude tracking. Small long-horizon groups are insufficient.
- `ad_loss_samples.csv`: uniform 20-Hz/4-Hz offline replay, with same-input and
  changed-input populations separated; not an exact reconstruction of each
  node's asynchronous receive cache.
- `runtime_pair_ad_loss.csv`: actual monitor calls, pair identity, input revision,
  active mode and PMR/MASD decomposition; cadence is measured, not assumed.
- `latency_samples.csv`: individual stages. Active-at-commit and inactive-at-commit
  are separated. Negative results remain visible, never clamped to zero.
- `roll_model_additional_delay_fit`: exploratory fixed-tau fit on a finite search
  range, **not** identified communication delay. Boundary hits make a claimed
  calibrated delay invalid.
- `masd_budget_summary.json`: machine-readable report, sample counts, clock
  diagnostics, ground truth, coverage and latency distributions.

Coverage is pointwise and conditional on the tested encounter. It does not prove
95% simultaneous whole-cone containment, fleet safety, or independent Monte Carlo
coverage. Epoch-bootstrap intervals are descriptive within one correlated run.

The dated findings and five-axis audit are in
`../docs/MASD_BUDGET_AUDIT_20260905.md`.
