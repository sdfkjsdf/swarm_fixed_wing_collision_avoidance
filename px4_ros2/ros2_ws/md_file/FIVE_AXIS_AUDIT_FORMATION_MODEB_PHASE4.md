# Five-Axis Audit — Formation Discrimination and Mode-B Phase 4

## Audit baseline

Authoritative design inputs for this implementation step:

1. `/home/hmcl/Downloads/lockheed_formation_discrimination_layer_codex_spec_v2.md`
2. `/home/hmcl/.codex/attachments/e88670d1-e116-4be8-a1fc-14a6f0db7f1f/pasted-text.txt`
3. `/home/hmcl/Downloads/auto_acas_termination_criteria_note.md`

The latest Mode-B instruction explicitly supersedes conflicting older V4
design statements. The source/design boundary declared in those inputs is
preserved: Lockheed formation structure and Auto-ACAS management are treated
as source-derived; the scalar Mode-B combination is identified as this
project's design.

## A. Lockheed formation-discrimination layer

| Axis | Verdict | Evidence |
|---|---|---|
| 1. Source accuracy | PASS | Uses 3-D range, range rate, `Vc=-R_dot`, FDZ, Standby, and stateful entry/exit boundaries. |
| 2. No over-interpretation | PASS | Source-reference and UAV-calibrated profiles remain separate and disabled until calibrated. Invalid/stale fail-open behavior and aggregation are documented as implementation choices. |
| 3. Proportional complexity | PASS | One pure classifier, one calibration helper, one plot tool, and one runtime gate cover the explicit logging/replay/sweep/integration requirements. No CBF dependency or second collision evaluator was added. |
| 4. Implementation correctness | PASS after remediation | Initial audit found FDZ-to-Standby and Standby-to-FDZ transitions using nominal rather than relaxed boundaries, and an active episode could have had its unsafe mask filtered. Cross-region transitions now use the relaxed union, while active episodes retain the complete AD-derived mask for CPA monitoring. |
| 5. Direction alignment | PASS | Effective output gates only a new `AD<0` AMAC activation. An active avoidance episode ignores formation inhibit and remains governed by CPA termination. |

Runtime policy is explicit:

- `per_threat_exemption_only`: remove only formation-classified threats from
  the new-activation unsafe mask;
- `all_relevant_threats_formation`: globally inhibit only if every relevant
  collision threat is classified formation;
- `any_relevant_threat_formation`: global inhibit if any relevant threat is
  classified formation.

No policy is selected silently. Runtime defaults to disabled and requires a
valid calibrated configuration before enablement.

## B. Lockheed alternate termination boundary

| Axis | Verdict | Evidence |
|---|---|---|
| 1. Source accuracy | PASS | Active pairs use current position and current velocity vectors projected to future CPA. |
| 2. No over-interpretation | PASS | Near-zero relative speed and past-CPA clamping are labeled project reconstructions because the public note does not disclose exact special-case code. |
| 3. Proportional complexity | PASS | The previous Flocking 4.5 s intent packet, post-handoff AD evaluation, peer handoff consensus, and follow-state were removed. |
| 4. Implementation correctness | PASS | Each pair's original MASD is latched when it first becomes unsafe; termination requires strict `d_CPA > D_activation,original` for every affected pair. |
| 5. Direction alignment | PASS | Flocking is only the nominal controller resumed after termination. It is not an input to the termination decision. |

## C. Closed-form backup Mode-B Phase 4

| Axis | Verdict | Evidence |
|---|---|---|
| 1. Source accuracy | PASS | The implementation follows the authoritative phase boundary: existing transport, moving-threat paths, LEFT/RIGHT certification, branch-local scalar interpolation, existing candidate management, and PX4 adapter. |
| 2. No over-interpretation | PASS | The combined moving-threat TC/OI construction remains labeled project design; it is not claimed as a theorem from a single paper. |
| 3. Proportional complexity | PASS | One intent adapter bridges the existing compressed/reconstructed trajectory to the already implemented certifier. No new ROS node, thread, transport schema, or legacy interval reconstruction was introduced. |
| 4. Implementation correctness | PASS | Threat intent is aligned to the common evaluation time and propagated for 4.5 s with its transmitted candidate input. `BackupControlInterpolatorV4` is called directly. `h_dot_cmd=0` is preserved and the V4/PX4 sign conversion occurs once. Missing/future/stale inputs fail closed at cutover. |
| 5. Direction alignment | PASS | Legacy remains an explicit comparison architecture; Mode B is a separate selectable primary safety path and reuses incumbent persistence, periodic selection, proposal consensus, and peer architecture cross-check. |

## Verification evidence

- Release build: `collision_avoidance` PASS, no compiler stderr after final rebuild.
- CTest: 19/19 executables PASS.
- GTest/result roll-up: 194 tests, 0 errors, 0 failures, 0 skipped.
- Formation focused tests: 17 PASS.
- Mode-B certifier/interpolator/threat-adapter/adapter tests: 42 PASS.
- `git diff --check`: PASS.
- Python syntax checks for formation plot and updated HILS analyzer: PASS.

## Remaining evidence gap

Gazebo SILS was not executed in this step. Therefore runtime timing,
closed-loop PX4 response, calibrated formation thresholds, and encounter-level
Mode-B effectiveness remain validation tasks rather than proven performance
claims. This does not change the static five-axis PASS verdict for the scoped
implementation.
