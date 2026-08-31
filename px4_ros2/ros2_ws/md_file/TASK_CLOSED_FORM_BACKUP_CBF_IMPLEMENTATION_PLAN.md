# Closed-Form Backup CBF V4 - Phased Implementation Plan

## 1. Authority and supersession

This plan implements the user-supplied design direction in:

- `/home/hmcl/.codex/attachments/e88670d1-e116-4be8-a1fc-14a6f0db7f1f/pasted-text.txt`

That direction supersedes the earlier instantaneous turning-circle interval as
the intended V4 safety architecture. The existing `SafeControlSetV4` and its
runtime wiring remain intact while the replacement backup architecture is
developed and verified. They are not silently relabelled as the new design.

The evidence baseline is:

- OI-bCBF paper:
  `/home/hmcl/workspace/reference/paper/control_barrier_function/Safety-Critical Control with Bounded Inputs.pdf`
- OI-bCBF reference code:
  `/home/hmcl/workspace/reference/code/OI-CBF/aircraft/control.py`
  `/home/hmcl/workspace/reference/code/OI-CBF/aircraft/dynamics.py`
  `/home/hmcl/workspace/reference/code/OI-CBF/aircraft/safety.py`
- TC-CBF paper: Lee, Park, and Kim, *Turning Circle-Based Control Barrier
  Function for Efficient Collision Avoidance of Nonholonomic Vehicles*,
  DOI `10.1109/TCST.2026.3692226`, especially Eqs. (7), (12)-(16).
- Existing project data/sign decisions:
  `/home/hmcl/workspace/reference/paper/MD_FILES/codex_v4_implementation_decisions_v2.md`

## 2. Source/design boundary

Source-derived elements retained:

- predefined backup controller and its forward flow;
- path-safety and terminal-condition structure of a backup CBF;
- nominal and backup directional sensitivities propagated along the same
  backup flow;
- scalar interpolation constraints `a_i + b_i * mu >= 0`;
- left/right turning-circle center and radius geometry.

Project-specific design elements, which must not be claimed as a published
theorem, are:

- scalar fixed-wing state `[p_N, p_E, psi, V]` with heading rate as the only V4
  decision channel;
- independent fixed-direction LEFT and RIGHT backup policies;
- physical multi-aircraft separation along the path;
- use of TC-CBF geometry only as a moving-threat terminal turn certificate;
- explicit scalar feasible-interval intersection instead of assuming that the
  original OI-bCBF feasibility theorem transfers to the new terminal condition;
- receding re-certification that may change direction between control cycles.

No forward-invariance claim is made for the moving-threat terminal turn
certificate.

## 3. Preserved invariants

- Internal V4 sign: positive heading rate is LEFT; negative is RIGHT.
- PX4 sign conversion remains confined to the existing adapter layer.
- Effective turn rate is recomputed from current propagated airspeed:
  `min(FW_Y_RMAX, g * tan(FW_R_LIM) / V)`.
- Actual/estimated airspeed is primary; trim speed is a runtime fallback only.
- Threat data are fail-closed when time alignment or trajectory coverage is
  invalid.
- Fixed-size, allocation-free evaluation is retained in the pure cores.
- The existing ROS node, worker thread, intent transport, predictor,
  uncertainty propagation, PMR/MASD/AD evaluator, and PX4 command path are not
  duplicated in Phases 1 or 2.

## 4. Phase 1 - fixed-direction backup certification

### Scope

Add a ROS-independent backup model and certifier with these layers:

1. scalar dynamics and effective turn-rate calculation;
2. predefined LEFT and RIGHT backup controls;
3. separate fixed-direction backup rollouts;
4. per-threat path separation over every sampled time;
5. per-threat terminal turning-circle margin at the same terminal timestamp;
6. structured LEFT and RIGHT branch results and four-way classification.

The core consumes already time-aligned sampled threat trajectories. Constant
velocity generation and reconstruction from production intent packets remain
adapter/test responsibilities and are deferred to Phase 4.

### Required behavior

For a branch `d`:

```text
r_b,d(tau) = sigma_d * r_max_eff(V(tau))
sigma_LEFT = +1
sigma_RIGHT = -1
```

The sign may never change inside one rollout. Speed and therefore turn-rate
magnitude may evolve.

Path margin:

```text
h_s,j(tau) = distance(p_own(tau), p_j(tau))
             - physical_clearance_j
             - reference_margin
```

Terminal margin:

```text
rho = V_T / r_max_eff(V_T)
c_d = p_T + sigma_d * rho * n_perp(psi_T)
h_TC,j,d = distance(c_d, p_j(T_B))
           - (physical_clearance_j + rho)
```

A branch is certified only when every path and terminal margin meets the
configured numerical tolerance. The low-level certifier does not choose a
maneuver.

### Files

- `include/collision_avoidance/selection/BackupControlModelV4.hpp`
- `src/selection/BackupControlModelV4.cpp`
- `include/collision_avoidance/selection/BackupSafetyCertifierV4.hpp`
- `src/selection/BackupSafetyCertifierV4.cpp`
- `test/test_backup_safety_certifier_v4.cpp`
- `CMakeLists.txt`

### Phase-1 acceptance gate

- no threat;
- same-speed parallel traffic;
- head-on traffic;
- left and right crossing traffic;
- LEFT-only, RIGHT-only, BOTH, and NEITHER classifications;
- multiple threats with conflicting preferred directions;
- changing speed changes effective turn rate;
- every LEFT rollout rate is nonnegative;
- every RIGHT rollout rate is nonpositive;
- separate evaluation cycles may change the newly certified direction;
- invalid/stale/future/misaligned threat trajectories fail closed;
- focused tests, full package build, and full package tests pass;
- all five audit axes pass for the bounded Phase-1 claim.

Phase 2 is forbidden until this gate passes.

## 5. Phase 2 - directional sensitivity and scalar interpolation

### Scope

Reuse the OI-bCBF architecture without copying its old terminal theorem:

- initialize `q_nom(0) = f(x) + g(x) * r_nom`;
- initialize `q_backup(0) = f(x) + g(x) * r_backup`;
- propagate both under the Jacobian of the same direction-specific backup
  flow used in Phase 1;
- form path and terminal affine constraints;
- explicitly intersect every scalar constraint with `[0, 1]`;
- choose the smallest feasible `mu` independently for LEFT and RIGHT;
- return `r_safe = (1-mu) * r_nom + mu * r_backup`.

LEFT and RIGHT feasible intervals and outputs remain separate. An unsafe gap
must not be merged into one continuous control interval.

### Files

- `include/collision_avoidance/selection/BackupControlInterpolatorV4.hpp`
- `src/selection/BackupControlInterpolatorV4.cpp`
- `test/test_backup_control_interpolator_v4.cpp`
- Phase-1 model files only where a shared directional-propagation primitive is
  required.
- `CMakeLists.txt`

### Phase-2 acceptance gate

- certified branches alone are interpolated;
- nominal-safe cases return `mu = 0` when all constraints permit it;
- increasing danger increases required intervention in a controlled fixture;
- positive, negative, and degenerate `b_i` constraints are handled correctly;
- empty scalar intervals are explicit infeasible results;
- `mu` remains in `[0,1]` and the safe rate remains between nominal and backup;
- LEFT and RIGHT are computed independently;
- focused tests, full build, full package tests, five-axis audit, and an
  adversarial direction-switching audit pass.

## 6. Deferred work and non-goals

The following are explicitly deferred until Phases 1 and 2 both pass:

- M1/M2/M3 candidate adaptation;
- replacement of the currently integrated instantaneous V4 path;
- active-encounter persistence and clearly-superior re-selection;
- peer-awareness and coordination changes;
- PX4 command cutover;
- HILS safety or performance claims;
- uncertainty-cone inflation inside the new certificate;
- formal moving-threat forward-invariance claims;
- MPC or within-rollout LEFT/RIGHT switching.

## 7. Initial five-axis audit of this plan

| Axis | Status | Evidence and reason |
|---|---|---|
| 1. Source accuracy | PASS | OI-bCBF Eqs. (7), (17)-(19), and (28), plus the reference `integrateStateBackupDirectional()`/`OI_QP()`, support backup-flow sensitivity and scalar interpolation. TC-CBF Eqs. (13)-(15) support separate turning-circle geometry. Project-specific composition is explicitly labelled. |
| 2. Interpretation fidelity | PASS | The plan does not call the new moving-threat terminal condition invariant, does not transfer the original feasibility theorem, and does not call the architecture MPC-TCCBF. |
| 3. Complexity proportionality | PASS | Two pure phased cores and focused tests map directly to the requested layers. Existing ROS, transport, predictor, evaluator, and command components are preserved and not duplicated. |
| 4. Correct implementation under aligned direction | PASS for plan | Equations, signs, timestamps, limits, branch classification, and phase gates are explicit and independently testable. Runtime implementation remains unevaluated until written. |
| 5. Directional alignment | PASS | One rollout has one fixed direction; direction changes are permitted only across newly certified control cycles and later coordination. Phase 2 follows Phase 1 and does not introduce Auto-ACAS early. |

Overall plan verdict: PASS. Implementation evidence must be re-audited after
each phase; this plan verdict is not a safety or runtime-performance claim.

## 8. Phase-1 implementation result - 2026-08-31

Phase 1 is complete. Phase 2 has not started.

Implemented artifacts:

- `BackupControlModelV4` computes the effective heading-rate limit and
  propagates independent LEFT and RIGHT scalar fixed-wing backup flows with
  RK4 integration.
- Every stored LEFT rate is nonnegative and every stored RIGHT rate is
  nonpositive. The magnitude is recomputed from propagated airspeed.
- `BackupSafetyCertifierV4` consumes an already aligned, fixed-size sampled
  threat trajectory for each threat.
- Path margins are evaluated at every aligned sample for every threat.
- Terminal turning-circle margins are evaluated for every threat at the same
  terminal timestamp.
- The result retains both branch trajectories, both margins, limiting threat
  and time diagnostics, explicit failure reasons, and the four-way branch
  classification. It does not choose a maneuver.
- The legacy `SafeControlSetV4`, candidate adapter, worker, ROS messages, and
  PX4 command path were not changed.

Verification evidence:

- Release package build: PASS.
- Focused Phase-1 suite: 10/10 tests PASS.
- Full `collision_avoidance` suite: 159 tests, 0 errors, 0 failures, 0 skipped.
- `git diff --check`: PASS.
- New sources under `-Wall -Wextra -Wpedantic -Werror -fsyntax-only`: PASS.

The focused suite covers no-threat, parallel, head-on, left/right crossing,
LEFT-only, RIGHT-only, BOTH, NEITHER, conflicting multi-threat directions,
speed-dependent turn authority, fixed direction inside one rollout,
direction change across fresh certification cycles, stale/future timestamps,
misaligned sampled trajectories, invalid configuration, and invalid state.

### Phase-1 adversarial findings

One defect was found and corrected before acceptance: the first terminal
geometry guard compared heading rate against a speed tolerance. A dedicated
`heading_rate_tolerance_radps` was added, and the horizon point-capacity
calculation was also rewritten so that a time tolerance is never subtracted
from a dimensionless ratio.

Adversarial direction checks pass:

- no code path changes `BackupDirectionV4` during a rollout;
- the LEFT and RIGHT calls have separate state and result storage;
- one threat cannot select LEFT while another selects RIGHT inside one branch;
- conflicting threats yield `NEITHER_CERTIFIED` rather than a mixed branch;
- a later evaluation may return the opposite certified branch without
  modifying either earlier rollout.

### Phase-1 five-axis source audit

Audit scope: the authoritative Mode-B directive, the Phase-1 plan, the two new
headers/sources, their CMake targets, and the focused tests. Phase 2,
Auto-ACAS integration, PX4 cutover, uncertainty, HILS performance, and formal
moving-threat invariance are excluded.

| Axis | Status | Source evidence | Implementation evidence | Reason / impact | Confidence |
|---|---|---|---|---|---|
| 1. Source accuracy and traceability | PASS | The authoritative directive requires predefined fixed-direction backup policies, backup flows, physical path separation, and direction-specific terminal turning-circle geometry. | The two new cores implement exactly those Phase-1 elements and label the terminal quantity as a certificate/margin. | No old instantaneous interval or MPC claim is presented as Mode B. | High |
| 2. Interpretation fidelity | PASS | Moving-threat terminal geometry is explicitly a project certificate, not a transferred invariant-set proof. | Public names use `terminal_turn_margin`; documentation defers all invariance and HILS claims. | The implementation preserves the stated limitation. | High |
| 3. Complexity proportionality | PASS | Phase 1 requires a model layer and a certification layer before interpolation or coordination. | Two ROS-independent fixed-size libraries and one focused test target were added; no node, thread, packet, optimizer, or runtime path was added. Existing ownship/type contracts are reused. | Every retained component maps directly to a Phase-1 requirement. | High |
| 4. Correct implementation under aligned direction | PASS | Required equations specify scalar dynamics, state-dependent turn limit, all-time/all-threat path margins, and same-time terminal margins. | RK4 dynamics, NED sign conversion, units, timestamp/grid checks, branch classification, limiting diagnostics, compiler warnings, focused tests, and full regression tests pass. | The requested realization and boundary cases have direct evidence. | High |
| 5. Directional alignment | PASS | One certificate has one fixed direction; re-certified switching is allowed only between control cycles; the low-level certifier must not select the final maneuver. | Direction is immutable per `propagate()` call, branches are evaluated independently, and the result returns both without selection. Cross-cycle reversal and conflicting-threat tests pass. | The governing architecture matches Mode B and does not reintroduce within-rollout switching. | High |

Cross-axis dependencies: none remain for the bounded Phase-1 claim.

Overall Phase-1 verdict: PASS. This is a software conformance result, not a
formal safety proof or HILS performance result. Phase 2 is the next permitted
implementation phase, but remains deferred pending an explicit next action.
