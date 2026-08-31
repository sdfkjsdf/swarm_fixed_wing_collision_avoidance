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

Phase 1 is complete. This subsection records the Phase-1 acceptance gate;
the later Phase-2 result is recorded in Section 9.

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
implementation phase.

## 9. Phase-2 implementation result - 2026-08-31

Phase 2 is complete as a ROS-independent computation layer. It is not wired
to candidate management, a ROS node, or PX4 command output.

Implemented artifacts:

- `BackupControlInterpolatorV4` first runs the Phase-1 certifier and only
  evaluates branches that Phase 1 certified.
- For each certified branch, `q_nom(0)=f(x)+g(x)r_nom` and
  `q_backup(0)=f(x)+g(x)r_backup` are propagated under the Jacobian of that
  same fixed-direction backup flow.
- Path-separation and terminal-turn directional rates form independent scalar
  constraints `a_i+b_i*mu>=0` for LEFT and RIGHT.
- Every constraint is explicitly intersected with `[0,1]`; positive,
  negative, and degenerate `b_i` cases have explicit handling. The code does
  not assume that `mu=1` is feasible.
- Each feasible branch returns the minimum feasible `mu` and
  `r_safe=(1-mu)r_nom+mu*r_backup`, along with bound and infeasibility
  diagnostics.
- The state-dependent turn-rate and turn-radius speed derivatives were added
  to `BackupControlModelV4` so longitudinal drift is represented in both the
  sensitivity flow and terminal gradient.
- Existing legacy `SafeControlSetV4`, candidate generation, Auto-ACAS worker,
  ROS communication, and PX4 command paths remain unchanged.

The position-separation function is not directly affected by heading-rate at
the initial instant. Accordingly, Phase 1 still checks the `t=0` physical
margin, while Phase-2 directional constraints begin at the first propagated
sample. This follows the reference OI implementation's relative-degree guard
rather than making an uncontrollable `t=0` first-order constraint.

Verification evidence:

- Release package build: PASS.
- Focused Phase-2 suite: 11/11 tests PASS.
- Full `collision_avoidance` suite: 171 tests, 0 errors, 0 failures, 0 skipped.
- `git diff --check`: PASS.
- Modified/new sources under
  `-Wall -Wextra -Wpedantic -Werror -fsyntax-only`: PASS.

The focused suite covers nominal-safe `mu=0`, positive and negative scalar
bounds, degenerate constraints, empty intersections, certified-only
interpolation, increasing geometric danger, independent LEFT/RIGHT results,
safe-rate bounds, finite-difference validation of directional rates,
bank/yaw-limited speed derivatives, conflicting threats, invalid nominal
commands, and invalid configuration.

### Phase-2 adversarial findings

One defect was found and corrected before acceptance: the initial draft
constructed a path directional constraint at `t=0`. Since heading-rate cannot
change position separation instantaneously, its coefficient was degenerate
and it could reject a recoverable certified branch independently of the
chosen command. The Phase-2 path-constraint loop now starts at the first
propagated point; Phase-1 `t=0` physical certification was not weakened.

Direction-separation checks pass:

- LEFT and RIGHT reuse two separate Phase-1 trajectories and two separate
  sensitivity arrays;
- an uncertified branch is marked `NotCertified` and is never interpolated;
- scalar intervals and minimum-intervention outputs are never merged across
  directions;
- the low-level result does not select an incumbent or emit a command;
- no stateful direction latch or within-rollout LEFT/RIGHT transition exists.

### Phase-2 five-axis source audit

Audit scope: the authoritative Mode-B directive Sections 13-18, the reference
OI directional implementation, the Phase-2 plan, the new interpolator
header/source, the two derivative additions to the Phase-1 model, CMake
targets, and focused tests. Phase 3 candidate management, runtime integration,
PX4 cutover, uncertainty-cone inflation, HILS performance, and a formal
moving-threat invariance proof are excluded.

| Axis | Status | Source evidence | Implementation evidence | Reason / impact | Confidence |
|---|---|---|---|---|---|
| 1. Source accuracy and traceability | PASS | The authoritative directive requires independent LEFT/RIGHT interpolation, `q_nom`/`q_backup` propagation under the same backup flow, affine scalar constraints, explicit `[0,1]` intersection, and minimum feasible `mu`. The reference OI code initializes and propagates the two directional vectors and skips its initial relative-degree point. | The interpolator implements those operations directly and uses the already accepted Phase-1 path and terminal definitions. | Every material Phase-2 claim has a directly inspectable requirement or reference-code counterpart. | High |
| 2. Interpretation fidelity | PASS | The directive explicitly forbids transferring the original feasibility theorem and separates candidate management from interpolation. | Empty intervals are reported as infeasible, `mu=1` is not presumed safe, and no candidate selection, persistence, coordination, or safety-proof claim was added. The gains remain configurable project parameters. | Source limitations and project-specific composition remain explicit. | High |
| 3. Complexity proportionality | PASS | Phase 2 requires sensitivity propagation, constraint construction, scalar intersection, branch diagnostics, and tests, but not a node, solver, candidate manager, or PX4 adapter. | One fixed-capacity ROS-independent library and one focused test target were added. It reuses Phase 1 and the existing state types; it allocates no optimizer, node, thread, transport, or duplicate predictor. | Retained structures map to required computation or fail-closed diagnostics; deferred layers were not pulled forward. | High |
| 4. Correct implementation under aligned direction | PASS | Required equations define the two initial directional vectors, same-flow Jacobian propagation, `a_i`/`b_i`, explicit scalar bounds, and the minimum-intervention blend. | NED/V4 signs, speed-dependent Jacobian and terminal-radius derivatives, all-threat indexing, degenerate bounds, residual checks, and output bounds are covered. Directional rates match central finite differences; focused and full regression suites pass. | The aligned architecture is realized correctly for the bounded Phase-2 software claim. | High |
| 5. Directional alignment | PASS | The directive requires fixed direction inside one rollout, independent LEFT/RIGHT computation, no interval merge, and no Phase-3 selection in the Phase-2 layer. | Each branch consumes one immutable Phase-1 direction, retains its own interval/result, and returns both without choosing or applying either. Conflicting and uncertified-branch tests pass. | The governing structure remains Mode B and does not reintroduce MPC or within-rollout switching. | High |

Cross-axis dependencies: none remain for the bounded Phase-2 claim.

Overall Phase-2 verdict: PASS. This is a static/software-conformance result,
not a formal safety proof or HILS performance result. Phase 3 candidate
management and all runtime/PX4 integration remain deferred.
