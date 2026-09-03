# Pairwise 7x7 AD Interaction Graph

Status: direct component cutover path implemented; frozen fleet rejoin input,
single-round existing peer-awareness qualification, and Formation smoke
verified

## Authority and scope

The authoritative implementation contract is
`pairwise_7x7_ad_interaction_graph_codex_spec.md`. The retained seven roll
maneuvers, trajectory prediction, PMR, MASD, AD, incumbent memory, and
peer-awareness paths are existing project capabilities.

The following graph decomposition is a **project design**. It is not presented
as a published Lockheed Martin Auto ACAS algorithm.

This change removes the abandoned ownship-local Top-3/J-gate experiment from
the interaction graph. It does not change candidate definitions, activation,
CPA termination, Formation guidance, or PX4 interfaces.

## Implemented data path

For each 4 Hz selection epoch:

1. Freeze the latest complete, canonically ordered seven-candidate trajectory
   library whose source timestamp does not exceed a common cutoff. The current
   project cutoff is
   `epoch_start + coordination_delay - 3 * trajectory_refresh_period`, which
   leaves three 20 Hz delivery periods before selection. This cutoff is a project
   synchronization rule, not a published Lockheed Martin constant.
2. Use the common evaluation time `epoch_start + coordination_delay`.
3. Evaluate and cache all 49 candidate pairs for every aircraft pair:

   \[
   AD_{ij}^{a,b}=PMR_{ij}^{a,b}-MASD_{ij}^{a,b}.
   \]

4. Store the full 7x7 AD table, its minimum, and the candidate IDs producing
   that minimum.
5. Add graph edge `(i,j)` exactly when:

   \[
   \min_{a,b} AD_{ij}^{a,b}<AD_{screen}.
   \]

6. Search each connected component using cached pair evaluations. A certified
   isolated aircraft is excluded from joint search and has no entry in the new
   avoidance proposal.
7. Assemble the component proposal and independently re-evaluate all ten
   aircraft pairs. A clear proposal slot uses the aircraft's actual execution
   state: its active latch, or its current Formation command when inactive. The
   post-check does not reuse the cached 7x7 AD entries or invoke another joint
   candidate search; it directly applies the existing pair evaluator ten times.

The external HILS scenario names are `formation` and `formation_pentagon`.
The former `flocking*` strings remain accepted only as backward-compatible
input aliases and are normalized before result metadata is written. The
`test_guidance_mode` value is likewise `formation`; the existing internal
`FlockingGuidance` implementation remains the guidance algorithm owned by
`FormationMode` and is not renamed as part of this task.

For five aircraft the certification count is fixed at:

\[
49\binom{5}{2}=490.
\]

The component search count is:

\[
\sum_{|C_k|\ge2}7^{|C_k|}.
\]

`AD_screen` is exposed in configuration and initially defaults to `0.0 m`.
No unpublished positive reserve is invented.

## Coordination identity

An active component proposal is qualified with these common identifiers:

- selection epoch;
- trajectory-library version and candidate-library hash;
- AD/MASD configuration version;
- graph configuration and graph hash;
- connected-component hash;
- assembled component-solution hash.

The component path does not require full local proposal-array/revision equality.
Receipt of matching component proposals from every participant is the existing
proposal/peer-awareness qualification. Component cutover does not add a second
all-peer acknowledgement round. The existing revision-sensitive V4 path keeps
its prior acknowledgement check. Failure to match the component identities
retains the previous best maneuver.

After a component proposal passes the global all-pair post-check, it is the
authoritative proposal. The component path does not call the legacy common-
incumbent builder or the node-local `clearlySuperior()` gate. Those operations
remain only on the selectable legacy comparison path.

## Diagnostics

The ROS diagnostic message records:

- all ten pairwise minimum AD values and argmin candidate IDs;
- edge flags, adjacency, components, and hashes;
- 35 trajectory-library entries expected for five aircraft;
- 490 pair-candidate evaluations expected for five aircraft;
- naive and component search counts;
- certification, graph, component-search, post-check, and total timing;
- shadow result, selected tuple, and global post-check result.

The full 7x7 matrices remain in the internal certification cache and are
covered by unit tests rather than copied into every ROS diagnostic message.

## Acceptance gates

1. Unit tests must verify the strict edge boundary, exact 49-per-pair count,
   canonical library rejection, deterministic hashes, `[3,2] -> 392`, chain
   connectivity, isolated handling, and cached-search equivalence with the
   legacy exhaustive evaluator.
2. The whole package test suite must pass.
3. In shadow SILS, all five nodes must agree on the candidate-library,
   certification, graph, component, and solution hashes for common epochs.
4. Component search plus certification p95 must remain below 250 ms.
5. Active cutover is allowed only after the shadow identity and timing gates
   pass. A 55 s smoke run precedes a 200 s Formation pentagon run.
6. Minimum separation and DSD duration are reported separately from compute
   performance and compared with the anchored pure-Lockheed baseline.

## Explicit non-goals

- no J-score graph or Top-3 threat participant selection;
- no reduction of the seven maneuver candidates;
- no new consensus protocol;
- no change to AD activation or CPA termination;
- no arbitrary hysteresis or hold time;
- no claim that component-wise selection equals the globally minimum cost
  among every safe 7^N tuple. The no-edge certificate establishes safety
  separability; the global post-check establishes assembled-tuple feasibility.

## Verification record

### Direct-path remediation

The static audit identified four interacting defects and unnecessary detours:

- the common component result was routed back through a node-local incumbent
  comparison;
- a locally latched safe-rejoin flag could change a component objective even
  when all nodes had the same certified candidate library;
- component cutover introduced an additional all-peer acknowledgement round
  beyond the existing proposal exchange;
- the analyzer classified an authoritative component switch as unauthorized
  unless the unrelated legacy superiority flags were set.

The remediation removes those paths instead of adding fallback layers:

- rejoin request and nominal lateral-acceleration input are carried in every
  candidate packet and included in the frozen candidate-library identity;
- the component evaluator builds one fleet rejoin objective exclusively from
  that certified library;
- component cutover maps `assembled_candidate_ids` directly into the certified
  library and does not construct or evaluate a local incumbent;
- one matching component-proposal exchange performs coordination qualification;
  the added second acknowledgement path was deleted;
- the interaction-graph evaluator no longer accepts three unused live-library
  arguments, and the misleading `missing_selected_command_count` analyzer path
  was deleted. Actual PX4 override input is the activation controller's latched
  `ownship_input`, not an exact historical intent packet lookup.

The legacy exhaustive evaluator remains only because shadow/baseline runs still
select it explicitly. It is not invoked by component cutover.

### Static and unit verification

- The first five-axis audit did not cover the end-to-end invariant from the
  assembled component candidate IDs to the published proposal IDs. Its prior
  implementation-correctness PASS is therefore withdrawn.
- The audit initially found two implementation defects: isolated aircraft were
  forced to `RollZero` instead of retaining the incumbent, and the final
  all-pair post-check reused the certification cache. Both were corrected
  before the final audit.
- A subsequent adversarial bag/code comparison found a third defect: the
  one-candidate-per-aircraft post-check necessarily returned candidate slot
  zero for every aircraft, and cutover incorrectly reused those temporary
  slots as indices into the original seven-candidate library. This collapsed
  every published proposal to candidate 0 (`RollMinus50`). Cutover now uses
  `assembled_candidate_ids` as the authoritative IDs and resolves each ID back
  into the certified library only to obtain its command, revision, and source
  timestamp.
- Full `collision_avoidance` package result: 229 tests, 0 errors, 0 failures,
  0 skipped.
- Python analyzer compilation and `git diff --check`: PASS.

### SILS verification

The final 55 s Formation-pentagon cutover smoke run established deterministic
distributed behavior:

- 64 common valid five-node epochs;
- candidate-library, certification, graph, component, component-solution, and
  assembled-tuple agreement: 100%;
- component path total compute time p95: 30.483 ms;
- minimum 3D separation: 1.675 m (safety FAIL).

The pre-fix 200 s Formation-pentagon cutover run produced:

- 284 common valid five-node epochs;
- candidate-library, certification, graph, component, component-solution, and
  assembled-tuple agreement: 100%;
- common proposals: 208, identical tuple and command: 100%;
- unauthorized active proposals: 0;
- pairwise certification p95: 27.725 ms;
- graph computation p95: 0.043 ms;
- total component path p95: 33.508 ms, below the 250 ms compute gate;
- minimum 3D separation: 0.617 m;
- estimated time below the 10 m DSD: 15.1 s;
- active candidate switches: 0.

This pre-fix active-control result is invalidated by the slot/ID mapping defect
and must not be used as graph safety-performance evidence.

After the mapping fix, a 55 s Formation-pentagon smoke run produced:

- minimum 3D separation: 10.778 m;
- estimated time below the 10 m DSD: 0.0 s;
- active-best switches: 1 to 3 per aircraft;
- switch-superiority evaluations: 106 to 129 per aircraft;
- candidate-library, certification, graph, and component hashes: 100% match;
- component-solution and assembled-tuple match: 65.15%;
- component path total compute time p95: 40.816 ms.

The mapping defect is therefore fixed: component choices no longer collapse to
candidate 0 and the active switching path executes. The smoke also exposes a
separate coordination issue: isolated members retain node-local incumbent
views, so equal library/graph inputs can still assemble different fleet tuples.
That issue was corrected by requiring the local worker and all qualified peers
to expose the same previously confirmed fleet tuple before it can seed an
isolated component. A mismatch now suppresses the new component proposal and
retains the previous best rather than assembling node-local variants.

The subsequent 55 s common-incumbent Formation-pentagon smoke produced:

- common component-solution and assembled-tuple agreement: 33/33, 100%;
- common proposal tuple and command agreement: 24/24, 100%;
- candidate-library, certification, graph, and component hash agreement: 100%;
- active-best switches: 4 to 6 per aircraft;
- component path total compute time p95: 38.641 ms;
- minimum 3D separation: 7.249 m;
- estimated time below the 10 m DSD: 0.5 s.

That run confirmed the former coordinated-incumbent workaround. The workaround
is superseded by the explicit execution-state rule below; its safety metric is
historical and no safety improvement is inferred from it.

### Explicit no-candidate semantics for isolated aircraft

The initial component implementation used `RollZero` to fill an isolated
aircraft's tuple slot before any coordinated incumbent existed. That was only
an array-initialization convention, but it was observable as if a real
avoidance candidate had been selected. It is no longer used as a selection
rule.

- `selected_candidate_valid_mask`, `proposed_candidate_valid_mask`, and
  `assembled_candidate_valid_mask` now state which tuple entries are actual
  avoidance candidates.
- Every isolated aircraft has a clear validity bit in the new component
  proposal. Its stored ID byte is ignored and cannot request a new PX4
  override.
- A connected component sets the validity bits of all evaluated members.
- For every clear validity bit, the final all-pair cross-check uses the actual
  execution state: an active avoidance latch remains active, while an inactive
  aircraft uses its timestamped current Formation input.
- Isolation no longer triggers an all-aircraft previous-tuple agreement check.
  Candidate selection and execution latching are independent state boundaries.
- Proposal and solution identity include the validity mask, so `no candidate`
  cannot compare equal to a real candidate merely because their raw ID bytes
  happen to match.

The internal nominal-intent builder still supplies a legal packet candidate-ID
value because the existing reconstruction API validates that field. The ID is
not published as a selected candidate and is not used as the command; the
packet's explicit `candidate_input` is the actual Formation setpoint used by the
post-check.

The final 55 s explicit-execution-fallback Formation-pentagon smoke produced:

- scenario metadata: `formation_pentagon`;
- minimum 3D separation: 12.832 m;
- estimated time below the 10 m DSD: 0.0 s;
- valid graph samples: 359;
- global all-pair post-check passes: 210;
- common valid five-node epochs: 65;
- candidate-library, certification, graph, component, component-solution, and
  assembled-tuple agreement: 100%;
- active selected-slot mismatch: 0;
- component path total compute time p95: 29.042 ms.

This smoke verifies the isolated execution-state separation and the Formation
fallback wiring. It is a single short run, not a statistical safety claim.

### Final direct-path Formation smokes

Before removal of the extra acknowledgement round, a 55 s Formation-pentagon
smoke confirmed that the frozen rejoin metadata fixed the prior node-local
objective split:

- 64 common valid five-node epochs;
- candidate-library, certification, graph, component, solution, and assembled
  tuple agreement: 100%;
- common proposal tuple and command agreement: 11/11, 100%;
- active-best switches: 0 because the added second acknowledgement repeatedly
  lost its race with the next selection epoch;
- component-path p95: 32.374 ms;
- minimum 3D separation: 7.381 m, with about 0.3 s below DSD.

After deleting that additional consensus round, the final 55 s
Formation-pentagon smoke produced:

- 63 common valid five-node epochs;
- candidate-library, certification, graph, component, solution, and assembled
  tuple agreement: 100%;
- common proposal tuple and command agreement: 5/5, 100%;
- active-best switches: 5 to 8 per aircraft;
- measured proposal-to-commit p95: 0.173 s;
- component-path total compute time p95: 31.709 ms;
- component proposals misclassified as unauthorized: 0;
- minimum 3D separation: 5.055 m, with about 1.7 s below DSD.

The final smoke passes the deterministic graph/coordination wiring checks but
fails the 10 m safety-performance requirement. Its separation result must not
be interpreted as evidence that component decomposition is safe. A longer
performance investigation remains separate from this Phase 1-2 implementation
audit.

### Final five-axis audit

| Axis | Result | Evidence |
| --- | --- | --- |
| Source accuracy | PASS | Lockheed-derived AD, trajectory sharing, incumbent memory, and peer awareness are explicitly separated from the project-defined graph decomposition. |
| Interpretation fidelity | PASS | No claim is made that Lockheed specifies the AD interaction graph, and component results are no longer reinterpreted by a node-local incumbent. |
| Complexity proportionality | PASS | Unused graph arguments, the component-only second acknowledgement round, and the misleading command-reference diagnostic were deleted; the legacy evaluator remains only as an explicitly selectable baseline. |
| Implementation correctness | PASS for Phase 1-2 wiring | Frozen rejoin metadata is hash-covered; common component identities and proposals agree 100%; active changes commit; unit/build tests pass. The SILS 10 m safety metric remains FAIL and is not promoted to an implementation PASS. |
| Directional alignment | PASS | The executable path is candidate certification → graph → components → assembled IDs → global post-check → existing proposal exchange → execution. |

Result artifacts:

- `result/summary/lockheed_pairwise_ad_graph_component_cutover_smoke_20260903_03/summary.json`
- `result/summary/lockheed_pairwise_ad_graph_component_cutover_200s_20260903_02/summary.json`
- `result/plot/lockheed_pairwise_ad_graph_component_cutover_200s_20260903_02/actual_maneuver_overview.png`
- `result/summary/lockheed_pairwise_ad_graph_slotfix_smoke_20260903_01/summary.json`
- `result/plot/lockheed_pairwise_ad_graph_slotfix_smoke_20260903_01/actual_maneuver_overview.png`
- `result/summary/lockheed_pairwise_ad_graph_common_incumbent_smoke_20260903_01/summary.json`
- `result/plot/lockheed_pairwise_ad_graph_common_incumbent_smoke_20260903_01/actual_maneuver_overview.png`
- `result/summary/lockheed_pairwise_ad_graph_execution_fallback_formation_smoke_20260903_02/summary.json`
- `result/plot/lockheed_pairwise_ad_graph_execution_fallback_formation_smoke_20260903_02/actual_maneuver_overview.png`
- `result/summary/lockheed_pairwise_ad_graph_common_rejoin_formation_smoke_20260903_01/summary.json`
- `result/summary/lockheed_pairwise_ad_graph_single_peer_awareness_formation_smoke_20260903_01/summary.json`
- `result/plot/lockheed_pairwise_ad_graph_single_peer_awareness_formation_smoke_20260903_01/actual_maneuver_overview.png`
