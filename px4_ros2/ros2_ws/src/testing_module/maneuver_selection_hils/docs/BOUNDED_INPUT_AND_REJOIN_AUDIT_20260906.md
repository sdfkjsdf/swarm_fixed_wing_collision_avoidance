# Candidate metadata and bounded queue processing

## Scope and authoritative evidence

Base: `b1e15d4`. The latest user instruction authorizes direct, minimal fixes
to (1) inconsistent rejoin metadata suppressing all avoidance candidates and
(2) unbounded input/callback draining and unnecessary reconstruction/logging.
It requires static five-axis review rather than SILS tuning. The user's retained
constraints are seven candidates, unchanged AD/CPA/coordination meaning, no
arbitrary positive safety margins, and no bypass architecture.

This is a project implementation audit, not a claim that Lockheed publishes a
ROS queue scheduling algorithm. No new paper-specific implementation claim is
made. Existing 200-second evidence is in
`result/summary/hybrid_formation_stopped_timing_200s_20260906_01/`, specifically
`static_input_and_candidate_failure_review.md` and the referenced bag/log.

The source audit is read-only; implementation changes and recording this report
were performed separately under the user's implementation authorization.

## Baseline findings

- Packet construction copied the held rejoin request even when nominal metadata
  was unavailable. Receiver validation correctly rejects request=true with NaN
  nominal acceleration. The recorded +8 ms nominal timestamp scenario therefore
  necessarily forms rejecting metadata if construction reaches the receiver.
- Worker and ROS output loops drained until empty, allowing concurrent producers
  to extend a single pass. The finite queue capacity alone did not bound the
  number of items consumed while the producer refilled it.
- A packet older than an incomplete staging cache could be reconstructed before
  being rejected. This ordering was unnecessary for cache correctness.
- Detailed per-decision INFO logging shared the output callback path.

Before correction, source traceability and directional objective were aligned,
but implementation correctness failed for the first two findings. The review
did not establish per-event network latency or a hard real-time guarantee.

## Implemented changes

1. `ManeuverSelectionWorkerTrajectory.cpp:222–235`: outgoing rejoin request is
   held-request AND nominal-available AND transmitted-nominal-finite. Missing
   auxiliary metadata cannot invalidate otherwise valid avoidance candidates.
   Actual rejoin/activation state is not overwritten by packet construction.
   Future-time rejection, covariance validation and receiver checks remain.
2. `SpscQueue.hpp:16–21`: consumer-only occupancy snapshot, using the existing
   atomic head/tail. No new lock, allocation, transport or timer. Single-consumer
   ownership is required, just as for try_pop. Snapshot occupancy is <= capacity.
3. `ManeuverSelectionWorker.cpp:259–281`: process only the input entries present
   at entry. FIFO is preserved, new arrivals remain for the next pass, and due
   trajectory/selection work can be reached after at most 64 input handlers.
   This is a work-count bound, not a measured millisecond bound.
4. `DistributedManeuverSelectionRuntime.cpp:397–414`: snapshot trace/output
   counts before draining; at most 256 traces and 16 output batches per callback,
   derived from existing capacities rather than a tuned new batch-size constant.
   Existing publication/control ordering is retained for the captured batches;
   arrivals during the pass are handled on a later timer callback. No deliberate
   dropping, reordering, or early execution of an unconfirmed decision was added.
5. `ManeuverSelectionWorker.cpp:619–647`: reject an older incomplete-staging key
   before reconstruction. Still validate a newer packet before resetting staging;
   invalid newer packets cannot destroy a partially assembled valid library.
6. `DistributedManeuverSelectionRuntime.cpp:828`: detailed decision log becomes
   DEBUG. Startup and warning logs and ROS diagnostics remain. Enabling DEBUG can
   restore the logging cost. HILS analysis scripts do not parse this log line.

Every valid remote candidate still gets the existing spline and uncertainty
propagation. No covariance reuse across different states/inputs is invented.
The 50 ms/250 ms parameters, source-time scheduling, proposal/commit conditions,
execution gates, formation release checks, candidate IDs, and wire schema are
unchanged. The previous drain-until-empty loops were replaced, not left behind
as selectable fallback implementations.

## Verification

- Release build of worker tests, trajectory intent tests, ROS runtime tests and
  the actual `vtol_guidance_node` succeeded (`-O3 -DNDEBUG`).
- Worker/queue tests: **54/54 PASS**. Includes the existing 52 tests plus:
  - Deterministic producer refilling during a captured consumer batch, FIFO
    across ring wrap, empty/full queue handling; next-batch items remain queued.
  - Public worker API scenario with seven candidates and active rejoin: valid
    nominal positive control, +8 ms future nominal, later usable nominal, stale
    nominal and invalid nominal. Seven packets remain available; peer receiver
    still rejects contradictory request=true/NaN packets; avoidance stays active
    without fabricating peer release confirmation.
- Existing ROS runtime tests: **2/2 PASS**, including five independently scoring
  runtimes exchanging intents and decisions.
- Existing trajectory intent tests: **5/5 PASS**, including covariance rejection
  and reconstructed trajectory/cone transport.
- Total relevant tests: **61/61 PASS**. XML: `/tmp/bounded-input-worker-tests.xml`,
  `/tmp/bounded-input-runtime-tests.xml`, `/tmp/bounded-input-intent-tests.xml`.
- `git diff --check` passed. No Pi deployment, live SILS retuning, commit or push
  was included in this correction step.

The new test initially assumed a held rejoin flag would recover immediately.
That assumption was corrected to follow the unchanged monitor-after-build order
and actual peer-confirmation state. Production activation logic was not changed
to force a test outcome. The +8 ms case starts with an explicitly verified held
request; its seven-candidate assertion exercises the original rejecting condition.

## Final five-axis source audit

| Axis | Status | Source evidence | Implementation evidence | Reason | Impact | Confidence |
|---|---|---|---|---|---|---|
| 1. Source accuracy | PASS | Latest user scope; b1e15d4 code; existing bag/log +8 ms evidence | Named packet/queue paths and recorded regression inputs above | Project corrections are not attributed to unpublished Lockheed code; historical timing separated | Prevents unsupported source/performance claims | High |
| 2. Interpretation fidelity | PASS | Preserve candidate/coordination/safety semantics; no experimental tuning | Eligibility clocks, AD/CPA, receiver PSD checks unchanged; report explicitly excludes deadline guarantee | Bounded work and consistent metadata are not called proven real-time or collision safety | No unjustified timing/margin contract added | High |
| 3. Complexity proportionality | PASS | Direct minimal correction, reuse existing mechanisms | One queue occupancy accessor, two worker output count accessors, bounded loops, reordered rejection, metadata condition, log level | No new executor, consensus, timeout, config tuning, or fallback branch | Reduces recurring waste and unbounded producer chasing | High |
| 4. Implementation correctness | PASS | Request must contain usable rejoin data; periodic work must not wait for perpetual producer drain | Future/stale/invalid nominal regression; unchanged receiver; FIFO bounded snapshot test; validation precedes staging reset | Corrects the two concrete source-level defects while preserving required data checks | Valid avoidance candidates survive unavailable rejoin metadata; each pass has a finite entry-batch bound | High for scoped invariants |
| 5. Directional alignment | PASS | User requested static correction, not trial-based tuning | Seven-candidate graph/AD path retained; full valid candidate reconstruction retained; no new safety margin or activation rule | Optimizes processing order/bounds without replacing the avoidance objective | Keeps the experiment's control design intact | High |

Axis 5 was assessed before Axis 4. No applicable source-dependent item is
indeterminate **within this narrowly defined static correction scope**.
Overall scoped static verdict: **PASS**. This is not a performance certification.

## Remaining evidence gaps / explicit exclusions

Actual Pi receipt/callback/input-drain timings and maximum release-to-completion
latency remain unmeasured. Bounded batches can still take longer than 50 ms and
can still overflow finite queues under sustained overload. The change bounds
one pass; it does not claim to eliminate all waiting, DDS/OS jitter or the work
of 560 offered remote candidate packets per second.

Output arriving during a captured callback batch waits until a later callback;
that scheduling tradeoff is intentional and is not described as bit-identical
runtime timing. Existing publication order remains and may still add finite
delay. An eventual Pi run is needed for updated cadence/DSD/formation outcomes.
The prior 11.66 m minimum separation belongs to b1e15d4, not this unrun version.
