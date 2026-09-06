# Stopped-only stage timing (baseline fd1698b)

Optional `stopped_stage_timing_enabled` (default false), independently enabled
with `STOPPED_STAGE_TIMING_ENABLED=true` by the shared guidance launcher. The
hybrid launcher passes the same variable to Pi. Existing MASD diagnostics are
unchanged; enabling this option does not enable their publisher.

## Runtime contract

- Allocate/value-initialize a 16384-record buffer once, before the worker starts.
  Each record is 48 bytes on the current ABI (768 KiB total record storage).
- Read only steady start/end times around complete candidate-refresh (stage 1),
  `evaluateCurrentSet` (stage 2), and activation-monitor (stage 3) calls.
- Keep temporary scalar records on the stack. Append to the private buffer only
  after the original `publishOutput` attempt; mark failed output enqueue.
- Preserve `drainWorkerOutput` byte-for-byte relative to fd1698b. No added ROS
  messages, DDS publishers, timers, live consumers, locks, files or statistics.
- When full, drop further records and count them; retain the initial interval,
  without overwrite, growth or blocking. Timestamp reads continue while enabled.
- The lifecycle owner calls `stopAndWriteStageTiming` only after ROS spin ends.
  It stops and joins the worker before reading the buffer. Formatting is then
  accumulated and written to stdout, captured by existing guidance_N.log/SSH.
  No concurrent restart/export or test-driven processing is permitted.

No zero-overhead claim: timestamp reads and scalar stores consume time, including
small bookkeeping before output handoff and append work before the next cycle.
The default-off path allocates no timing buffer and makes no added clock calls.
It still has conditionals/temporary metadata. New timing is not a safety input.

## Span and output interpretation

Stage 1 includes candidate construction/local reconstruction, and V4 if explicitly
enabled. It excludes input draining, belief compensation and candidate-ID refresh.
Stage 2 includes proposal work, not just graph evaluation. Its unavailable flag
means `proposal_valid=false`, not necessarily a skipped evaluation. Stage 3 has
no artificial 50 ms deadline, since it also runs on commit events.

End-to-end command delivery and ROS callback waiting are not measured by these
spans. Completion cadence uses successful stage end times with output enqueued;
it is not DDS arrival or actuator cadence. Source times are worker state times,
not central bag receipt times. Do not use the latter as `--start-source-us`.

## Shutdown requirement (important for the existing hybrid runner)

The guidance process must exit gracefully (SIGINT/SIGTERM handled by ROS).
SIGKILL/container forced removal/power loss cannot export an in-memory buffer.
The existing remote launcher uses `ros2 run` as PID 1 and force-removes containers
in cleanup; do **not** assume `docker stop` of that wrapper exports the child.
For a measurement run, after recording is complete, signal the actual
`vtol_guidance_node` process gracefully while keeping the SSH stdout connection
open, wait for `[stop-stage-end]` in its PC log, then remove the container.
Do not signal it during the evaluated flight. No hybrid cleanup changes were
bundled into this implementation; this post-run collection step must be observed
when a Pi measurement is next requested. A missing footer is a failed measurement,
never treated as evidence of good cadence. Production defaults remain unchanged.

## Offline analysis

```bash
python3 analysis/analyze_stopped_stage_timing.py \
  --log result/log/RUN/guidance_0.log \
  --output result/summary/RUN/pi_stopped_timing.json
```

The standard-library-only parser requires one complete versioned dump with a
matching count/footer. It rejects truncation and invalid/overlapping spans,
reports drops, unavailable results and failed output handoffs, and separates
compute duration from completion intervals. It does not run on the aircraft.

## Verification boundary

Release build; worker regression checks compare timing ON/OFF on identical input
sequences (including means/covariances, candidates, agreement and execution
flags), fixed-capacity overflow, independent enable, disabled empty output and
stop/join before export. ROS integration verifies exported stages after stopping
the executor. Offline tests cover compute/cadence separation and incomplete dump
rejection. These tests are not a SILS equivalence or hard real-time guarantee.
Pi deployment and a new 200-second SILS were subsequently completed as described
below; they were not part of the initial local implementation checks.

Local verification completed: 52/52 worker tests, 2/2 ROS runtime tests, 3/3
offline parser tests (57 total); Release build, shell syntax and diff whitespace
checks passed. Static comparison confirms the entire drainWorkerOutput body is
byte-identical to fd1698b. XML results are in /tmp/stopped-stage-*-tests.xml.

## Subsequent hybrid verification

Run `hybrid_formation_stopped_timing_200s_20260906_01` used Pi vehicle 0 and four
local guidance nodes, with Formation pentagon, formation discrimination disabled,
and the existing controller behavior/configuration retained. The optional buffer
was enabled on all five nodes. The 200-second recording yielded a 180.3-second
common flight evaluation window. All five complete dumps were recovered with
zero buffer overflow. Minimum sampled 3D separation was 11.6649 m (DSD 10 m,
zero sampled violations); final position/velocity spread was 34.4759 m and
0.33834 m/s. This single run supports successful operation with instrumentation,
not zero overhead or deterministic equivalence across separate SILS runs.

Pi mean completion rates were 19.9841 Hz for trajectory refresh and 3.99848 Hz
for combination selection; maximum computation spans were 3.73174 ms and
9.62226 ms. Maximum completion intervals were 101.229 ms and 282.359 ms,
respectively: mean target cadence is not a strict per-cycle deadline guarantee.

The offline analyzer now accepts `--start-source-us` and `--end-source-us` to
exclude preflight and post-recording shutdown records. Bounds must come from
valid, non-backward state timestamps corresponding to the recorded flight window;
they are not exact receipt-time alignment. Complete-dump validation and overflow
counts still cover the original dump. The added window test passes: 4/4 offline
tests, together with the previously passed 52 worker and 2 runtime tests (58).
Per-node JSON, manifest and detailed notes are under
`result/summary/hybrid_formation_stopped_timing_200s_20260906_01/`.
