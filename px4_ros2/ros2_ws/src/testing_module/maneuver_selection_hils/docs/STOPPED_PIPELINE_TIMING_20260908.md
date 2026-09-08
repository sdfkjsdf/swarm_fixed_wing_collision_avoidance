# Stopped pipeline timing — measurement-only extension

Base: `2fbb5ef`. Goal: distinguish arrival cadence, ownship queue waiting and
input processing from the existing stage computation spans. No controller,
queue bounds/order, epoch, threshold, uncertainty, or output-drain logic changes.

Runtime uses the existing default-off `stopped_stage_timing_enabled` option.
Callback reads steady/system time and available RMW received time, passes scalars
through the existing queue; no producer writes the worker-owned timing buffer.
Worker times belief acceptance, remote-intent processing, and nonempty passes.
Fixed preinitialized buffers retain the first 131072 pass / 32768 belief records,
with separate overflow counters. No added ROS messages/timer, locks, live I/O,
statistics, allocations during recording or live consumer. Shutdown joins the
worker and exports buffers. Record storage grows by approximately 10.25 MiB/node.
Clock calls and scalar stores are nonzero overhead, including idle-poll clocks.

Static checks: timing fields are never read by a control predicate; same accept
functions execute in the same order; callback metadata is diagnostic-only;
runtime output drain unchanged; no shared mutable buffer between callback and
worker. Single-threaded producer contract remains unchanged. Disabled mode has
no buffer allocation or added clock calls, but metadata/conditional overhead.

Local Release verification: 54 worker/queue tests (including ON/OFF result
comparison and bounded-buffer retention), 2 ROS runtime tests, 8 offline parser
tests. All passed. Pi Release build succeeded; installed/build ELF Build IDs
match (installation changes RPATH, so full-file hashes need not match).

Offline analyzer: `analysis/analyze_stopped_pipeline_timing.py`. Complete footer
and counts are mandatory; absent middleware timestamp is unavailable, not zero
delay. Source-to-callback is a cross-clock aggregate, not pure network transit.
Pass decomposition excludes unrecorded idle polls/sleep/scheduling; residual
time is not automatically labelled DDS delay. This instrumentation does not
measure publisher NIC egress, packet ingress, ROS publication or PX4 application.
It cannot certify release-relative hard deadlines or zero observer effect.

New 200-second hybrid measurement is recorded separately from the base run.
Analyze the same common post-Formation source-time window, report queue drops,
buffer overflow, candidate completeness, compute and cadence separately. Do not
silently tune the controller to obtain better flight results.
