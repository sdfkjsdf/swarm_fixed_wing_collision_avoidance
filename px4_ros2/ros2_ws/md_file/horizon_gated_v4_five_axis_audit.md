# Horizon-Gated V4: Implementation Contract and Five-Axis Audit

Date: 2026-08-31

## Scope implemented in this step

The existing current-state `SafeControlSetV4` core is unchanged. This step adds
an execution supervisor and a candidate hard filter around it:

1. Align each transmitted trajectory cone to one evaluation timestamp.
2. At every common 0.1 s sample over the remaining 4.5 s horizon, compute

   \[
   h_k = \lVert p_{B,k}-p_{A,k}\rVert
         -(r_A+r_B)-r_{95,k},
   \]

   where \(r_{95,k}\) is the support of the summed relative position
   covariance in the instantaneous line-of-sight direction.
3. Define \(h_{\mathrm{worst}}=\min_k h_k\) over every aircraft pair for the
   transmitted `NearNominal` V4 candidates.
4. Open the V4 command gate when \(h_{\mathrm{worst}}\le 10\,\mathrm{m}\).
5. Before AD scoring, reject candidate combinations that violate the
   positive-margin discrete condition at any pair or horizon interval.
6. Use PMR/MASD/AD cost only to rank combinations that passed the hard filter.
7. Once the distributed gate opens, latch only each aircraft's own committed
   V4 candidate for the same 4.5 s horizon used by the ZOH predictor. Do not
   freeze the locally inferred five-aircraft tuple. A peer's published actual
   command state constrains subsequent evaluations.
8. If the existing V4 interval search explicitly returns
   `SearchSetInfeasible`, fail closed by requesting the gate and retaining the
   last coordinated ownship command. Do not treat an unproven interval as
   permission to resume the point-convergence command.

`NearNominal` means the V4 adapter's transmitted candidate closest to the
nominal heading-rate command. It is not a separately transmitted raw formation
trajectory. That distinction is deliberate and is exposed here to prevent the
gate from being misreported later.

## Source classification

- Source-backed: sampled 4.5 s candidate propagation; checking every threat
  and time sample; applying the positive-margin discrete condition before
  PMR/MASD/AD/Cost. See
  `reference/paper/tc_cbf_schmitt_interval_safe_control_note_v3.md`, Sections 0
  and 11.
- Project integration: using propagated 95% position-covariance support in
  \(h_k\), using DSD = 10 m as the gate/reference value, separating the V4
  gate from AMAC activation, and holding an activated ownship command for the
  predictor's 4.5 s ZOH horizon. Treating `SearchSetInfeasible` as a
  fail-closed gate request is also a project integration safety policy.
- Not claimed: a 95% probability that an entire continuous trajectory lies in
  the cone, inter-sample invariance, recursive feasibility, or actuator-lag
  robustness.

## Five-axis result after remediation

| Axis | Result | Evidence |
|---|---|---|
| Source accuracy | PASS | The implementation follows the local v3 sampled-data ordering and labels covariance/gating choices as project integration, not as disclosed Lockheed or TC-CBF facts. |
| No over-interpretation | PASS | Diagnostics report a pointwise 95% ellipsoid support and `h_worst`; no whole-trajectory probability or formal safety guarantee is asserted. |
| Proportional complexity | PASS | The existing time alignment, cone interpolation, uncertainty propagation, candidate transport, joint evaluator, proposal consensus, and worker thread are reused. No node, optimizer, predictor, or communication thread was added. |
| Implementation correctness | PASS | The robust clearance is evaluated for every aligned sample; the discrete residual is evaluated for every interval; the hard filter precedes AD evaluation; the command gate is separate from AMAC activation; only the ownship command is latched for 4.5 s; infeasible interval search fails closed. All 122 package tests and the five-aircraft acceptance SILS pass. |
| Directional alignment | PASS | `h_worst` now controls V4 execution, while AD remains only downstream ranking among hard-filtered candidates. The legacy `positive_margin_filter_enabled` behavior remains unchanged under a separate flag. |

The first SILS run invalidated the initial implementation-correctness PASS:
different 20 Hz cone snapshots made local gate onset differ by about 9 s and
the minimum separation fell to 1.67 m. A distributed OR synchronized the gate
in the second run, but that run also failed: the gate closed after roughly
0.5 s, the raw point-convergence command resumed, and minimum separation fell
to 1.22 m. This contradicted the predictor's 4.5 s held-input assumption.

The current remediation keeps each local gate diagnostic, rolls only the raw
local requests up with a distributed OR, and latches each aircraft's own
committed V4 command for at least 4.5 s. It deliberately does not latch the
five-aircraft tuple inferred by that node. A fourth run then exposed a separate
fail-open path: after the hold ended, `SearchSetInfeasible` did not reopen the
gate and minimum separation fell to 3.01 m. The final remediation maps that
explicit status to a local fail-closed request.

The final acceptance run
`horizon_gated_v4_hworst_failclosed_20260831_05` produced:

- minimum 3-D separation: 11.967 m,
- DSD = 10 m violation samples: 0,
- four synchronized activation starts per aircraft,
- completed active intervals of approximately 4.50--4.56 s,
- active candidate switches: 0 for every aircraft,
- active input-revision switches: 0 for every aircraft,
- common selected tuple agreement: 60/60 epochs,
- peer ownship-assumption mismatches: 0.

## Adversarial checks

- An unsafe intermediate cone sample is rejected even when the endpoint is
  safe.
- Relative covariance is summed and projected on the instantaneous LOS at the
  actual minimum sample.
- Future, stale, invalid, or non-overlapping horizons do not create a new gate
  activation; a previously active gate is retained on transient invalid data.
- A V4 command cannot execute without both distributed coordination and a
  committed V4 candidate set.
- `SearchSetInfeasible` requests the gate; it cannot silently fall through to
  the nominal point-convergence command.
- A peer's raw local gate request is distributed separately from the rolled-up
  gate state, preventing self-sustaining distributed OR feedback.
- An active aircraft retains only its own candidate ID, input revision, and
  command input for the 4.5 s hold; peer assumptions remain replaceable by
  their published actual commands.
- The original direction-based positive-margin tests still pass, proving the
  new robust filter did not silently change the legacy filter contract.

## Remaining validation boundary

One successful SILS run is an integration acceptance result, not a statistical
95% trajectory-containment proof. Repeated randomized and Monte Carlo runs are
still required to estimate violation probability and calibrate covariance and
model-error assumptions. `SearchSetInfeasible` remains visible in diagnostics;
fail-closed execution does not relabel it as a safe candidate set.
