# Session Handoff — 2026-05-13 (R-series validation + t=0 sync diagnosis)

> Continuation document for the next session. Self-contained: contains active
> configuration, recent insights, open issue with diagnosis, and proposed next
> steps. New session can resume work without re-reading the entire chat history.

---

## 1. Active configuration (currently live in the codebase)

| Item | Value | Path |
|---|---|---|
| `tau_phi` (τ_φ, roll TC) | **0.5 s** | `StateType.hpp:97`, `airframe_spec.yaml:39` |
| `tau_hdot` (τ_ḣ, altitude TC) | 5.0 s | `airframe_spec.yaml:38` (FW_T_ALT_TC default) |
| `tau_V` (τ_V, airspeed TC) | 5.0 s | `airframe_spec.yaml:37` (FW_T_TAS_TC default) |
| `b_h` (altitude P gain in PD form) | **0.02** | `airframe_spec.yaml:53` — matches FlockingGuidance K_p=0.1 in steady state (b_h ≡ K_p/τ_hdot) |
| `alt_offset` | **0.0** | `airframe_spec.yaml:79` — baseline as-is (debug only) |
| `β_φ` (roll-altitude coupling) | **REMOVED** | Code, yaml, main.cpp all cleaned. TODO(question) preserved at `StateType.hpp:104-116` |
| `baseline_alt` recapture | sp_a ≠ 0 first row | `TrajectoryReplayMode.cpp:202-216` |
| compare timing fix | `stimulus_row + 5` (100 ms) | `compare_single_input.py:107` |
| R-series yaml | 2 segments: V=20 baseline 10s + stimulus 10s | `config/cases/R{05,15,25}{P,M}.yaml` |
| compare PNG layout | **3x3 all-error-norm (NEW 2026-05-13)** | `compare_single_input.py:218-340` |

### R-series final results (latest run, 6 cases × 4.5 s horizon)

| Case | xy_end [m] | h_end [m] | V_end [m/s] | ψ_end [°] | φ_end [°] | ḣ_end [m/s] |
|---|---|---|---|---|---|---|
| R05P | 5.37 | 0.18 | 0.19 | 0.16 | 0.028 | 0.035 |
| R05M | 5.97 | 0.29 | 0.21 | 0.66 | 0.029 | 0.068 |
| R15P | 5.95 | 0.11 | 0.12 | 0.88 | 0.019 | 0.0003 |
| R15M | 4.96 | 0.07 | 0.22 | 0.94 | 0.044 | 0.059 |
| R25P | 6.57 | 0.50 | 0.04 | 2.35 | 0.005 | 0.043 |
| R25M | 4.25 | 0.20 | 0.11 | 2.00 | 0.069 | 0.029 |

**Largest error channel = XY position (4-7 m)**. ψ heading and h altitude are
small (< 2.4°, < 0.5 m). V/φ/ḣ are very small (< 0.1 unit). XY dominates due
to integration of ψ over 4.5 s × V≈20 m/s.

---

## 2. Recent insights (from this session)

### 2.1 *Timing race was the decisive fix, not the model equation*

- User asked "why does predicted h go opposite of measured h?"
- Suspected coordinate-frame sign error → ruled out (signs are correct).
- True cause: `predict_timer` (10 Hz) used baseline **before** the
  `setBaselineAlt()` recapture.
- Fix: `compare_single_input.py:97` use `stimulus_row + TICKS_PER_PREDICT`
  (= 5 rows = 100 ms shift).
- **Effect: h_end mean 0.57 m → 0.16 m (★ 72% improvement)**.

### 2.2 *β_φ (roll-altitude coupling) term is unnecessary*

- After the timing fix, β_φ=0 and β_φ=0.5 produced **numerically identical**
  trajectories (h_end difference < 0.01 m).
- Option A1 form `β_φ·(1-cos φ)` overcorrected at small φ (16× theory).
- Conclusion: adding terms ≠ solution. **Synchronization is the real cause.**
- The term was removed everywhere; TODO(question) at
  `StateType.hpp:104-116` documents when to revisit (φ > 25° with h_end > 0.5 m).

### 2.3 *b_h ≡ K_p/τ_hdot only in steady state*

- FlockingGuidance:    `ḣ^c = K_p·(h^c - h)`     ← proportional, direct output
- TrajectoryPredict:   `ḧ = (ḣ^c - ḣ)/τ + b_h·(h^c - h)`  ← ODE integration
- Steady state match: `b_h = K_p / τ_hdot = 0.1/5 = 0.02`.
- ⚠ Transient response is **structurally different** — they only agree at
  steady state, not over a 4.5 s transient horizon.
- Damping ratio: ζ = (1/τ)/(2√b_h) = 0.707 (critically damped region).
- Previous b_h=0.1 was K_p=0.5 equivalent → under-damped ζ=0.316 → oscillation.

### 2.4 *Debug priority pattern (general)*

1. ✗ Model equation itself (math) — usually fine.
2. ✗ Sign / coordinate frame — usually fine.
3. ✓ **★ Timing / synchronization — the most frequently missed point**.
4. ✗ PX4 cross-coupling (external to model) — last resort.

---

## 3. ★ Open issue (CRITICAL — next session's primary task)

### 3.1 Symptom

In the new error-norm PNG layout (3x3, all-channel norm error), the **error
at t=0 is NOT zero** even though the prediction is initialized from current
measurement at predict-timer call time.

- Typical magnitude at t=0:
  - V:    ~ 0.1 m/s
  - h:    ~ 0.01-0.03 m
  - ψ:    a few degrees during a roll stimulus
  - φ:    < 0.05° (small)
  - ḣ:    small

### 3.2 Root cause — *period mismatch*

```
predict_timer (10 Hz, 100 ms period)       log onTick (50 Hz, 20 ms period)
─────────────────────────────────────      ─────────────────────────────────
t=300ms  m_300 = readMeasurement()         t=300ms  csv row ← m_300 + new_traj
         x0 ← m_300                                  → true_airspeed=m_300, p_V_0=m_300 ★ match
         traj = predict(x0)
         logger.push(traj)
                                            t=320ms  csv row ← m_320 + last_traj
                                                     → true_airspeed=m_320, p_V_0=m_300 ★ mismatch
                                            t=340ms  csv row ← m_340 + last_traj
                                                     → true_airspeed=m_340, p_V_0=m_300
                                            t=380ms  csv row ← m_380 + last_traj
                                                     → true_airspeed=m_380, p_V_0=m_300
t=400ms  m_400 = readMeasurement()         t=400ms  csv row ← m_400 + new_traj
         x0 ← m_400                                  → true_airspeed=m_400, p_V_0=m_400 ★ match
         ...
```

**Out of every 5 CSV rows, only 1 has matching predict-t=0 vs measurement;
the other 4 are 0-80 ms stale.**

### 3.3 Numerical verification (0.1 m/s breakdown)

```
dV in 100 ms ≈
  (V_cmd - V)/τ_V × 0.1s        = 0.01 m/s   (slow asymptote)
  + roll-induced V dip          = 0.02-0.05 m/s (R25 cases especially)
  + EKF airspeed noise          = 0.05-0.10 m/s
  ───────────────────────────────────────────
  Sum                           ≈ 0.08-0.16 m/s    ← matches observation
```

### 3.4 Where the bug is (precisely)

**NOT in `main.cpp:217-224`** — the prediction-initialization code is correct
(x0 = measurement at that moment).

**Bug is in `TrajectoryLogger::onTick()`** — it writes:
- `row.true_airspeed = m_NOW`    (fresh, 50 Hz)
- `row.p_V_0 = traj_last[0]`     (stale, 10 Hz — held over)

into the *same row*, but these two values originate from **different
timestamps** up to 100 ms apart.

---

## 4. ★ Proposed next steps (pick one for the new session)

### Option A — *recommended, accurate*

**Add `m_at_predict_*` columns to CSV.**

Steps:
1. Modify `TrajectoryLogger`:
   - Add new struct fields: `m_at_predict_v`, `m_at_predict_x`, `m_at_predict_y`,
     `m_at_predict_z`, `m_at_predict_vd`, `m_at_predict_yaw`, `m_at_predict_roll`.
   - Add `setMeasurementAtPredict(...)` method called inside `predict_timer`
     lambda (right after `m = logger->getCurrentMeasurements()`).
2. Modify `main.cpp:200` predict lambda — after reading `m`, push it to logger
   alongside the trajectory.
3. Add CSV columns in the logger's row writer.
4. Modify `compare_single_input.py` — use `m_at_predict_*` columns as
   `meas[*]` values instead of `true_airspeed` / `x` / `y` / `z` / `vd` / `yaw` / `roll`.

**Result**: t=0 error becomes *numerically zero* by construction.

**Cost**: 3-4 file changes, ~1 hour. Logger struct grows by 7 doubles per row.

### Option B — *simplest, leave as-is*

Accept the 0.1 m/s mismatch as physical (100 ms timing artifact). Annotate
the PNG with a caption: "t=0 error reflects ≤100 ms predict-vs-log sync gap,
not model error".

**Cost**: 1 line of caption text in `plot_case`.

### Option C — *quick but inaccurate*

Shift measurement by -5 rows in `compare_single_input.py` so that
`meas[k] = df.iloc[stimulus_row + 5k - 5]` (i.e., use the row that was the
last full predict-timer cycle).

**Problem**: predict_timer's actual call time is unknown — only bounded by
`[0, 100 ms]` before each row. So this just shifts the bias, doesn't eliminate
it.

---

## 5. Key files (absolute paths, with critical line numbers)

| Purpose | Path | Key lines |
|---|---|---|
| Prediction ODE | `~/ros2_ws/src/trajectory_prediction/src/TrajectoryPredict.cpp` | 179-191 (state derivatives) |
| Predict-timer lambda | `~/ros2_ws/src/trajectory_prediction/src/main.cpp` | 197-262 |
| Baseline recapture | `~/ros2_ws/src/trajectory_prediction/src/TrajectoryReplayMode.cpp` | 195-213 |
| Parameter struct | `~/ros2_ws/src/trajectory_prediction/include/trajectory_prediction/StateType.hpp` | 93-126 |
| Airframe yaml | `~/ros2_ws/src/trajectory_prediction/config/airframe_spec.yaml` | (entire file) |
| R-series cases | `~/ros2_ws/src/trajectory_prediction/config/cases/R{05,15,25}{P,M}.yaml` | (6 files) |
| compare script (★ updated 2026-05-13) | `~/ros2_ws/src/trajectory_prediction/scripts/compare_single_input.py` | 218-340 (all-error-norm 3x3 layout) |
| Model spec doc | `~/ros2_ws/src/trajectory_prediction/md_file/how_to_predict.md` | (full doc) |

---

## 6. How to run things

### Run a single R-series case (e.g. R15P) and regenerate PNG

```bash
cd ~/ros2_ws
./src/trajectory_prediction/scripts/run_all_cases.sh --cases R15P
~/anaconda3/bin/python3 src/trajectory_prediction/scripts/compare_single_input.py
```

### Run all R-series (6 cases) and regenerate PNGs + summary

```bash
cd ~/ros2_ws
./src/trajectory_prediction/scripts/run_all_cases.sh
~/anaconda3/bin/python3 src/trajectory_prediction/scripts/compare_single_input.py
```

### Output locations

- Per-case PNG: `~/ros2_ws/results/single_input/{case}_compare.png` (3x3 norm-error layout)
- Per-case velocity PNG: `~/ros2_ws/results/single_input/{case}_velocity.png`
- Summary: `~/ros2_ws/results/single_input/summary.{csv,png}`
- Raw CSV (per case): `~/ros2_ws/results/cases/{case}_*.csv`

---

## 7. Memory pointers (auto-loaded in new session)

The new session will auto-load these via `MEMORY.md`:

- `project_trajectory_prediction.md` — package general info
- `project_trajectory_prediction_session_20260513.md` — this session's
  insights + trial-and-error history (7 items preserved)
- `project_collision_avoidance.md` — FlockingGuidance K_p reference
- `project_fw_offboard.md` — FW offboard setpoint types
- `project_fvo_cbf_snapshot_grid.md` — related ACS/FVO research grid

The handoff document you are reading (`succced_0513.md`) is **the** primary
entry point for continuing the t=0 sync work. Memory files give broader
background; this file gives the concrete next action.

---

## 8. Recommended new-session opening prompt

> Continue from `~/ros2_ws/src/trajectory_prediction/md_file/succced_0513.md`.
> Read it first to recover state. Primary task: implement **Option A**
> (add `m_at_predict_*` columns to CSV so that t=0 error in compare PNG
> becomes zero by construction).

---

**Session metadata**:
- Date: 2026-05-13
- Branch: student
- Build: unit tests 12/12 PASS
- Latest R-series PNGs regenerated: 2026-05-13 18:29-18:30
- Largest open issue: t=0 sync mismatch (Option A pending)
