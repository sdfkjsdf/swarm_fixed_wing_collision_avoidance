# PROPOSAL: TrajectoryPredict 명세 §11 보완 — schedule-aware 패턴 명시

> **수신**: 가이던스 알고리즘 설계자
> **발신**: TrajectoryPredict 구현 + 검증 담당
> **날짜**: 2026-05-11
> **참조 명세**: `~/ros2_ws/md_file/TASK_trajectory_predictor.md`
> **목적**: 명세는 100% 준수했으나, 검증 단계에서 ZOH 가정의 한계가 두드러짐.
>          명세 §11 의 "단일 호출 책임" 원칙은 유지하면서, 호출자가 미래 입력 schedule
>          을 알 때 `stepRK4` 직접 호출로 schedule-aware 예측이 가능함을 명시 권장.

---

## 1. 배경 — 명세 100% 준수 구현 + SITL 검증

명세 §1~§11 그대로 구현했으며 다음을 모두 통과:

| 항목 | 결과 |
|---|---|
| 단위테스트 10개 (정확성 7 + RT 특성 3) | 10/10 PASS |
| WCET (horizon=15) | mean 2.93μs · p99 3.69μs (목표 < 30μs) |
| Heap-free (predict<30> ×1000) | malloc 0 회 |
| Jitter ratio (σ/mean) | 0.079 (목표 < 0.1) |
| SITL 통합 (PX4 v1.17 + Gazebo Standard VTOL) | 천이 + Replay 35초 시퀀스 정상 비행 |
| CSV 50Hz × 7-state predict 매 100ms 갱신 | 정합성 검증 통과 |

---

## 2. 검증에서 발견된 패턴

### 2.1 짧은 horizon rolling 예측 (§5.2 의 N_STEPS 컨벤션) — 정확

매 100ms 마다 새 4.5초 예측. 4.5초 이내 segment 변화 없는 경우 (cruise 안정):
- XY 오차 1~2m / 4.5s
- psi 오차 < 1°
- altitude 오차 < 0.1m

→ 모델 baseline 정확도 매우 양호.

### 2.2 ZOH 가정의 한계 (segment 전환을 horizon 안에 포함하는 chunk)

명세 §5 의 호출 패턴 예시 (`m_predictor->predict(current_state, current_setpoint_input, m_predict_dt, m_predicted_traj_rt)`) 는 단일 입력 ZOH 가정. 호출자가 시퀀스 전체를 미리 알아도 한 번의 `predict()` 호출은 그 시점 setpoint 가 horizon 끝까지 유지된다고 가정.

**검증 결과 (rolling-mode CSV 에서 4.5s 간격 chunk 추출, ZOH only)**:

| chunk | t_start | 이벤트 | XY end | psi end |
|---|---|---|---|---|
| 2 | 9.0s | seg1→seg2 전환 (10s 우선회) | **10.14m** | **39.22°** |
| 3 | 13.5s | seg2→seg3 전환 (15s 직진 가속) | **10.33m** | **29.43°** |

→ horizon 안에 segment 전환 들어오면 모델이 옛 setpoint 로 끝까지 적분 → 발산.

### 2.3 호출자 책임으로 우회 가능 (라이브러리 변경 0)

명세 §4.3 의 public API 인 `stepRK4(x, u, dt) const` 가 이미 존재. 호출자가 시간 따라가며 직접 호출:

```cpp
// 호출자 (검증용 wall_timer / 또는 collision_avoidance 의 MPC trajectory 호출자)
PredictState x = x0;
double t_now = ...;   // 현재 시뮬/시퀀스 시간
for (std::size_t k = 0; k + 1 < N_STEPS; ++k) {
    PredictInput u_at_t = sequencer_or_planner->lookup(t_now + (k+1) * dt);
    x = predictor->stepRK4(x, u_at_t, dt);
    out_traj[k+1] = x;
}
```

**검증 결과 (schedule-aware 적용 후)**:

| chunk | XY end (ZOH) | XY end (schedule-aware) | psi end (ZOH) | psi end (schedule-aware) |
|---|---|---|---|---|
| 2 | **10.14m** | **7.98m** (-22%) | **39.22°** | **2.55° (-93%)** |
| 3 | 10.33m | 3.59m (-65%) | 29.43° | 5.30° (-82%) |
| 4 | 6.29m | 3.67m (-42%) | 18.20° | 5.13° (-72%) |

→ heading 채널이 가장 극적. 라이브러리 변경 없이 호출자 패턴 변화만으로 segment 전환 발산 거의 0.

---

## 3. 제안 — 명세 §11 에 다음 한 줄 추가 (Anti-requirements 보완)

### 현재 §11

```
- 동적 horizon 금지
- mutex / lock 사용 금지
- 풍속 모델 추가 금지
- 신경망 / 학습 기반 예측기 추가 금지
- 이웃/swarm 시나리오 가정 금지
  (본 클래스는 단일 무인기 1회 호출만 책임. 호출 패턴은 호출자 책임)
```

### 제안 추가 (§11 마지막 또는 §5 "호출 컨텍스트" 에 추가)

> **schedule-aware 예측이 필요한 경우의 호출 패턴**:
> 호출자가 미래 입력 시퀀스를 알고 있는 경우 (예: trajectory_replay_node 의 SetpointSequencer,
> 또는 collision_avoidance 의 MPC plan), 단일 `predict<N>()` 호출 (ZOH) 대신 `stepRK4`
> 직접 호출로 시간 따라가며 입력을 갱신할 수 있다:
>
> ```cpp
> for (k = 0; k+1 < N; ++k) {
>     PredictInput u_at_t = horizon_input_source(t + (k+1) * dt);
>     x = predictor->stepRK4(x, u_at_t, dt);
>     out_traj[k+1] = x;
> }
> ```
>
> 라이브러리는 이 패턴을 제약하지 않음 (`stepRK4` 가 이미 public).
> 호출자가 schedule 모르면 (rt_loop 의 매 ms 새 입력) 단일 `predict<N>(x0, u_zoh, dt, out)`
> ZOH 사용 — 둘 다 정상적인 사용법.

---

## 4. 구현 영향 범위

라이브러리 자체 변경 0. 검증용 호출자 (`main.cpp` 의 wall_timer 람다) 만 다음 패턴 적용:

```diff
  auto predict_timer = node->create_wall_timer(predict_period,
-     [predictor, logger, predicted_traj, dt]() {
+     [predictor, logger, sequencer_for_predict, predicted_traj, dt]() {
          ...
          PredictState x0 = ...;   // 6→7 변환 어댑터
-         PredictInput u_zoh{ sp.V, sp.h_dot, sp.a_lat };
-         predictor->predict<N>(x0, u_zoh, dt, *predicted_traj);
+         const double t_now = logger->getReplayTime();
+         if (std::isfinite(t_now)) {
+             auto & traj = *predicted_traj;
+             traj[0] = x0;
+             PredictState x = x0;
+             for (std::size_t k = 0; k+1 < N; ++k) {
+                 const auto sp_at_t = sequencer_for_predict->lookup(t_now + (k+1) * dt);
+                 PredictInput u_at_t{ sp_at_t.V, sp_at_t.h_dot, sp_at_t.a_lat };
+                 x = predictor->stepRK4(x, u_at_t, dt);
+                 traj[k+1] = x;
+             }
+         } else {
+             /* fallback: ZOH (Preflight 단계) */
+             PredictInput u_zoh{ sp.V, sp.h_dot, sp.a_lat };
+             predictor->predict<N>(x0, u_zoh, dt, *predicted_traj);
+         }
          logger->pushPredictedTrajectory(*predicted_traj);
      });
```

추가 변경:
- `TrajectoryLogger`: `m_replay_t_relative` atomic + `setReplayTime/getReplayTime` 추가
- `TrajectoryReplayMode::rt_loop`: 매 cycle 마다 `m_logger->setReplayTime(t)` 한 줄

명세 §6.2 의 모든 RT 보장 그대로 (heap-free, mutex-free, branch-minimized) — 호출자 패턴이 stack-only 루프이므로.

---

## 5. 추가 검증 — 종/횡 채널 분리 시퀀스

명세 외 추가 시퀀스 (h_dot=0 모든 segment, V/a_lat 만 변화) 도 검증:
- 고도 발산 거의 0 (h_end < 1m 모든 chunk)
- 순수 횡 응답 baseline: psi 4~6° / 4.5s
- 시정수 식별 (FW_R_TC) 단독 검증 가능

상승+선회 시퀀스와 보완적으로 활용:
- 수평 only: τ_a 단독 식별
- 상승+선회: 종/횡 결합 효과 측정

---

## 6. 정리

| 측면 | 현재 명세 | 제안 |
|---|---|---|
| 라이브러리 API | `predict<N>` ZOH + `stepRK4` public | **변경 없음** |
| 명세 §11 | "단일 호출 책임" | **유지 + schedule-aware 패턴 명시 한 줄 추가** |
| RT 보장 (§6) | heap-free / mutex-free / WCET | **유지** (schedule-aware 도 stack-only 루프) |
| 검증 정확도 | ZOH 단독 측정 | **두 모드 비교 가능 → 모델 정확도 vs 입력 가정 영향 분리** |

→ **명세의 단순성 + 호출자 유연성 둘 다 확보**. 검증 결과는 **모델 baseline ~3-8m / 4.5s, segment 전환 발산 별도** 로 정량화됨.

---

## 첨부 자료

- 검증 CSV (상승+선회): `/tmp/trajectory_20260511_132823.csv` (24.9 MB)
- 검증 CSV (수평 only): `/tmp/trajectory_20260511_142459.csv` (~13 MB)
- 사후분석 스크립트: `~/ros2_ws/src/trajectory_prediction/scripts/chunk_analysis.py`
- ZOH plot: `/tmp/chunk_error_curves.png`, `/tmp/chunk_trajectories.png`
- Schedule-aware plot: `/tmp/aware/chunk_*.png`
- 수평 only plot: `/tmp/horizontal/chunk_*.png`
- 메모리 (작업 영속): `~/.claude/projects/-home-leedonghyuck/memory/project_trajectory_prediction.md`
- 작동 원리 + 질문 자료 (이전 작성): `~/ros2_ws/md_file/TRAJECTORY_PREDICT_HOW_IT_WORKS.md`
