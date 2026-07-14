# Handoff — 2026-05-11 PX4 시정수 동기화 후 검증 (미완료)

이 문서는 새 Claude 세션에서 같은 작업을 이어가기 위한 컨텍스트 묶음.
**원본 세션**: `/home/leedonghyuck/.claude/projects/-home-leedonghyuck-ros2-ws/97a94807-02a6-489b-860a-2c29be465a4e.jsonl`

---

## 1. 한 줄 요약

PX4 SITL 의 실제 시정수 (`FW_T_ALT_TC=5.0`) 와 모델 시정수 (`tc_alt=2.0`) 가 -60% 다른 걸 발견 → `airframe_spec.yaml` 동기화 완료. **하지만 동기화 효과 검증 비행은 SITL 부팅 트러블로 미완료.**

---

## 2. 이번 세션에서 한 일 (완료)

### 2.1 PX4 SITL live 파라미터 readout (완료)
`pxh>` 콘솔 직접 입력으로 1040_gazebo-classic_standard_vtol 의 실제 파라미터 6개 확인.

| 파라미터 | SITL live | 이전 모델 | 갱신 후 | 변경 |
|---|---|---|---|---|
| `FW_T_TAS_TC` (tau_V) | 5.0 s | 4.0 s | **5.0** | -20% |
| `FW_T_ALT_TC` (tau_hdot) | 5.0 s | 2.0 s | **5.0** | **-60%** ★ |
| `FW_R_TC` (tau_phi) | 0.4 s | 0.5 s | **0.4** | +25% |
| `FW_R_LIM` | 50° | 50° | 50° | OK |
| `FW_AIRSPD_MIN` | 10 m/s | 12 m/s | **10.0** | -17% |
| `FW_AIRSPD_MAX` | 25 m/s | 25 m/s | 25.0 | OK |

→ 1040 기체 설정이 위 3개 시정수를 재정의하지 않아 PX4 default 값이 그대로 적용됨.

### 2.2 `airframe_spec.yaml` 동기화 (완료, 디스크에 반영됨)
파일: `~/ros2_ws/src/trajectory_prediction/config/airframe_spec.yaml`
```yaml
airspeed_min: 10.0      # 12 → 10
tc_tas:  5.0            # 4.0 → 5.0
tc_alt:  5.0            # 2.0 → 5.0  ★ 가장 중요
tc_roll: 0.4            # 0.5 → 0.4
```

### 2.3 launch_1vtol_replay.sh 신규 작성 (완료, 1차 버그 fix 완료)
파일: `~/ros2_ws/src/trajectory_prediction/scripts/launch_1vtol_replay.sh` (실행권한 ON)

7 단계 통합:
1. 정리 (기존 px4/gz/agent/노드 KILL + 락 정리)
2. MicroXRCEAgent 시작
3. gzserver headless 시작
4. PX4 instance 0 (PX4_UXRCE_DDS_NS=px4_0)
5. SDF spawn (standard_vtol_0)
6. ready check (`/px4_0/fmu/out/register_ext_component_reply_v1` polling, max 30s)
7. trajectory_replay_node 실행 (foreground, Ctrl+C trap 으로 전체 정리)

**1차 버그**: `set -u` 가 `setup_gazebo.bash` source 시 미정의 env 변수 (`$GAZEBO_PLUGIN_PATH:` 빈 prefix concat) 에서 셸 전체를 죽임 → `set -u` 제거 후 정상 동작 가능.

사용법:
```bash
~/ros2_ws/src/trajectory_prediction/scripts/launch_1vtol_replay.sh           # 전체 + 노드
~/ros2_ws/src/trajectory_prediction/scripts/launch_1vtol_replay.sh --no-node # SITL 만
```

### 2.4 메모리 갱신 (완료)
파일: `~/.claude/projects/-home-leedonghyuck/memory/project_trajectory_prediction.md`
- frontmatter description 갱신
- "★ PX4 SITL live 시정수 동기화" 섹션 추가
- 7-state POD 마지막 멤버 `a_lat → phi` 명시 (이전 phi PATCH 반영 누락분 보정)
- 단위테스트 12/12 (phi-PATCH 추가 2개) 반영
- MEMORY.md 인덱스도 한 줄 갱신

---

## 3. 미완료 — 다음 세션에서 해야 할 일

### 3.1 [최우선] tc 동기화 효과 검증 비행
**현재 상황**: launch_1vtol_replay.sh 의 `set -u` 버그 수정 직후 사용자가 다시 실행 안 한 상태.

**다음 액션**:
```bash
~/ros2_ws/src/trajectory_prediction/scripts/launch_1vtol_replay.sh
```

성공 신호 (모두 보여야 정상):
- `[launch 1/7]` ~ `[launch 7/7]` 단계별 메시지
- `[launch]   → ready (대기 Ns)` (uXRCE-DDS 토픽 대기 통과)
- 노드 로그에 `[main] Predictor: horizon=4.50s rate=10.0Hz call=10.0Hz (N=45 dt=0.100s tau_V=5.00 tau_hdot=5.00 tau_phi=0.40)` ← `tau_V=5.00 tau_hdot=5.00 tau_phi=0.40` 가 동기화 적용 확인 신호
- `[main] doRegister() 성공 → spin 시작`
- 자동 이륙 → VTOL→FW 천이 → 35초 시퀀스
- 약 50~55초 후 시퀀스 완료 (hold_last 진입)

**중간 발생했던 트러블 (참고용)**:
- 초기 부팅 시 `Quad-chute triggered` (VTOL 천이 실패) 한 번 발생 — attitude estimator 초기화 글리치, dataman 캐시 정리 후 재시도 권장
- `doRegister()` 15초 timeout — PX4 가 uXRCE-DDS 연결 완료 전에 노드 시작 시 발생. launch script 의 ready check 가 이걸 막아줌
- single-vehicle SITL (`make px4_sitl gazebo-classic_standard_vtol`) 은 `topic_namespace_prefix=""` 가 doRegister 와 mismatch → multi-vehicle 패턴 (`PX4_UXRCE_DDS_NS=px4_0`) 필수

### 3.2 [검증 끝나면] chunk_analysis 비교
```bash
mkdir -p /tmp/tc_synced
/home/leedonghyuck/anaconda3/bin/python3 \
  ~/ros2_ws/src/trajectory_prediction/scripts/chunk_analysis.py \
  --out-dir /tmp/tc_synced/
```

**비교 대상**:
- `/tmp/horizontal/` ← 이전 a_lat 기반 + tc_alt=2.0
- `/tmp/phi/` ← 이전 phi 기반 + tc_alt=2.0
- `/tmp/tc_synced/` ← 이번 phi 기반 + tc_alt=5.0 (★ 새 데이터)

**예상 효과**: 수평 시퀀스에서는 h_dot=0 이라 tc_alt 영향 거의 없음 (대조군). 상승/강하 추가 시퀀스 비행하면 tc_alt 효과가 크게 보일 것.

### 3.3 [선택] 상승/강하 시퀀스로 tc_alt 효과 강화
`~/ros2_ws/src/trajectory_prediction/config/setpoint_sequence.yaml` 의 일부 segment 에 `height_rate: 2.0` 또는 `-1.5` 같은 값 추가 → tc_alt=2.0 vs 5.0 차이가 chunk 발산에 명확히 나타남.

---

## 4. 참조 — 핵심 파일 경로

### 4.1 기준 명세 / 디자인
- `~/ros2_ws/md_file/TASK_trajectory_predictor.md` — 라이브러리 명세 (ground truth)
- `~/ros2_ws/md_file/PROPOSAL_trajectory_predictor_update.md` — 명세 업데이트 제안
- `~/ros2_ws/md_file/TRAJECTORY_PREDICT_HOW_IT_WORKS.md` — 외부 Claude 자문용 모델 설명
- `~/ros2_ws/PROJECT_FILE_DIAGRAM.md` / `FORMATION_MODE_ARCHITECTURE.md` / `SUBSCRIBER_DATA_FLOW.md`

### 4.2 코드 (현재 상태 모두 빌드 통과 + 단위테스트 12/12)
- `~/ros2_ws/src/trajectory_prediction/include/trajectory_prediction/StateType.hpp` — 7-state POD (`phi` 마지막)
- `~/ros2_ws/src/trajectory_prediction/include/trajectory_prediction/TrajectoryPredict.hpp` — public API + private InternalInput
- `~/ros2_ws/src/trajectory_prediction/src/TrajectoryPredict.cpp` — Beard-McLain phi 기반 ODE + RK4
- `~/ros2_ws/src/trajectory_prediction/src/main.cpp` — wall_timer schedule-aware predict + ZOH fallback
- `~/ros2_ws/src/trajectory_prediction/src/TrajectoryLogger.cpp` — CSV 50Hz 기록 (헤더 `p_phi_k`)
- `~/ros2_ws/src/trajectory_prediction/test/test_trajectory_predict.cpp` — 12개 테스트
- `~/ros2_ws/src/trajectory_prediction/scripts/chunk_analysis.py` — 사후 분석
- `~/ros2_ws/src/trajectory_prediction/scripts/launch_1vtol_replay.sh` — ★ 본 세션 신규

### 4.3 yaml (★ tc 동기화 후)
- `~/ros2_ws/src/trajectory_prediction/config/airframe_spec.yaml` — `tc_tas:5.0 tc_alt:5.0 tc_roll:0.4 airspeed_min:10.0`
- `~/ros2_ws/src/trajectory_prediction/config/replay_params.yaml` — predict 메타
- `~/ros2_ws/src/trajectory_prediction/config/setpoint_sequence.yaml` — 35s 수평 시퀀스 (h_dot=0 모두)

### 4.4 메모리
- `~/.claude/projects/-home-leedonghyuck/memory/MEMORY.md` (인덱스)
- `~/.claude/projects/-home-leedonghyuck/memory/project_trajectory_prediction.md` ★ 갱신 완료
- 기타: `user_profile.md`, `project_px4_ros2_setup.md`, `project_collision_avoidance.md`, `feedback_coding_style.md`

### 4.5 검증 산출물 (PNG / CSV)
- `/tmp/horizontal/chunk_*.png` — a_lat + tc_alt=2.0 (수평 시퀀스, 이전 baseline)
- `/tmp/phi/chunk_*.png` — phi + tc_alt=2.0 (PATCH 적용, tc 동기화 전)
- `/tmp/tc_synced/chunk_*.png` — ★ 새 세션에서 만들 결과

### 4.6 점검 에이전트
- `~/ros2_ws/.claude/agents/design-compliance-checker.md` — 코드 수정 후 자동 호출
- 7 카테고리 점검 (네이밍 / thread / rt_loop / ModeBase / fallback / heap0 / sub-pub)

---

## 5. 새 세션 시작 시 권장 첫 메시지

```
지난 세션 핸드오프 문서: /home/leedonghyuck/ros2_ws/md_file/HANDOFF_2026-05-11_tc_sync_verification.md
이 문서 §3 부터 이어서 진행. 우선 launch_1vtol_replay.sh 실행해서 검증 비행부터.
```

→ Claude 가 이 문서 + `project_trajectory_prediction.md` 메모리 둘 다 읽으면 완전한 컨텍스트 복원됨.

---

## 6. 디자인 원칙 (변경 불가, 라이브러리 명세 §1~§4)

- **7-state POD**: `(p_n, p_e, h, V, psi, h_dot, phi)` double, sizeof=56B
- **3-input POD (외부)**: `(V_cmd, h_dot_cmd, a_lat_cmd)` — `FwSetpointOutput_rt2mt` 1:1 매칭
- 내부에서 `phi_cmd = atan2(a_lat_cmd, g)` 변환 (stepRK4 진입 시 1회)
- stateless / heap-free / lock-free
- horizon = 45 (= 4.5s × 10Hz), `TrajectoryLogger::kPredictHorizon` constexpr
- 외부 의존 0 (Eigen / rclcpp / px4_msgs include 금지, 라이브러리 자체)
- 시정수 외부 주입 (airframe_spec.yaml 의 tc_tas/tc_alt/tc_roll)
- 분기 회피 (clamp → fmin/fmax, wrapPi → fmod 2회)

---

## 7. 한 번에 다음 단계 실행 (cheat sheet)

```bash
# 1. 검증 비행 (자동 모든 단계)
~/ros2_ws/src/trajectory_prediction/scripts/launch_1vtol_replay.sh
#   ↓ 약 55초 후 시퀀스 끝나면 Ctrl+C

# 2. 사후분석
mkdir -p /tmp/tc_synced
/home/leedonghyuck/anaconda3/bin/python3 \
  ~/ros2_ws/src/trajectory_prediction/scripts/chunk_analysis.py \
  --out-dir /tmp/tc_synced/

# 3. 비교
ls /tmp/horizontal/chunk_error_curves.png  # 이전 (tc_alt=2.0)
ls /tmp/tc_synced/chunk_error_curves.png   # 새 (tc_alt=5.0)
# psi end / XY end 가 더 작아졌는지 확인
```
