# 프로젝트 파일 다이어그램 — `ros2_ws/`

> **컨텍스트**: PX4 + ROS2 기반 고정익 swarm 비행 / 궤적 예측 프로젝트의 전체 파일 구조를 한눈에. 각 파일이 무엇을 하고 어떻게 연결되는지 다이어그램.

---

## 1. 워크스페이스 최상위

```
/home/leedonghyuck/ros2_ws/
├── src/                              ← ROS2 패키지 소스 (★ 작업 영역 ★)
│   ├── collision_avoidance/          ← [우리 코드] 5대 swarm formation
│   ├── trajectory_prediction/        ← [우리 코드] sequence replay + RK4 예측
│   ├── px4_msgs/                     ← [외부] PX4 ROS2 메시지 정의 (msg files)
│   ├── px4_ros_com/                  ← [외부] PX4 ↔ ROS2 bridge (uXRCE-DDS 헬퍼)
│   ├── px4-ros2-interface-lib/       ← [외부] px4_ros2 라이브러리 (ModeBase, FwSetpoint 등)
│   ├── example/                      ← [외부] 샘플 코드
│   ├── how_to_know_coordinate.txt    ← 좌표계 메모
│   └── how_to_use.txt                ← 사용법 메모
│
├── build/                            ← colcon build 산출물 (auto-generated, 무시)
├── install/                          ← colcon 설치 결과 (source install/setup.bash 대상)
├── log/                              ← colcon 빌드/실행 로그
├── docker/                           ← Docker 환경 설정
│
├── FORMATION_MODE_ARCHITECTURE.md    ← [문서] FormationMode 전체 구조 (이 디렉토리)
├── SUBSCRIBER_DATA_FLOW.md           ← [문서] subscriber → rt_thread 변환 사슬
├── PROJECT_FILE_DIAGRAM.md           ← [문서] 이 파일
├── migration_guide_v2.md             ← [문서] 마이그레이션 가이드 (v1 → v2)
└── terminal.txt                      ← 터미널 명령어 메모
```

---

## 2. 패키지 1: `collision_avoidance` — 5대 swarm formation flight

### 빌드 타겟 (CMakeLists.txt)

| 실행 파일 | 역할 | 실행 위치 |
|---|---|---|
| `vtol_guidance_node` | mode 등록 + Flocking 가이던스 + setpoint publish | **라즈베리파이 (각 기체)** |
| `coordinate_transformer_node` | raw odometry → 통일 원점 좌표 변환 + republish | **로컬 PC** |

### 디렉토리 구조

```
src/collision_avoidance/
│
├── CMakeLists.txt                    ← 빌드 설정 (위 2개 실행파일 정의)
├── package.xml                       ← ROS2 패키지 메타데이터·의존성 선언
│
├── include/collision_avoidance/      ← 헤더 (.hpp)
│   │
│   ├── ── PX4 mode 정의 ──
│   ├── VtolPreflightMode.hpp          → 이륙 + VTOL→FW 전이 + cruise 안정화 mode
│   ├── FormationMode.hpp              → ★ 편대 비행 mode (사용자 연구 핵심) ★
│   ├── VtolGuidanceExecutor.hpp       → preflight → formation 자동 전환 state machine
│   │
│   ├── ── 가이던스 알고리즘 ──
│   ├── FlockingGuidance.hpp           → Olfati-Saber flocking + setpoint 변환
│   ├── AirframeLimits.hpp             → TECS 에너지 기반 dV/dt 제약 계산
│   │
│   ├── ── 좌표 변환 ──
│   ├── TransferSameCoordinate.hpp     → 5대 GPS origin 통일 (PC 측 노드)
│   │
│   ├── ── 자료형 + 유틸 ──
│   ├── StateType.hpp                  → 모든 통신용 struct + enum + queue typedef
│   └── spsc_queue.hpp                 → Lock-free Single Producer Single Consumer 큐
│
├── src/                              ← 구현 (.cpp)
│   │
│   ├── ── vtol_guidance_node 의 진입점 ──
│   ├── main.cpp                       → Node 생성 + Mode 인스턴스화 + doRegister + spin
│   │
│   ├── ── PX4 mode 구현 ──
│   ├── VtolPreflightMode.cpp          → VtolPreflightMode.hpp 구현
│   ├── FormationMode.cpp              → FormationMode.hpp 구현 (rt_loop 포함)
│   ├── VtolGuidanceExecutor.cpp       → VtolGuidanceExecutor.hpp 구현
│   │
│   ├── ── 가이던스 ──
│   ├── FlockingGuidance.cpp           → FlockingGuidance.hpp 구현 (computeFwSetpoint)
│   │
│   ├── ── coordinate_transformer_node 의 진입점 ──
│   ├── transformer_main.cpp           → 좌표 변환 노드 main
│   └── TransferSameCoordinate.cpp     → 좌표 변환 알고리즘 구현
│
├── config/                           ← yaml 설정 파일
│   ├── airframe_spec.yaml             → PX4 airframe 한계값 (FW_AIRSPD_*, FW_R_LIM 등)
│   ├── flocking_params.yaml           → Flocking 가이던스 게인 (lambda, beta, k1, k2)
│   ├── ros_params.yaml                → vehicle_ID, total_agent_num
│   └── spawn_config.yaml              → 5대 spawn 위치 (Gazebo)
│
└── scripts/                          ← 보조 스크립트
    ├── convert_px4_params.py          → QGC export → yaml 변환기
    └── launch_5vtol.sh                → 5대 SITL + ROS2 일괄 기동
```

### 클래스 의존 관계 (collision_avoidance)

```
┌─────────────────────────────────────────────────────────────┐
│ main.cpp                                                     │
│  └─ rclcpp::Node                                             │
│      ├─ VtolPreflightMode  ────┐                             │
│      ├─ FormationMode      ─┐  │                             │
│      └─ VtolGuidanceExecutor│  │                             │
│           ├─ owns preflight ◄──┘ (전이 mode)                 │
│           └─ ref to formation ◄─┘ (편대 mode)                │
└─────────────────────────────────────────────────────────────┘
        │                   │
        │                   │
        ▼                   ▼
┌────────────────────────────────────────────────────┐
│ FormationMode (rt_thread + queue 보유)              │
│  ├─ FlockingGuidance       ── 가이던스 알고리즘     │
│  │   └─ AirframeLimits     ── 한계값 계산           │
│  ├─ SpscQueue<...> × 2     ── input + output 채널   │
│  └─ StateType::*           ── 통신용 struct         │
└────────────────────────────────────────────────────┘
```

```
┌─────────────────────────────────────┐
│ transformer_main.cpp                 │
│  └─ TransferSameCoordinate           │
│      └─ 좌표 변환 → republish         │
└─────────────────────────────────────┘
```

---

## 3. 패키지 2: `trajectory_prediction` — sequence replay + RK4 예측

### 빌드 타겟

| 실행 파일 | 역할 | 실행 위치 |
|---|---|---|
| `trajectory_replay_node` | yaml 의 setpoint 시퀀스 재생 + 비행 결과 CSV + RK4 예측 | **로컬 PC** (현재) → **라즈베리파이 라이브러리화 예정** |

### 디렉토리 구조

```
src/trajectory_prediction/
│
├── CMakeLists.txt                    ← 빌드 설정
├── package.xml                       ← 패키지 메타데이터
│
├── include/trajectory_prediction/    ← 헤더 (.hpp)
│   │
│   ├── ── Mode 정의 (collision_avoidance 와 같은 패턴) ──
│   ├── VtolPreflightMode.hpp          → 이륙 + 전이 (같은 코드 재사용)
│   ├── TrajectoryReplayMode.hpp       → ★ setpoint 시퀀스 재생 mode ★
│   ├── VtolGuidanceExecutor.hpp       → preflight → replay 전환 state machine
│   │
│   ├── ── Replay / Prediction 핵심 ──
│   ├── SetpointSequencer.hpp          → yaml 시퀀스 파싱 + 시간순 재생
│   ├── TrajectoryLogger.hpp           → state + setpoint + predict 결과 → CSV
│   ├── TrajectoryPredict.hpp          → ★ RK4 forward integration 예측기 ★
│   │
│   ├── ── 자료형 + 유틸 (collision_avoidance 와 동일 패턴) ──
│   ├── StateType.hpp                  → 통신용 struct
│   └── spsc_queue.hpp                 → Lock-free 큐 (재사용)
│
├── src/                              ← 구현 (.cpp)
│   ├── main.cpp                       → Node + Mode 인스턴스 + wall_timer 등록
│   ├── VtolPreflightMode.cpp
│   ├── TrajectoryReplayMode.cpp
│   ├── VtolGuidanceExecutor.cpp
│   ├── SetpointSequencer.cpp
│   ├── TrajectoryLogger.cpp
│   └── TrajectoryPredict.cpp
│
└── config/
    ├── airframe_spec.yaml             → 항공기 spec (collision_avoidance 와 동일)
    ├── replay_params.yaml             → CSV 경로, log rate, predict 파라미터
    └── setpoint_sequence.yaml         → 재생할 setpoint 시퀀스 정의
```

### 클래스 의존 관계 (trajectory_prediction)

```
┌─────────────────────────────────────────────────────────────┐
│ main.cpp                                                     │
│  └─ rclcpp::Node                                             │
│      ├─ VtolPreflightMode                                    │
│      ├─ TrajectoryReplayMode  ───┐                           │
│      ├─ VtolGuidanceExecutor                                 │
│      ├─ TrajectoryLogger      ─┐ │                           │
│      ├─ TrajectoryPredictor  ──┼─┤                           │
│      └─ wall_timer (10Hz)  ────┘ │                           │
│           └─ predictor->predict(logger snapshot)             │
└─────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌────────────────────────────────────────────────────┐
│ TrajectoryReplayMode                                │
│  ├─ SetpointSequencer    ── yaml 시퀀스 재생        │
│  └─ updateSetpoint()     ── _fw_setpoint->update    │
└────────────────────────────────────────────────────┘
```

---

## 4. 외부 의존 패키지 (`src/` 안에 있지만 우리 코드 아님)

```
src/
├── px4_msgs/                         ← PX4 의 모든 ROS2 메시지 정의
│   └── msg/
│       ├── VehicleOdometry.msg       (위치·속도)
│       ├── VehicleAttitude.msg       (자세)
│       ├── VtolVehicleStatus.msg     (VTOL 상태)
│       ├── Wind.msg                  (바람)
│       ├── TrajectorySetpoint.msg    (MC setpoint)
│       └── ... (수백 개)
│
├── px4_ros_com/                      ← PX4 ↔ ROS2 bridge 유틸
│   └── (uXRCE-DDS 통신 헬퍼)
│
└── px4-ros2-interface-lib/           ← ModeBase / FwSetpoint 등 wrapper
    └── px4_ros2_cpp/
        └── (우리가 import 하는 lib)
```

→ **우리는 이들을 의존만 함** (수정하지 않음). 의존 선언은 각 패키지의 `package.xml` 에서.

---

## 5. 두 패키지의 관계 — 같은 패턴의 변주

| 항목 | collision_avoidance | trajectory_prediction |
|---|---|---|
| **목적** | 5대 swarm 자율 편대비행 | 1대 setpoint 시퀀스 재생 + 예측 모델 검증 |
| **기체 수** | N 대 (yaml: total_agent_num) | 1 대 |
| **mode 종류** | Preflight + Formation | Preflight + Replay |
| **Executor** | preflight → formation | preflight → replay |
| **rt_thread** | 있음 (Flocking 계산) | 없음 (단순 시퀀스 재생) |
| **wall_timer** | 없음 | 있음 (predict 10Hz) |
| **CSV 로깅** | 없음 | 있음 (TrajectoryLogger) |
| **재사용된 코드** | — | StateType, spsc_queue, VtolPreflightMode 패턴 |

→ **trajectory_prediction 은 collision_avoidance 의 인프라 (mode + queue 패턴) 를 재사용**해서 만든 검증/실험용 패키지.

---

## 6. 데이터 흐름 — 시스템 전체 (테스트 환경 기준)

```
┌─────────────────────────── 로컬 PC ──────────────────────────┐
│                                                              │
│  ┌──────────────┐    ┌──────────────────────────┐           │
│  │ PX4 SITL × N │    │ Gazebo (Standard VTOL × N)│           │
│  └──────┬───────┘    └────────────────────────┬─┘           │
│         │                                      │             │
│         │  /px4_n/fmu/out/vehicle_odometry     │             │
│         │  (raw, 각 기체 origin)                │             │
│         ▼                                      │             │
│  ┌──────────────────────────────┐              │             │
│  │ coordinate_transformer_node  │              │             │
│  │  (TransferSameCoordinate)    │              │             │
│  └──────┬───────────────────────┘              │             │
│         │                                      │             │
│         │  /common/px4_n/trans_vehicle_odometry│             │
│         │  (통일 원점 좌표)                     │             │
│         ▼                                      ▼             │
│  ┌────────────────────────────────────────────────────┐     │
│  │   QGroundControl (mode switch / 모니터링)           │     │
│  └────────────────────────────────────────────────────┘     │
└──────────────────┬───────────────────────────────────────────┘
                   │
                   │ uXRCE-DDS / DDS
                   ▼
┌─────────────────────────── 라즈베리파이 (각 기체별) ──────────────┐
│                                                                  │
│  ┌──────────────────────────────────────────────────┐            │
│  │ vtol_guidance_node (collision_avoidance)         │            │
│  │  ├─ subscribe /common/px4_n/trans_vehicle_odom   │            │
│  │  ├─ FormationMode.rt_loop (Flocking 계산)         │            │
│  │  └─ _fw_setpoint->update(sp) ─────────────┐      │            │
│  └────────────────────────────────────────────┼──────┘            │
│                                                │                  │
│         ▲ (predict 추가시 같은 노드 안에)        │                  │
│  ┌──────┴──────────────────────────────┐     │                  │
│  │ TrajectoryPredictor (라이브러리화)    │     │                  │
│  │  └─ 10Hz wall_timer 로 predict       │     │                  │
│  └──────────────────────────────────────┘     │                  │
└────────────────────────────────────────────────┼─────────────────┘
                                                 │
                                                 │ uXRCE-DDS
                                                 ▼
                                  [PX4 commander 가 setpoint 인가]
                                                 │
                                                 ▼
                                       [PX4 fixed-wing controller]
                                                 │
                                                 ▼
                                          [모터·서보 제어]
```

---

## 7. 빠른 참고 — "어디서 무엇을 보나"

| 질문 | 보면 되는 파일 |
|---|---|
| "subscribe 어디서 등록?" | `*Mode.cpp` 의 생성자 (예: `FormationMode.cpp:21-70`) |
| "publish 어디서?" | `_fw_setpoint->update(...)` 호출 — `updateSetpoint(dt)` 안 |
| "rt_thread 시작 어디?" | `*Mode.cpp` 생성자 마지막 (`FormationMode.cpp:155-156`) |
| "rt_thread 본문?" | `*Mode::rt_loop()` (`FormationMode.cpp:266-359`) |
| "큐 자료형?" | `StateType.hpp` (`InputQueue_mt2rt`, `OutputQueue_rt2mt`) |
| "큐 구현체?" | `spsc_queue.hpp` (전체 30 줄) |
| "가이던스 알고리즘?" | `FlockingGuidance.cpp` (`computeFwSetpoint`) |
| "yaml 파라미터 정의?" | `config/*.yaml` (airframe, flocking, replay) |
| "main 진입점?" | `src/main.cpp` 또는 `src/transformer_main.cpp` |
| "ModeBase 인터페이스?" | `src/px4-ros2-interface-lib/px4_ros2_cpp/...` |
| "PX4 메시지 타입?" | `src/px4_msgs/msg/*.msg` |
| "RK4 예측 본체?" | `trajectory_prediction/src/TrajectoryPredict.cpp` |
| "CSV 로깅 본체?" | `trajectory_prediction/src/TrajectoryLogger.cpp` |
| "5대 SITL 기동 스크립트?" | `collision_avoidance/scripts/launch_5vtol.sh` |

---

## 8. 빌드 / 실행 cheat sheet

```bash
# 빌드 (워크스페이스 루트에서)
cd /home/leedonghyuck/ros2_ws
colcon build --packages-select collision_avoidance trajectory_prediction
source install/setup.bash

# collision_avoidance 실행
ros2 run collision_avoidance coordinate_transformer_node \
    --ros-args --params-file src/collision_avoidance/config/spawn_config.yaml

ros2 run collision_avoidance vtol_guidance_node \
    --ros-args --params-file src/collision_avoidance/config/airframe_spec.yaml \
               --params-file src/collision_avoidance/config/flocking_params.yaml \
               --params-file src/collision_avoidance/config/ros_params.yaml

# trajectory_prediction 실행
ros2 run trajectory_prediction trajectory_replay_node \
    --ros-args --params-file src/trajectory_prediction/config/replay_params.yaml \
               --params-file src/trajectory_prediction/config/airframe_spec.yaml \
               --params-file src/trajectory_prediction/config/setpoint_sequence.yaml

# 5대 SITL 일괄 기동
./src/collision_avoidance/scripts/launch_5vtol.sh
```

---

## 9. 관련 문서

같은 디렉토리 (`/home/leedonghyuck/ros2_ws/`) 의 자매 문서:

- **`FORMATION_MODE_ARCHITECTURE.md`** — FormationMode 의 rt_thread + queue + ModeBase 통합 구조
- **`SUBSCRIBER_DATA_FLOW.md`** — subscriber 데이터가 멤버 변수에 저장되고 rt_thread 로 전달되는 변환 사슬
- **`migration_guide_v2.md`** — v1 → v2 마이그레이션 가이드 (별도 문서)
