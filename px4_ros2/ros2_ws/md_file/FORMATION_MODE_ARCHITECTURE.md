# FormationMode 구조 설명 — rt_thread + lock-free queue + ModeBase 통합

> **컨텍스트**: 기존 코드의 실시간 스레드 + lock-free 큐 + Formation 모듈 구조를 유지하면서, ModeBase 로 옮긴 형태. 핵심은: 서브스크라이버·스레드·큐는 그대로 두고 setpoint 발행만 `updateSetpoint()` 에서 `_fw_setpoint->update(sp)` 로 대체.

코드 위치:
- 헤더: `/home/leedonghyuck/ros2_ws/src/collision_avoidance/include/collision_avoidance/FormationMode.hpp`
- 구현: `/home/leedonghyuck/ros2_ws/src/collision_avoidance/src/FormationMode.cpp`
- 가이던스: `/home/leedonghyuck/ros2_ws/src/collision_avoidance/src/FlockingGuidance.cpp`
- 자료형: `/home/leedonghyuck/ros2_ws/src/collision_avoidance/include/collision_avoidance/StateType.hpp`

---

## 전체 구조 한눈에

```
┌─────────────────── ROS2 토픽 입력 (PC 의 TransferSameCoordinate 가 publish) ───────────────────┐
│                                                                                                │
│   /common/px4_0/trans_vehicle_odometry  /common/px4_1/...  ...  (각 기체별 N개)               │
│                              ↓                                                                 │
│ ┌───────────────────────── main thread (rclcpp::spin) ──────────────────────────────┐         │
│ │  FormationMode.cpp:31  _trans_odom_subs[n] 콜백                                    │         │
│ │     - m_state_for_control_mt[n] 채움                                                │         │
│ │     - m_agent_updated_mt[n] = true                                                  │         │
│ │     - 5 대 전부 도착? YES → snapshot 만들기 → m_input_queue_mt2rt.try_push()        │         │
│ │                                                                                     │         │
│ │  FormationMode.cpp:23  _wind_sub 콜백                                               │         │
│ │     - m_wind_n_mt2rt.store(...)   ← atomic                                          │         │
│ │     - m_wind_e_mt2rt.store(...)                                                     │         │
│ └────────────────────────────────────┬────────────────────────────────────────────────┘         │
│                                      │                                                          │
│                                      ▼                                                          │
│             ┌─────────────────────────────────────────────┐                                    │
│             │   m_input_queue_mt2rt   (SPSC Lock-Free)    │   ← StateType.hpp:62               │
│             │   element = Total_state_for_Control_mt2rt    │                                    │
│             └────────────────────┬───────────────────────┘                                     │
│                                  │  try_pop()                                                   │
│                                  ▼                                                              │
│ ┌─────────────────────────── rt_thread (별도 std::thread) ──────────────────────────┐          │
│ │  FormationMode::rt_loop (cpp:266)        주기: ~1ms (sleep_for(1ms))               │          │
│ │     1. m_input_queue_mt2rt.try_pop()      ← swarm snapshot                         │          │
│ │     2. self / others 추출 → AgentState_rt                                          │          │
│ │     3. m_wind_n_mt2rt.load(), wind_e.load()  ← atomic                              │          │
│ │     4. m_flocking->computeFwSetpoint(...)    ← 모든 무거운 계산                     │          │
│ │        (Flocking 가속도 + 적분 + saturation + 변환)                                  │          │
│ │     5. m_output_queue_rt2mt.try_push(out)                                          │          │
│ └────────────────────────────────────┬────────────────────────────────────────────────┘         │
│                                      │                                                          │
│                                      ▼                                                          │
│             ┌─────────────────────────────────────────────┐                                    │
│             │   m_output_queue_rt2mt  (SPSC Lock-Free)    │   ← StateType.hpp:102              │
│             │   element = FwSetpointOutput_rt2mt           │                                    │
│             └────────────────────┬───────────────────────┘                                     │
│                                  │  try_pop()                                                   │
│                                  ▼                                                              │
│ ┌─────────────────────── main thread — px4_ros2 가 호출 ────────────────────────────┐          │
│ │  FormationMode::updateSetpoint(dt)  (cpp:196)   주기: 30Hz (ModeBase 가 호출)      │          │
│ │     1. m_output_queue_rt2mt.try_pop()                                              │          │
│ │     2. 성공 → m_last_output_mt 갱신, m_has_last_output_mt=true                      │          │
│ │     3. 실패 + 이전 값 있음 → hold last (ZOH)                                        │          │
│ │     4. 실패 + 이전 값 없음 → cruise fallback                                        │          │
│ │     5. _fw_setpoint->update(sp)  ★ 여기가 PX4 로 진짜 publish 되는 지점 ★         │          │
│ └────────────────────────────────────┬────────────────────────────────────────────────┘         │
│                                      │                                                          │
│                                      ▼ uXRCE-DDS                                                │
│                         [PX4 의 fixed-wing 컨트롤러]                                            │
└────────────────────────────────────────────────────────────────────────────────────────────────┘
```

---

## 통신 채널 4 종류 — 모두 `StateType.hpp` 의 네이밍 컨벤션

코드에서 어떤 변수가 어떤 통신을 하는지 접미사로 알 수 있게 설계됨:

| 접미사 | 의미 | 자료구조 | 예시 |
|---|---|---|---|
| `_mt2rt` | main → rt 단방향 | atomic 또는 SPSC queue | `m_wind_n_mt2rt` (atomic), `m_input_queue_mt2rt` (queue) |
| `_rt2mt` | rt → main 단방향 | SPSC queue | `m_output_queue_rt2mt` |
| `_mt`    | main 전용 | (스레드 안전 불필요) | `m_state_for_control_mt`, `m_last_output_mt` |
| `_rt`    | rt 전용 | (스레드 안전 불필요) | `m_others_buf_rt` |

이 컨벤션 덕에 **race condition 위험을 변수명만 봐도 알 수 있음**. 만약 `m_wind_n_mt` 였으면 atomic 안 써도 되고, `m_wind_n_mt2rt` 면 atomic 또는 큐 필수.

---

## 왜 이렇게 두 스레드 + 두 큐 인가?

### (1) main thread = ROS2 / PX4 콜백 처리만 (가벼움)

- `rclcpp::spin` 이 돌리는 콜백 (subscription, timer) — 빠르게 끝나야 함
- `updateSetpoint(dt)` 도 main thread — PX4 가 30Hz 데드라인으로 호출
- 여기서 무거운 계산하면 → spin 지연 → 토픽 buffer 쌓임 → 시스템 죽음

### (2) rt_thread = Flocking 계산 (무거움)

- 5 대 위치/속도 받아서 가속도 계산 + RK 적분 + saturation + 변환 + ...
- 이걸 spin 안에 끼우면 jitter 위험. 별도 스레드로 격리.
- "rt" 라고 이름 붙었지만 실제로는 RT 스케줄링 안 됨 — 단지 **별도 스레드** 라는 뜻

### (3) SPSC Lock-Free Queue 가 둘을 이어줌

- **Single Producer Single Consumer** = 한쪽이 push, 한쪽이 pop 만 함
- Mutex 없음 (atomic 만 사용) → 락 대기 0
- 둘 사이 jitter 가 다른 쪽에 전파 안 됨

### (4) Atomic 은 **단일 스칼라값** 에만 사용

- `m_wind_n_mt2rt` 처럼 float 한 개 → atomic 으로 충분
- snapshot 같은 큰 구조체는 atomic 못 씀 → SPSC queue 필요

---

## ModeBase 적용으로 인한 변화 — **pipeline 의 끝부분만**

| 위치 | Pre-ModeBase | After ModeBase | 변화 |
|---|---|---|---|
| Subscription (입력) | `_node.create_subscription<...>(...)` | `_node.create_subscription<...>(...)` | **그대로** |
| rt_thread (계산) | `m_flocking->computeFwSetpoint()` | `m_flocking->computeFwSetpoint()` | **그대로** |
| Queue (mt↔rt 통신) | `SpscQueue<>` | `SpscQueue<>` | **그대로** |
| Publisher (출력) | `_setpoint_pub->publish(...)` | `_fw_setpoint->update(sp)` | **★ 변경 ★** |
| 주기적 호출 메커니즘 | `rclcpp::Timer` 직접 등록 (30Hz) | `ModeBase::updateSetpoint(dt)` 자동 호출 | **★ 변경 ★** |
| Mode 활성화 | mode switch 직접 처리 | `onActivate()` / `onDeactivate()` 콜백 | **★ 변경 ★** |

→ **요지**: Producer-Consumer 패턴 (sub → queue → rt → queue → 출력) 의 **출력 쪽만** ModeBase 의 wrapper 로 바뀜. 내부 알고리즘과 스레딩은 손대지 않음.

---

## 라이프사이클 — 언제 무엇이 시작되나

```
[main() 시작]
    ↓
FormationMode 생성자 호출:
    - _wind_sub, _trans_odom_subs 생성  (ROS2 sub 들 등록)
    - m_flocking 생성 (FlockingGuidance 인스턴스)
    - m_rt_running_mt2rt = true
    - m_rt_thread_mt = std::thread(&rt_loop, this)   ← rt_thread 즉시 시작 (mode 비활성 상태에서도 돔)
    ↓
rclcpp::spin(node)  ← main thread 가 콜백 처리 시작
    ↓
QGC 에서 mode switch → "VTOL Formation" 선택
    ↓
ModeBase → FormationMode::onActivate() 호출:
    - m_reinit_vdesired_mt2rt = true   ← rt_thread 한테 "초기화 신호"
    - m_has_last_output_mt = false      ← hold-last 버퍼 리셋
    ↓
ModeBase → FormationMode::updateSetpoint(dt) 30Hz 호출 시작
    - rt_thread 가 push 한 값을 pop 해서 _fw_setpoint->update(sp)
    ↓
QGC 에서 mode switch → 다른 모드 선택
    ↓
ModeBase → FormationMode::onDeactivate() 호출:
    - 로그만 남김. rt_thread 는 안 멈춤! (재활성화 시 워밍업 0)
    ↓
[node 종료]
    ↓
FormationMode 소멸자:
    - m_rt_running_mt2rt = false
    - m_rt_thread_mt.join()   ← rt_thread 완전 종료 대기
```

핵심 설계 결정 — **rt_thread 는 `onActivate()` 가 아니라 생성자에서 시작**.
이유: mode 가 매번 활성/비활성 될 때마다 thread 만들고 죽이면 워밍업 갭 + 큐 잔존 버그 위험. 항상 돌고 있다가, 활성화 신호만 atomic flag 로 보냄.

---

## trajectory_prediction 통합 시 — 이 패턴을 따라야 함

```
[기존 FormationMode 패턴]              [너의 onboard mode 에서 추가]
┌─────────────────────────┐            ┌─────────────────────────────┐
│ ROS2 sub callback       │            │ predict_timer (10Hz, main)  │
│ ↓                       │            │ ↓                           │
│ m_input_queue_mt2rt     │            │ logger->getCurrentSnapshot()│
│ ↓                       │            │ ↓                           │
│ rt_loop (rt_thread)     │            │ predictor->predict(x0,sp)   │
│ ↓                       │            │ ↓                           │
│ m_output_queue_rt2mt    │            │ logger->pushPredictedTraj() │
│ ↓                       │            │                             │
│ updateSetpoint (30Hz)   │            │                             │
│ ↓                       │            │                             │
│ _fw_setpoint->update    │            │                             │
└─────────────────────────┘            └─────────────────────────────┘
```

trajectory_prediction 는 **별도 wall_timer** 로 가는 거라 위 파이프라인의 어느 큐에도 끼지 않음. 같은 ModeBase 안에 추가 멤버로 `m_predictor` 와 `predict_timer` 를 두면 됨.

---

# 부록 — 어디서 무엇을 어떻게 생성했는지 코드 분석

> 위 다이어그램의 각 박스가 **실제 파일의 어느 줄에서 어떻게 생성되는지** 줄 단위로 추적.

## A. main.cpp — 전체 부팅 시퀀스

파일: `/home/leedonghyuck/ros2_ws/src/collision_avoidance/src/main.cpp`

```cpp
int main(int argc, char * argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);     // [a] stdout buffering 끔 (실시간 로그)
    rclcpp::init(argc, argv);                  // [b] ROS2 시스템 초기화

    /* [1] Node 생성 */
    auto node = std::make_shared<rclcpp::Node>("vtol_guidance_node");

    /* [2] 파라미터 선언 및 할당 */
    node->declare_parameter<int>("vehicle_ID", 0);
    node->declare_parameter<int>("total_agent_num", 0);
    int vehicle_ID      = node->get_parameter("vehicle_ID").as_int();
    int total_agent_num = node->get_parameter("total_agent_num").as_int();

    /* [3] 두 개의 Mode 인스턴스 생성 */
    auto preflight = std::make_shared<VtolPreflightMode>(*node, vehicle_ID, total_agent_num);
    auto formation = std::make_shared<FormationMode>(*node, vehicle_ID, total_agent_num);

    /* [4] Executor 생성 (owned_mode = preflight, second_mode = formation) */
    auto executor = std::make_shared<VtolGuidanceExecutor>(*preflight, *formation);

    /* [5] PX4 에 등록 */
    executor->doRegister();
    formation->doRegister();

    /* [6] 실행 */
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### 주의 깊게 볼 것

| 라인 | 의미 |
|---|---|
| `*node` (call-by-reference) | mode 들의 생성자에 Node 참조를 넘김. Node 의 owner 는 main(), mode 들은 사용자일 뿐 |
| `make_shared<...>` | mode 들도 shared_ptr 로 관리 → executor 와 main() 이 같이 소유 가능 |
| `executor->doRegister()` | "이 executor 와 그것이 소유한 preflight mode 를 PX4 에 등록" |
| `formation->doRegister()` | formation 은 executor 가 소유하지 않으므로 따로 등록 필요 |
| `rclcpp::spin(node)` | main thread 가 여기서 블록 → 모든 sub callback / timer / updateSetpoint 가 여기서 분기됨 |

→ **그 아래로 모든 thread 와 queue 가 만들어지는 진입점.** 노드는 단 1 개, mode 는 2 개.

---

## B. FormationMode 생성자 — 실제로 무엇을 만드는가?

파일: `FormationMode.cpp:6-160`

생성자는 **5 개 단계** 로 나뉨:

### B-1. 부모 클래스 초기화 (`ModeBase`)

```cpp
FormationMode::FormationMode(rclcpp::Node & node, int vehicle_id, int total_agent_num)
: ModeBase(node, Settings{"VTOL Formation"},
           "/px4_" + std::to_string(vehicle_id) + "/")     // namespace
, _node(node)                                              // 참조 멤버 초기화
{
    m_vehicle_id      = vehicle_id;
    m_total_agent_num = total_agent_num;
```

- `ModeBase(node, Settings{...}, "/px4_X/")`: 부모 클래스 (px4_ros2 라이브러리) 가 mode 등록에 필요한 식별자 + topic namespace 설정
- `: _node(node)` ← **참조 멤버는 반드시 초기화 리스트에서 초기화** (대입 불가). 이게 `rclcpp::Node & _node` 가 헤더에서 reference 타입인 이유 중 하나

### B-2. px4_ros2 wrapper 생성 (publisher 역할)

```cpp
    _fw_setpoint = std::make_shared<px4_ros2::FwLateralLongitudinalSetpointType>(*this);
    _vtol_status = std::make_shared<px4_ros2::VtolStatus>(*this);
```

- `*this` ← 자기 자신 (FormationMode = ModeBase 서브클래스) 을 wrapper 에 넘김
- wrapper 가 `*this` 로부터 **PX4 의 setpoint topic** 을 자동으로 advertise/subscribe 함
- 사용자는 `_fw_setpoint->update(sp)` 만 호출하면 됨 — topic 이름·QoS·timestamp 다 wrapper 가 알아서

### B-3. 표준 ROS2 Subscription 생성 — **여기서 callback 등록**

```cpp
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    /* (1) 바람 구독 — atomic 으로 mt → rt */
    _wind_sub = _node.create_subscription<Wind>(
        "/px4_" + std::to_string(m_vehicle_id) + "/fmu/out/wind", qos,
        [this](const Wind::UniquePtr msg) {                      // ← 람다 callback
            m_wind_n_mt2rt.store(msg->windspeed_north, std::memory_order_relaxed);
            m_wind_e_mt2rt.store(msg->windspeed_east,  std::memory_order_relaxed);
        });

    /* (2) 5 대 odometry 구독 — vector 에 N 개 sub 동적 생성 */
    _trans_odom_subs.resize(m_total_agent_num);
    for (int n = 0; n < m_total_agent_num; n++) {
        _trans_odom_subs[n] = _node.create_subscription<VehicleOdometry>(
            "/common/px4_" + std::to_string(n) + "/trans_vehicle_odometry",
            qos,
            [this, n](const VehicleOdometry::UniquePtr msg) {    // ← capture: this + n
                m_state_for_control_mt[n].position = msg->position;
                m_state_for_control_mt[n].velocity = msg->velocity;
                /* ... 변량 + 시각 채움 ... */
                m_agent_updated_mt[n] = true;

                /* 5 대 전부 도착 체크 — snapshot 만들고 큐에 push */
                bool all_updated = true;
                for (int i = 0; i < m_total_agent_num; i++) {
                    if (!m_agent_updated_mt[i]) { all_updated = false; break; }
                }
                if (all_updated) {
                    StateType::Total_state_for_Control_mt2rt snapshot{};
                    snapshot.num_agents = m_total_agent_num;
                    snapshot.agents     = m_state_for_control_mt;
                    m_input_queue_mt2rt.try_push(snapshot);   // ★ rt thread 로 넘김 ★
                    m_agent_updated_mt.fill(false);
                }
            });
    }
```

핵심 패턴:
- **람다 캡처 `[this]`**: callback 안에서 멤버 변수 (`m_wind_n_mt2rt` 등) 에 접근하기 위해 `this` 포인터를 캡처
- **람다 캡처 `[this, n]`**: 추가로 loop 변수 `n` 도 캡처 → 각 sub callback 이 자기 인덱스를 기억
- **`atomic.store(value, memory_order_relaxed)`**: 짧은 단일값은 atomic 만으로 OK, ordering 신경 안 써도 됨
- **All-arrived 패턴**: N 대 모두 도착해야 snapshot push → "swarm 동시 시점" 보장
- **`m_agent_updated_mt.fill(false)`**: 새 사이클을 위해 플래그 리셋

### B-4. yaml 파라미터 로딩 + Flocking 모듈 생성

```cpp
    auto declare_or_get = [&](const std::string& name, float default_val) -> float {
        if (!_node.has_parameter(name)) {
            _node.declare_parameter<float>(name, default_val);
        }
        return static_cast<float>(_node.get_parameter(name).as_double());
    };

    FlockingGuidance::Parameters flocking_params;
    flocking_params.lambda           = declare_or_get("flocking_lambda", 1.0f);
    flocking_params.beta             = declare_or_get("flocking_beta", 0.5f);
    /* ... 더 많은 파라미터 ... */

    AirframeLimits airframe_limits;
    airframe_limits.max_climb_rate = declare_or_get("height_rate_max_climb", 5.0f);
    /* ... */

    m_flocking = std::make_unique<FlockingGuidance>(flocking_params, airframe_limits);
```

- **`declare_or_get` 람다**: yaml 에서 파라미터를 가져오거나 기본값으로 선언. `has_parameter` 체크로 중복 선언 방지
- **`std::unique_ptr<FlockingGuidance>`**: Flocking 모듈은 FormationMode 가 단독 소유. shared 가 아니라 unique
- 이 시점에 yaml 파일은 launch file 의 `--params-file` 인자로 이미 ROS2 파라미터 서버에 로딩됨

### B-5. rt_thread 시작

```cpp
    m_rt_running_mt2rt.store(true);                                    // ★ 시작 신호
    m_rt_thread_mt = std::thread(&FormationMode::rt_loop, this);       // ★ thread spawn
}
```

- `std::thread(member_function_ptr, instance)`: 멤버 함수를 새 스레드로 시작하는 표준 C++ 패턴
- `&FormationMode::rt_loop`: 멤버 함수 포인터
- `this`: 어떤 인스턴스의 메서드인지 지정
- → 이 줄 다음부터 `rt_loop` 가 별도 스레드에서 무한 루프 시작

→ **즉 생성자 끝나면 이미 rt_thread 는 돌고 있음**. mode 활성화 여부와 무관.

---

## C. rt_loop 구현 — 한 사이클의 안

파일: `FormationMode.cpp:266-359`

```cpp
void FormationMode::rt_loop()
{
    bool first_push_done = false;

    while (m_rt_running_mt2rt.load()) {           // ← 종료 신호 체크
        /* (0) 활성화 직후 재초기화 신호 처리 */
        if (m_reinit_vdesired_mt2rt.exchange(false)) {
            first_push_done = false;
        }

        /* (1) 입력 큐에서 swarm snapshot 받기 */
        std::optional<StateType::Total_state_for_Control_mt2rt> input_state =
            m_input_queue_mt2rt.try_pop();

        if (!input_state.has_value() || !m_flocking) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        /* (2) self / others 추출 */
        const auto & snapshot = input_state.value();
        const int self_idx = m_vehicle_id;

        StateType::AgentState_rt self;
        self.pos_n = snapshot.agents[self_idx].position[0];
        /* ... pos_e, pos_d, vel_n, vel_e, vel_d ... */
        self.speed = std::sqrt(self.vel_n*self.vel_n + ...);
        self.psi   = std::atan2(self.vel_e, self.vel_n);   // ← 코스각
        self.gamma = std::atan2(-self.vel_d, v_horizontal); // ← 비행 경로각

        int num_others = 0;
        for (int i = 0; i < snapshot.num_agents; i++) {
            if (i == self_idx) continue;
            if (num_others >= kMaxAgents) break;
            StateType::AgentState_rt & s = m_others_buf_rt[num_others];   // ← 정적 버퍼 재사용 (heap 0)
            s.pos_n = snapshot.agents[i].position[0];
            /* ... 동일하게 채움 ... */
            num_others++;
        }

        /* (3) atomic 으로 wind 값 받기 */
        const float wind_n = m_wind_n_mt2rt.load(std::memory_order_relaxed);
        const float wind_e = m_wind_e_mt2rt.load(std::memory_order_relaxed);

        /* (3b) height_setpoint — m_ref_pos_d_mt 가 캡처 안되었으면 NaN */
        const float height_setpoint = m_ref_pos_d_valid_mt
                                          ? m_ref_pos_d_mt
                                          : std::nanf("");

        /* (4) ★ 무거운 계산 — Flocking 가이던스 한 방에 ★ */
        const StateType::FwSetpointOutput_rt2mt out =
            m_flocking->computeFwSetpoint(self, m_others_buf_rt, num_others,
                                          wind_n, wind_e,
                                          height_setpoint);

        /* (5) 출력 큐로 push */
        m_output_queue_rt2mt.try_push(out);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));   // ← 1ms 주기 유지
    }
}
```

### 핵심 디자인 포인트

| 라인 | 디자인 |
|---|---|
| `while (m_rt_running_mt2rt.load())` | 소멸자가 false 로 set 하면 루프 탈출. atomic 이라 mutex 불필요 |
| `m_reinit_vdesired_mt2rt.exchange(false)` | "신호 읽고 동시에 클리어" — atomic exchange 한 번에 |
| `try_pop()` 실패 시 `sleep_for(1ms)` + `continue` | busy-wait 회피, CPU 100% 안 먹음 |
| `m_others_buf_rt` 재사용 | rt 전용 멤버, heap 할당 0 (실시간성 확보) |
| `kMaxAgents` 정적 상한 체크 | snapshot 에 비정상 데이터 와도 buffer overflow 안 남 |
| 마지막 `sleep_for(1ms)` | 데이터가 있어도 1ms 간격 유지 — CPU 절약 |

---

## D. updateSetpoint — main thread 의 출력 단계

파일: `FormationMode.cpp:196-260`

```cpp
void FormationMode::updateSetpoint(float /*dt_s*/)
{
    /* (0) VTOL 가 FW 모드인지 확인 */
    if (!_vtol_status->isFwMode()) {
        RCLCPP_ERROR_THROTTLE(_node.get_logger(), *_node.get_clock(), 1000,
            "[Formation] VTOL이 고정익 모드가 아님!");
    }

    /* (1) 출력 큐에서 pop */
    std::optional<StateType::FwSetpointOutput_rt2mt> maybe_out =
        m_output_queue_rt2mt.try_pop();

    if (maybe_out.has_value()) {
        m_last_output_mt     = maybe_out.value();           // 갱신
        m_has_last_output_mt = true;
    } else if (m_has_last_output_mt) {
        /* (2) 큐 비었지만 이전 값 있음 → hold last (ZOH) */
        RCLCPP_WARN_THROTTLE(...,  "output_queue 비어있음 → hold last (ZOH)");
    } else {
        /* (3) 큐 비었고 이전 값도 없음 → cruise fallback */
        if (std::isfinite(m_cruise_altitude_amsl)) {
            _fw_setpoint->updateWithAltitude(
                m_cruise_altitude_amsl, m_initial_course, m_initial_ground_speed);
        } else {
            _fw_setpoint->updateWithHeightRate(0.f, m_initial_course, m_initial_ground_speed);
        }
        return;
    }

    /* (4) m_last_output_mt 값 인가 */
    if (m_last_output_mt.is_fallback) {
        /* fallback 결과 → cruise */
        _fw_setpoint->updateWithAltitude(...);
    } else {
        /* 정상 결과 → lateral_acceleration + airspeed 모드로 인가 */
        px4_ros2::FwLateralLongitudinalSetpoint sp;
        sp.withLateralAcceleration(m_last_output_mt.lateral_acceleration)
          .withEquivalentAirspeed(m_last_output_mt.airspeed);
        _fw_setpoint->update(sp);   // ★ PX4 로 진짜 publish ★
    }
}
```

### 3 단계 fallback 전략 (가장 잘 읽어야 하는 부분)

| 상태 | 동작 | 의미 |
|---|---|---|
| 큐에 값 있음 | 새 값 사용 | 정상 |
| 큐 비었지만 이전 값 있음 | 이전 값 재사용 (ZOH) | rt_thread 가 잠시 늦음. setpoint 는 끊기지 않음 |
| 큐 비었고 이전 값도 없음 | cruise altitude/heading 으로 fallback | 활성화 직후. rt_thread 가 아직 첫 push 못 함 |

→ **PX4 setpoint 는 절대 끊기면 안 됨** (controller 가 zero 로 떨어져서 추락). 이걸 보장하기 위한 3 단계 안전망.

---

## E. SPSC Lock-Free Queue 의 내부

파일: `/home/leedonghyuck/ros2_ws/src/collision_avoidance/include/collision_avoidance/spsc_queue.hpp`

```cpp
template <typename T, size_t Capacity>
class SpscQueue {
    static_assert((Capacity & (Capacity - 1)) == 0, "must be power of 2");
public:
    bool try_push(const T& item) noexcept {
        const auto head = head_.load(std::memory_order_relaxed);
        const auto tail = tail_.load(std::memory_order_acquire);
        if (head - tail == Capacity) return false;          // ← 가득
        buf_[head & kMask] = item;                          // ← 데이터 복사
        head_.store(head + 1, std::memory_order_release);   // ← head 전진
        return true;
    }
    std::optional<T> try_pop() noexcept {
        const auto tail = tail_.load(std::memory_order_relaxed);
        const auto head = head_.load(std::memory_order_acquire);
        if (head == tail) return std::nullopt;              // ← 비어있음
        T item = buf_[tail & kMask];
        tail_.store(tail + 1, std::memory_order_release);
        return item;
    }
private:
    static constexpr size_t kMask = Capacity - 1;
    alignas(64) std::atomic<size_t> head_{0};               // ← cache line 분리
    alignas(64) std::atomic<size_t> tail_{0};               // ← cache line 분리
    alignas(64) std::array<T, Capacity> buf_{};
};
```

### 왜 빠른가?

| 트릭 | 이유 |
|---|---|
| `static_assert(Capacity is power of 2)` | `head & kMask` 가 modulo 보다 빠름 |
| `alignas(64)` head/tail/buf 분리 | 다른 CPU core 의 cache line bouncing 방지 (false sharing) |
| `memory_order_acquire/release` 만 사용 | 전체 sequential consistency 보다 가볍지만 정확성 보장 |
| Single Producer Single Consumer 가정 | head 는 producer 만 쓰고, tail 은 consumer 만 씀 → 진짜 race 없음 |
| `noexcept` | 컴파일러가 더 공격적으로 inline 가능 |

→ **무거운 mutex 한 번이 ~수십 ns. 이 lock-free 구현은 ~수 ns.** 1000 배 차이.

---

## F. 라이프사이클 — 무엇이 언제 살고 죽는가

```
 시간 →
─────────────────────────────────────────────────────────────────────────────────────
[main 시작]
   │
   │  rclcpp::init()
   │  Node 생성 (shared_ptr, refcount=1)
   │  preflight 생성 (shared_ptr, refcount=1)
   │  formation 생성 (shared_ptr, refcount=1)
   │     │
   │     ├─ ModeBase 부모 초기화
   │     ├─ _fw_setpoint, _vtol_status 생성 (shared_ptr 멤버)
   │     ├─ _wind_sub, _trans_odom_subs[] 생성 (sub 핸들 멤버)
   │     ├─ m_flocking 생성 (unique_ptr 멤버)
   │     └─ m_rt_thread_mt 시작 (rt_loop 무한루프 진입)  ★★★
   │
   │  executor 생성 (shared_ptr, refcount=1, preflight 와 formation 의 ref 보유)
   │  executor->doRegister()  ← PX4 commander 에 등록
   │  formation->doRegister()
   │
   │  rclcpp::spin(node)  ← main thread 가 여기서 블록
   │
   │  ┌──────────────────────────────────────────────────────────────┐
   │  │ [동시 진행] main thread: subscription callback / updateSetpoint│
   │  │ [동시 진행] rt_thread: rt_loop 무한 루프                     │
   │  └──────────────────────────────────────────────────────────────┘
   │
   │  Ctrl-C / 외부 종료 시그널
   │
   │  rclcpp::spin() 리턴
   │  rclcpp::shutdown()
   │  main() 의 local 변수들 (executor, formation, preflight, node) 가 scope 종료
   │
   │  formation 의 shared_ptr 가 마지막 ref 해제
   │     │
   │     └─ FormationMode 소멸자 호출:
   │           - m_rt_running_mt2rt.store(false)  ← rt_loop 의 while 조건 깨짐
   │           - m_rt_thread_mt.join()             ← rt_thread 종료 대기
   │           - 멤버들 자동 소멸 (unique_ptr, shared_ptr)
   │
[main 종료]
```

### 절대 까먹지 말아야 할 것

1. **rt_thread 는 생성자에서 시작, 소멸자에서만 종료**. onActivate/Deactivate 는 신호만 보냄.
2. **소멸자에서 `join()` 안 하면 std::terminate 호출됨**. C++ 스레드 표준이 그렇게 정의함.
3. **순서 중요**: `m_rt_running_mt2rt.store(false)` → `join()` 이지 그 반대 아님. 거꾸로 하면 데드락.

---

## G. 너가 새 mode 만들 때 따라할 보일러플레이트

```cpp
// MyOnboardMode.hpp
class MyOnboardMode : public px4_ros2::ModeBase
{
public:
    MyOnboardMode(rclcpp::Node & node);
    ~MyOnboardMode() override;

    void onActivate() override;
    void onDeactivate() override;
    void updateSetpoint(float dt_s) override;

private:
    void rt_loop();

    /* px4_ros2 wrapper */
    rclcpp::Node & _node;
    std::shared_ptr<px4_ros2::FwLateralLongitudinalSetpointType> _fw_setpoint;

    /* ROS2 subscriptions */
    rclcpp::Subscription<...>::SharedPtr _odom_sub;

    /* Lock-free queues */
    SpscQueue<MyInput,  8> m_input_queue_mt2rt;
    SpscQueue<MyOutput, 8> m_output_queue_rt2mt;

    /* rt_thread */
    std::thread       m_rt_thread_mt;
    std::atomic<bool> m_rt_running_mt2rt{false};

    /* ZOH 버퍼 */
    MyOutput m_last_output_mt{};
    bool     m_has_last_output_mt{false};
};

// MyOnboardMode.cpp
MyOnboardMode::MyOnboardMode(rclcpp::Node & node)
: ModeBase(node, Settings{"My Onboard"}, "/px4_0/")
, _node(node)
{
    _fw_setpoint = std::make_shared<px4_ros2::FwLateralLongitudinalSetpointType>(*this);

    _odom_sub = _node.create_subscription<...>(...,
        [this](const auto & msg) {
            MyInput input{...};
            m_input_queue_mt2rt.try_push(input);
        });

    m_rt_running_mt2rt.store(true);
    m_rt_thread_mt = std::thread(&MyOnboardMode::rt_loop, this);
}

MyOnboardMode::~MyOnboardMode()
{
    m_rt_running_mt2rt.store(false);
    if (m_rt_thread_mt.joinable()) m_rt_thread_mt.join();
}

void MyOnboardMode::rt_loop()
{
    while (m_rt_running_mt2rt.load()) {
        auto input = m_input_queue_mt2rt.try_pop();
        if (!input.has_value()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        MyOutput out = doHeavyComputation(*input);
        m_output_queue_rt2mt.try_push(out);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void MyOnboardMode::updateSetpoint(float)
{
    auto out = m_output_queue_rt2mt.try_pop();
    if (out) {
        m_last_output_mt = *out;
        m_has_last_output_mt = true;
    }
    if (m_has_last_output_mt) {
        px4_ros2::FwLateralLongitudinalSetpoint sp;
        sp.withLateralAcceleration(m_last_output_mt.a_lat)
          .withEquivalentAirspeed(m_last_output_mt.airspeed);
        _fw_setpoint->update(sp);
    } else {
        _fw_setpoint->updateWithAltitude(cruise_alt, cruise_course, cruise_speed);
    }
}
```

→ **이 50 줄이 PX4 + ROS2 + lock-free + thread 안전성을 모두 갖춘 최소 보일러플레이트**.

---

## 마무리 체크리스트

새 mode 작성할 때 확인해야 할 항목:

- [ ] 헤더에서 `_<name>` (외부 핸들), `m_<name>_mt2rt` (통신용) 네이밍 컨벤션 따름
- [ ] 생성자에서 `_fw_setpoint = std::make_shared<...>(*this)` 호출
- [ ] subscription 의 callback 안에서 atomic 또는 SPSC queue 로 데이터 넘김 (직접 공유 변수 쓰기 금지)
- [ ] rt_thread 는 생성자에서 시작, 소멸자에서 stop+join
- [ ] `updateSetpoint(dt)` 는 큐 pop + ZOH/fallback + `_fw_setpoint->update(sp)` 만
- [ ] main.cpp 에서 `make_shared` 로 인스턴스 + `doRegister()` 호출

