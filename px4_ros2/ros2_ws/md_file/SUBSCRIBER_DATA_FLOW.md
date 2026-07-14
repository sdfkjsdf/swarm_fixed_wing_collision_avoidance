# ROS2 ↔ rt_thread 데이터 흐름 — Subscriber + Publisher 변환 사슬 분석

> **컨텍스트**: 양방향 데이터 흐름의 단계별 추적.
> - **Part 1 (Subscriber 쪽)**: ROS2 메시지 → 멤버 변수 → SPSC queue → rt_thread → 가이던스 알고리즘 입력
> - **Part 2 (Publisher 쪽)**: rt_thread 계산 결과 → SPSC queue → main thread → px4_ros2 wrapper → PX4 토픽 publish
>
> 분석 대상 코드: `/home/leedonghyuck/ros2_ws/src/collision_avoidance/`

---

# 📥 PART 1 — Subscriber 쪽 (입력: ROS msg → rt_thread)

---

## 큰 그림 — 저장 방식이 3 종류

| 데이터 종류 | 저장 위치 | 동기화 방식 | 예시 |
|---|---|---|---|
| **단일 스칼라값** (소형) | `std::atomic<float>` 멤버 | atomic store/load | wind_n, wind_e |
| **N개 누적 상태** (대형, 점진 누적) | 평범한 `std::array<Struct, N>` 멤버 | **동기화 없음** (단일 스레드만 접근) | `m_state_for_control_mt` |
| **N개 동시 스냅샷** (대형, 한 번에 넘김) | `SpscQueue<Struct>` | lock-free queue | `m_input_queue_mt2rt` |

→ **데이터의 성격에 따라 저장·동기화 방식이 다름**. "전부 atomic" 도 아니고 "전부 mutex" 도 아님.

---

## 1️⃣ 단일 스칼라 — atomic 저장

가장 단순한 케이스. **wind 데이터** (북향·동향 풍속).

### 헤더에서 멤버 선언

`FormationMode.hpp:93-94`
```cpp
/* ── mt → rt: 바람 값 (콜백 쓰기, rt_loop 읽기) ── */
std::atomic<float> m_wind_n_mt2rt{0.f};
std::atomic<float> m_wind_e_mt2rt{0.f};
```

- `std::atomic<float>` → CPU 명령어 수준에서 원자적 read/write 보장
- 초기값 `0.f` 로 0 초기화
- 변수명 `_mt2rt` 접미사 → "main thread 에서 쓰고 rt_thread 에서 읽음"

### main thread (subscription callback) 에서 쓰기

`FormationMode.cpp:21-26`
```cpp
_wind_sub = _node.create_subscription<Wind>(
    "/px4_" + std::to_string(m_vehicle_id) + "/fmu/out/wind", qos,
    [this](const Wind::UniquePtr msg) {
        m_wind_n_mt2rt.store(msg->windspeed_north, std::memory_order_relaxed);
        m_wind_e_mt2rt.store(msg->windspeed_east,  std::memory_order_relaxed);
    });
```

흐름:
```
[PX4 publish wind 토픽] → [DDS] → [rclcpp::spin]
                                       ↓
                                   callback 람다 실행 (main thread)
                                       ↓
                                   msg 객체에서 windspeed_north 추출
                                       ↓
                                   m_wind_n_mt2rt.store(...)  ★ atomic write ★
```

### rt_thread 에서 읽기

`FormationMode.cpp:329-330`
```cpp
const float wind_n = m_wind_n_mt2rt.load(std::memory_order_relaxed);
const float wind_e = m_wind_e_mt2rt.load(std::memory_order_relaxed);
```

→ atomic load 한 번. 그게 끝.

### 왜 atomic 이면 충분한가?

- **데이터 크기가 작다** (float 1 개 = 4 byte) → CPU 가 한 명령어로 read/write
- **bit-pattern 순간 일관성만 필요** → "이 시점의 값" 이면 충분, 이전 값과의 관계 신경 안 씀
- **`memory_order_relaxed`** → ordering 없음, 가장 빠름 (하나의 변수만 다루므로 ordering 무의미)

→ mutex / queue 같은 무거운 도구 불필요.

---

## 2️⃣ N 개 누적 상태 — 평범한 멤버 배열 (★ 동기화 없음 ★)

복잡한 케이스. **5 대 odometry** — N 대가 각자 다른 시점에 도착하므로 누적해서 모아야 함.

### 헤더에서 멤버 선언

`FormationMode.hpp:96-98`
```cpp
/* ── mt: 콜백이 채우는 누적 상태 ── */
std::array<StateType::State_for_Control_mt, kMaxAgents> m_state_for_control_mt{};
StateType::Check_update_mt                              m_agent_updated_mt{};
```

- `std::array<State_for_Control_mt, 8>` → 8 칸 짜리 정적 배열 (heap 0)
- `Check_update_mt` = `std::array<bool, 8>` (StateType.hpp:65) → "각 칸 채워졌나?" 플래그
- 둘 다 **`_mt` 접미사** → main thread 전용. atomic 아님!

### `State_for_Control_mt` 구조

`StateType.hpp:39-48`
```cpp
struct State_for_Control_mt
{
    std::array<float, 3> position          = {0.0f, 0.0f, 0.0f};
    std::array<float, 3> velocity          = {0.0f, 0.0f, 0.0f};
    std::array<float, 3> position_variance = {0.0f, 0.0f, 0.0f};
    std::array<float, 3> velocity_variance = {0.0f, 0.0f, 0.0f};
    double timestamp        = 0.0;
    int    check_vehicle_id = 0;
    bool   update_state     = false;
};
```

→ ROS2 의 `VehicleOdometry` 메시지를 그대로 담을 수 있는 단순 구조체.

### main thread callback 에서 쓰기 — **변환 1단계: ROS msg → 멤버 struct**

`FormationMode.cpp:31-70`
```cpp
_trans_odom_subs[n] = _node.create_subscription<VehicleOdometry>(
    "/common/px4_" + std::to_string(n) + "/trans_vehicle_odometry", qos,
    [this, n](const VehicleOdometry::UniquePtr msg) {
        /* ── ① ROS msg 의 데이터를 그대로 복사 ── */
        m_state_for_control_mt[n].check_vehicle_id  = n;
        m_state_for_control_mt[n].position          = msg->position;
        m_state_for_control_mt[n].velocity          = msg->velocity;
        m_state_for_control_mt[n].position_variance = msg->position_variance;
        m_state_for_control_mt[n].velocity_variance = msg->velocity_variance;
        m_state_for_control_mt[n].timestamp         = 0.0;

        /* ── ② "이 칸 채워졌다" 플래그 ── */
        m_agent_updated_mt[n] = true;

        /* ── ③ N대 모두 도착했나 체크 ── */
        bool all_updated = true;
        for (int i = 0; i < m_total_agent_num; i++) {
            if (!m_agent_updated_mt[i]) { all_updated = false; break; }
        }

        /* ── ④ all-arrived → snapshot 만들고 큐에 push ── */
        if (all_updated) {
            StateType::Total_state_for_Control_mt2rt snapshot{};
            snapshot.num_agents = m_total_agent_num;
            snapshot.agents     = m_state_for_control_mt;       // ★ 통째로 복사 ★
            m_input_queue_mt2rt.try_push(snapshot);
            m_agent_updated_mt.fill(false);                     // ★ 플래그 리셋 ★
        }
    });
```

### 왜 atomic 이 아닌가?

핵심 이유: **`m_state_for_control_mt` 는 main thread 만 접근함**.

- 5 대의 sub callback (`_trans_odom_subs[0]`, `[1]`, ...) 이 모두 같은 main thread 에서 순차 실행됨 (`rclcpp::spin` 의 single executor)
- rt_thread 는 이 배열을 **건드리지 않음**
- → 한 스레드만 쓰니까 race condition 자체가 없음 → atomic 불필요

❗ **이게 이 설계의 핵심 아이디어**:
- 데이터를 **누적할 때는** main thread 안에서만 (동기화 0)
- 데이터를 **rt_thread 로 넘길 때만** SPSC queue 사용

→ 매 callback 마다 큐에 push 하면 큐 트래픽이 N 배 늘어남. 5 대 다 모일 때까지 main thread 안에서 쌓아두다가, 한 번에 넘김 = 효율적.

---

## 3️⃣ N 개 동시 스냅샷 — SPSC Queue

**변환 2단계: 누적 상태 → snapshot → 큐**

### snapshot 자료형

`StateType.hpp:54-58`
```cpp
struct Total_state_for_Control_mt2rt {
    std::array<State_for_Control_mt, kMaxAgents> agents{};
    double timestamp  = 0.0;
    int    num_agents = 0;
};
```

→ N 대 전부의 odometry 를 **한 시점에** 묶은 것. `_mt2rt` 접미사 = 채널용 자료형.

### 큐 자료형

`StateType.hpp:62`
```cpp
using InputQueue_mt2rt = SpscQueue<Total_state_for_Control_mt2rt, kMaxAgents>;
```

→ snapshot 8 개 까지 담을 수 있는 ring buffer.

### push 시점에 일어나는 일

```cpp
StateType::Total_state_for_Control_mt2rt snapshot{};   // [a] stack 위 임시 객체
snapshot.num_agents = m_total_agent_num;
snapshot.agents     = m_state_for_control_mt;           // [b] 멤버 array 통째 복사
m_input_queue_mt2rt.try_push(snapshot);                 // [c] 큐 내부 버퍼로 한 번 더 복사
```

| 단계 | 어디로 | 비용 |
|---|---|---|
| [a] stack 위에 빈 snapshot 생성 | stack | 0 (정적 할당) |
| [b] 멤버 array → snapshot.agents 통째 복사 | stack | sizeof(State_for_Control_mt) × 8 ≈ 1KB 복사 |
| [c] snapshot → 큐 내부 ring buffer 복사 | heap (큐 내부) | 같은 1KB 한 번 더 |

→ 총 **2 회 복사**. 1KB 정도면 nanosecond 단위, 무시 가능.

→ **복사하는 게 핵심**. 큐에 reference 가 아닌 **값**을 넣음으로써 producer 쪽이 다음 사이클에 멤버를 덮어써도 큐 안의 데이터는 안전.

### push 의 atomic 동작 (lock-free 의 진짜 정체)

`spsc_queue.hpp:10-17`
```cpp
bool try_push(const T& item) noexcept {
    const auto head = head_.load(std::memory_order_relaxed);   // [1] 내 head 위치
    const auto tail = tail_.load(std::memory_order_acquire);   // [2] consumer 의 tail
    if (head - tail == Capacity) return false;                 // [3] 가득찼나
    buf_[head & kMask] = item;                                 // [4] ★ 데이터 복사 ★
    head_.store(head + 1, std::memory_order_release);          // [5] head 전진 (releasing)
    return true;
}
```

핵심:
- **[4] 까지는 producer 의 thread-local 작업** — buf 의 새 칸에 데이터 복사
- **[5] head_.store(release)** ← 이 한 줄이 "이제 데이터 준비 끝!" 라는 atomic signal
- consumer 가 [5] 의 release 를 [2] 의 acquire 로 보면 → [4] 의 메모리 변화도 함께 보임 (memory ordering 보장)

→ **mutex 없이도 producer→consumer 데이터 가시성 보장**. 이게 lock-free 의 마법.

---

## 4️⃣ rt_thread 가 받아서 또 다시 변환 — **변환 3단계: snapshot → AgentState_rt**

큐에서 꺼낸 snapshot 은 ROS 메시지 그대로의 형태인데, 가이던스 알고리즘이 원하는 형태 (코스각·비행경로각 포함) 로 또 변환됨.

### 사용할 자료형

`StateType.hpp:73-83`
```cpp
struct AgentState_rt {
    float pos_n{0.f};
    float pos_e{0.f};
    float pos_d{0.f};
    float vel_n{0.f};
    float vel_e{0.f};
    float vel_d{0.f};
    float speed{0.f};      // ← 새로 계산된 derived field
    float psi{0.f};        // ← 새로 계산된 derived field (코스각)
    float gamma{0.f};      // ← 새로 계산된 derived field (비행경로각)
};
```

- `_rt` 접미사 = rt_thread 전용
- 메시지 원본에 없던 `speed`, `psi`, `gamma` 추가 (계산해서 채움)

### rt_loop 안의 변환

`FormationMode.cpp:294-326`
```cpp
/* ── ① 큐에서 snapshot pop ── */
std::optional<StateType::Total_state_for_Control_mt2rt> input_state =
    m_input_queue_mt2rt.try_pop();
if (!input_state.has_value()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    continue;
}
const auto & snapshot = input_state.value();
const int self_idx    = m_vehicle_id;

/* ── ② self 추출 + derived field 계산 ── */
StateType::AgentState_rt self;
self.pos_n = snapshot.agents[self_idx].position[0];        // 단순 복사
self.pos_e = snapshot.agents[self_idx].position[1];
self.pos_d = snapshot.agents[self_idx].position[2];
self.vel_n = snapshot.agents[self_idx].velocity[0];
self.vel_e = snapshot.agents[self_idx].velocity[1];
self.vel_d = snapshot.agents[self_idx].velocity[2];
self.speed = std::sqrt(self.vel_n*self.vel_n
                     + self.vel_e*self.vel_e
                     + self.vel_d*self.vel_d);             // ★ 계산 ★
self.psi   = std::atan2(self.vel_e, self.vel_n);           // ★ 코스각 ★
const float v_horizontal = std::sqrt(self.vel_n*self.vel_n + self.vel_e*self.vel_e);
self.gamma = std::atan2(-self.vel_d, v_horizontal);        // ★ 비행경로각 ★

/* ── ③ others 추출 (정적 버퍼 m_others_buf_rt 재사용) ── */
int num_others = 0;
for (int i = 0; i < snapshot.num_agents; i++) {
    if (i == self_idx) continue;
    if (num_others >= kMaxAgents) break;
    StateType::AgentState_rt & s = m_others_buf_rt[num_others];
    s.pos_n = snapshot.agents[i].position[0];
    /* ... 동일하게 9개 필드 채움 ... */
    num_others++;
}
```

핵심 포인트:
- **`m_others_buf_rt` 는 rt_thread 전용 멤버** (`_rt` 접미사) → 매번 재사용, heap 할당 0
- **derived field 계산은 rt_thread 에서만** — main thread callback 은 무거운 sqrt/atan2 안 함
- snapshot 은 함수 끝나면 자동으로 사라짐 (stack 위 std::optional)

---

## 전체 변환 사슬 — 데이터의 변천사

ROS2 메시지 한 발이 어떻게 가이던스 알고리즘 입력까지 도달하는지:

```
[PX4 / TransferSameCoordinate node]
     │
     │  publish: px4_msgs::msg::VehicleOdometry
     │         { position, velocity, position_variance, velocity_variance, ... }
     ▼
[main thread: _trans_odom_subs[n] callback]
     │
     │  변환 1: ROS msg → State_for_Control_mt
     │  m_state_for_control_mt[n].position = msg->position;  // 누적 저장
     │  m_state_for_control_mt[n].velocity = msg->velocity;
     │  m_agent_updated_mt[n] = true;
     │
     │  ★ all-arrived 체크 ★
     │  (N대 다 모이면 ↓ 진행, 아니면 위에서 끝)
     │
     │  변환 2: 누적 array → snapshot (Total_state_for_Control_mt2rt)
     │  snapshot.agents = m_state_for_control_mt;  // 통째 복사
     │  snapshot.num_agents = m_total_agent_num;
     │
     │  m_input_queue_mt2rt.try_push(snapshot)
     │     │
     │     │  큐 내부 ring buffer 로 또 한 번 복사 (lock-free)
     │     │  head_.store(release)  ← atomic signal
     ▼
[m_input_queue_mt2rt 의 ring buffer 안에 snapshot 대기]
     │
     │  ─── 다른 스레드 경계 ───
     │
     │  rt_thread 가 try_pop()
     │     │
     │     │  tail_.load(acquire)  ← release 와 짝, 메모리 가시성 보장
     │     │  buf_[tail & kMask] 에서 한 번 더 복사 (snapshot 반환)
     ▼
[rt_thread: rt_loop]
     │
     │  변환 3: snapshot → AgentState_rt
     │  self.pos_n = snapshot.agents[self_idx].position[0];
     │  self.psi   = atan2(self.vel_e, self.vel_n);   ← derived
     │  self.gamma = atan2(-self.vel_d, v_horizontal); ← derived
     │  self.speed = sqrt(...);                        ← derived
     │
     │  others 도 같은 방식으로 m_others_buf_rt 에 채움
     ▼
[FlockingGuidance::computeFwSetpoint(self, others, ...)]
     │
     │  ★ 가이던스 알고리즘 입력 완료 ★
```

---

## 정리 — 왜 이렇게 단계가 많나?

| 단계 | 이유 |
|---|---|
| 변환 1 (msg → mt struct) | ROS msg 형식과 우리 형식이 다름. 우리 형식으로 통일 |
| 누적 (N 대 모일 때까지 대기) | rt_thread 는 "한 시점의 swarm 전체" 를 받아야 가이던스 가능 |
| 변환 2 (struct → snapshot, 큐로 push) | rt_thread 로 thread-safe 하게 넘기는 유일한 방법 |
| 변환 3 (snapshot → AgentState_rt) | 가이던스 알고리즘이 원하는 derived field (psi, gamma, speed) 추가 |

**왜 한 번에 안 하고 단계적인가?**

1. **각 단계가 다른 스레드의 책임**
   - 변환 1, 누적 → main thread (가벼운 작업, ROS 콜백 안에서)
   - 변환 3, 무거운 계산 → rt_thread (격리)

2. **각 단계가 다른 데이터 lifetime**
   - 누적 array: 다음 사이클까지 살아있음 (callback 사이)
   - snapshot: 큐에 push 되면 producer 쪽 사라짐
   - AgentState_rt: 가이던스 함수 호출 한 번 동안만

3. **race condition 회피**
   - 변환 1은 같은 thread 안에서 → 동기화 0
   - 변환 2의 SPSC push 는 atomic store-release → mutex 0
   - 변환 3은 rt_thread 안에서 → 동기화 0

→ **결과: thread 경계는 단 한 군데 (SPSC queue), 거기서만 atomic. 나머지는 전부 단일-스레드 작업.** 이게 lock-free 패턴의 정수.

---
---

# 📤 PART 2 — Publisher 쪽 (출력: rt_thread → PX4)

> 가이던스 알고리즘이 계산한 setpoint 가 어떻게 main thread 로 돌아오고, 어떻게 px4_ros2 wrapper 를 통해 PX4 로 publish 되는지 추적.

---

## 큰 그림 — 출력도 3 단계 변환

| 단계 | 어디서 | 무엇을 |
|---|---|---|
| **변환 4** (rt_thread) | `FlockingGuidance::computeFwSetpoint()` 내부 | 가이던스 계산 결과 → `FwSetpointOutput_rt2mt` 구조체 |
| **변환 5** (rt → mt) | `m_output_queue_rt2mt.try_push()` → `try_pop()` | SPSC 큐로 thread-safe 전달 + ZOH 버퍼 적재 |
| **변환 6** (main thread) | `updateSetpoint()` 안 | `FwSetpointOutput_rt2mt` → `px4_ros2::FwLateralLongitudinalSetpoint` (builder pattern) |
| **변환 7** (px4_ros2 lib) | `_fw_setpoint->update(sp)` | C++ struct → PX4 ROS2 메시지 → DDS publish |

---

## 5️⃣ 가이던스 결과 만들기 — `FwSetpointOutput_rt2mt`

가이던스 알고리즘 내부에서 **출력 전용 자료형**으로 결과 묶기.

### 출력 자료형

`StateType.hpp:93-99`
```cpp
struct FwSetpointOutput_rt2mt {
    float course               = 0.f;  /* [rad] atan2(v_e, v_n) */
    float airspeed             = 0.f;  /* [m/s] clamp 적용 후 */
    float height_rate          = 0.f;  /* [m/s] ENU (NED 의 -v_d) */
    float lateral_acceleration = 0.f;  /* [m/s^2] 횡방향 가속도 */
    bool  is_fallback          = true; /* true = cruise fallback 으로 처리 */
};
```

- `_rt2mt` 접미사 = rt → main 채널용
- 가이던스가 결정해야 할 **모든 setpoint 필드** 가 한 구조체에 묶임
- `is_fallback` 플래그 = "정상 계산 실패. main thread 가 cruise 로 fallback 해야 함" 신호

### FlockingGuidance 가 채우는 부분

`FlockingGuidance.cpp:160-165`
```cpp
StateType::FwSetpointOutput_rt2mt out;
out.airspeed             = m_speed_setpoint_rt;          // 클램프 후
out.height_rate          = m_height_rate_setpoint_rt;    // alt_hold P 제어
out.lateral_acceleration = m_lateral_acceleration_rt;    // coordinated turn 클램프
out.is_fallback          = false;                         // 정상 계산
return out;
```

- 가이던스 본체 (`computeFwSetpoint`) 가 NE 평면 가속도 → saturation → 변환을 모두 마친 결과
- 멤버 변수 `m_*_rt` 는 디버그/내부 상태 추적용 (외부엔 `out` 만 노출)

→ **이 시점에 `out` 은 rt_thread 의 stack 위에 있는 임시 객체.**

---

## 6️⃣ rt → mt 출력 큐 — `m_output_queue_rt2mt`

### 큐 자료형

`StateType.hpp:102`
```cpp
using OutputQueue_rt2mt = SpscQueue<FwSetpointOutput_rt2mt, 8>;
```

- capacity 8 (입력 큐와 같음)
- element 타입은 `FwSetpointOutput_rt2mt`

### 헤더에서 멤버 선언

`FormationMode.hpp:113-114`
```cpp
/* ── rt → mt: 최종 setpoint 채널 ── */
StateType::OutputQueue_rt2mt m_output_queue_rt2mt{};
```

### rt_thread 가 push

`FormationMode.cpp:338-348`
```cpp
const StateType::FwSetpointOutput_rt2mt out =
    m_flocking->computeFwSetpoint(self, m_others_buf_rt, num_others,
                                  wind_n, wind_e,
                                  height_setpoint);

/* (4) output_queue 로 push */
m_output_queue_rt2mt.try_push(out);
```

- `out` 이 큐 내부 ring buffer 로 복사 (sizeof = 17 byte 정도, 무시 가능)
- atomic store-release 로 head 전진 → main thread 가 곧 pop 가능

### main thread 가 pop — `updateSetpoint(dt)` 안

`FormationMode.cpp:220-240`
```cpp
/* (1) rt_thread 가 push 한 최신 setpoint pop */
std::optional<StateType::FwSetpointOutput_rt2mt> maybe_out =
    m_output_queue_rt2mt.try_pop();

if (maybe_out.has_value()) {
    m_last_output_mt     = maybe_out.value();   // ★ ZOH 버퍼에 보관 ★
    m_has_last_output_mt = true;
} else if (m_has_last_output_mt) {
    /* (2) 큐 비었지만 이전 값 있음 → hold last (ZOH) */
    RCLCPP_WARN_THROTTLE(...,  "output_queue 비어있음 → hold last (ZOH)");
} else {
    /* (3) 큐 비었고 이전 값도 없음 → cruise fallback */
    _fw_setpoint->updateWithAltitude(
        m_cruise_altitude_amsl, m_initial_course, m_initial_ground_speed);
    return;
}
```

### 핵심: ZOH (Zero-Order Hold) 버퍼

`FormationMode.hpp:117-118`
```cpp
/* ── mt: updateSetpoint 의 hold-last 버퍼 ── */
StateType::FwSetpointOutput_rt2mt m_last_output_mt{};
bool m_has_last_output_mt{false};
```

- `_mt` 접미사 = main thread 전용 (rt 가 안 건드림)
- atomic 불필요. 주체는 한 thread 뿐
- 큐가 비었을 때 **이전 값을 재사용** → setpoint 가 끊기지 않음

### 3 단계 fallback 의 이유

| 상태 | 동작 | 왜 필요? |
|---|---|---|
| 큐 hit | 새 값 사용 | 정상 |
| 큐 miss + ZOH | 이전 값 재사용 | rt_thread 가 잠깐 늦거나 (1ms 주기) updateSetpoint (33ms 주기) 가 빨리 호출되면 큐 비어있을 수 있음. setpoint 끊으면 PX4 controller 가 오류 |
| 큐 miss + 첫 호출 | cruise fallback | 활성화 직후. rt_thread 가 첫 push 못 한 상태. 안전한 디폴트로 |

→ **PX4 setpoint 는 절대 끊기면 안 됨.** controller 입력이 없으면 PX4 가 fail-safe 로 진입 (RC return 등). 이 3 단계가 그걸 막음.

---

## 7️⃣ `_fw_setpoint->update(sp)` — Builder 패턴 + DDS publish

### 변환 6: `FwSetpointOutput_rt2mt` → `px4_ros2::FwLateralLongitudinalSetpoint`

`FormationMode.cpp:255-258`
```cpp
/* 2D flocking → lateral_acceleration + airspeed 사용 */
px4_ros2::FwLateralLongitudinalSetpoint sp;
sp.withLateralAcceleration(m_last_output_mt.lateral_acceleration)
  .withEquivalentAirspeed(m_last_output_mt.airspeed);
_fw_setpoint->update(sp);
```

#### Builder 패턴이란?

```cpp
sp.withLateralAcceleration(0.5f)          // ① 객체 sp 의 멤버 설정
  .withEquivalentAirspeed(15.0f);         // ② 위 함수가 *this 를 반환 → 체이닝
```

- 각 `with*()` 함수가 `FwLateralLongitudinalSetpoint &` 를 반환
- 그래서 `.A().B().C()` 처럼 연쇄 호출 가능
- ✅ 장점: 어떤 필드를 설정했는지 한눈에. 설정 안 한 필드는 NaN 으로 남음 → wrapper 가 무시
- ❌ 안 좋은 점: 컴파일러 에러 메시지가 길어짐

#### 왜 4 개 중 2 개 (lateral_acceleration + airspeed) 만 채우나?

`px4_ros2::FwLateralLongitudinalSetpoint` 는 여러 모드 지원:

| 모드 | 필요 필드 | 사용 시점 |
|---|---|---|
| Course tracking | course, airspeed | 직선 비행 |
| Lateral acceleration | lateral_acceleration, airspeed | ★ flocking ★ |
| Altitude hold | altitude_amsl, course, airspeed | cruise fallback |
| Height rate | height_rate, course, airspeed | climb/descent |

- **Flocking 의 출력은 횡 가속도 (`lateral_acceleration`)** 라 이 모드 사용
- course 를 NaN 으로 두면 wrapper 가 "lateral_acceleration 이 직접 제어 입력" 으로 해석
- 수직 채널은 (현재 코드에서) `m_height_rate_setpoint_rt` 로 따로 처리 — 헤더 주석 참고

### 변환 7: wrapper 안에서 일어나는 일

`_fw_setpoint->update(sp)` 가 호출되면 `px4_ros2::FwLateralLongitudinalSetpointType` 안에서:

```
[update(sp) 진입]
    ↓
1. sp 의 NaN 필드 검사 → 어떤 setpoint 모드인지 자동 결정
    ↓
2. PX4 의 raw 메시지 (LateralControlConfiguration, LongitudinalControlConfiguration 등)
   여러 개를 채움 — uORB topic 단위
    ↓
3. timestamp 자동 설정 (현재 시각)
    ↓
4. 내부적으로 보유한 publisher 들로 publish
   - /px4_n/fmu/in/lateral_control_configuration
   - /px4_n/fmu/in/longitudinal_control_configuration
   - 등
    ↓
5. uXRCE-DDS bridge → PX4 의 fixed-wing controller
    ↓
6. PX4 가 실제 actuator (모터·서보) 제어
```

→ **사용자는 위 1~6 단계를 직접 안 함.** wrapper 가 PX4 의 메시지 형식·timestamp·QoS·multiple-publish 를 모두 책임.

### 만약 wrapper 없이 직접 publish 한다면?

```cpp
// [BEFORE] — 직접 publisher 시절
auto pub = node->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    "/fmu/in/trajectory_setpoint", qos);

px4_msgs::msg::TrajectorySetpoint msg;
msg.timestamp     = now_us();           // ← 사용자가 직접 시각 채워야
msg.velocity[0]   = ...;                // ← 어떤 필드가 어디로 가는지 외워야
msg.velocity[1]   = ...;
msg.acceleration  = {...};              // ← 단위·좌표계 다 이해해야
msg.yaw           = ...;
pub->publish(msg);

// 그리고 mode 가 활성화 되었음을 PX4 에 알리는 메시지를 또 다른 publisher 로...
```

→ **boilerplate 가 폭발**. wrapper 가 이걸 다 숨겨줌.

---

## 전체 변환 사슬 — Publisher 쪽 데이터 변천사

가이던스 알고리즘의 결과가 어떻게 PX4 actuator 까지 도달하는지:

```
[FlockingGuidance::computeFwSetpoint(self, others, ...)]
     │
     │  변환 4: 알고리즘 내부 상태 → 출력 자료형
     │
     │  StateType::FwSetpointOutput_rt2mt out;
     │  out.airspeed             = m_speed_setpoint_rt;
     │  out.height_rate          = m_height_rate_setpoint_rt;
     │  out.lateral_acceleration = m_lateral_acceleration_rt;
     │  out.is_fallback          = false;
     │  return out;
     ▼
[rt_thread: rt_loop]
     │
     │  변환 5a: rt → mt 큐로 push
     │  m_output_queue_rt2mt.try_push(out)
     │     │
     │     │  큐 ring buffer 안으로 복사 (lock-free)
     │     │  head_.store(release)  ← atomic signal
     ▼
[m_output_queue_rt2mt 의 ring buffer 안에 out 대기]
     │
     │  ─── 다른 스레드 경계 ───
     │
     │  main thread updateSetpoint(dt) 가 try_pop()
     │     │
     │     │  tail_.load(acquire)  ← release 와 짝
     │     │  buf_[tail & kMask] 에서 복사 (out 반환)
     ▼
[main thread: updateSetpoint(dt)]
     │
     │  변환 5b: pop 결과 → ZOH 버퍼
     │  m_last_output_mt = maybe_out.value();   ← 보관
     │  m_has_last_output_mt = true;
     │
     │  ★ 큐 비었을 때 fallback 처리 ★
     │  큐 hit → 새 값 사용
     │  큐 miss + 이전값 → ZOH (m_last_output_mt 재사용)
     │  큐 miss + 첫 호출 → cruise fallback
     │
     │  변환 6: 우리 자료형 → px4_ros2 builder 자료형
     │
     │  px4_ros2::FwLateralLongitudinalSetpoint sp;
     │  sp.withLateralAcceleration(m_last_output_mt.lateral_acceleration)
     │    .withEquivalentAirspeed(m_last_output_mt.airspeed);
     │
     │  _fw_setpoint->update(sp)
     ▼
[px4_ros2::FwLateralLongitudinalSetpointType wrapper]
     │
     │  변환 7: builder struct → PX4 ROS2 메시지
     │
     │  - sp 의 NaN 필드 검사 → 모드 자동 결정
     │  - LateralControlConfiguration, LongitudinalControlConfiguration 등 채움
     │  - timestamp 자동 추가
     │
     │  내부 publisher 들이 publish
     │     │
     │     │  /px4_n/fmu/in/lateral_control_configuration
     │     │  /px4_n/fmu/in/longitudinal_control_configuration
     │     │  ... (여러 토픽)
     ▼
[uXRCE-DDS bridge]
     │
     │  ROS2 메시지 → uORB 메시지 변환
     ▼
[PX4 fixed-wing controller]
     │
     │  setpoint 를 받아서 PID·TECS·NPFG 통과
     │  → 모터 throttle / aileron / elevator / rudder 명령 생성
     ▼
[실제 actuator 제어]
```

---

## Subscriber vs Publisher — 비교 정리

| 항목 | Subscriber 쪽 (📥) | Publisher 쪽 (📤) |
|---|---|---|
| **데이터 시작점** | PX4 / Gazebo / 다른 노드의 publish | FlockingGuidance::computeFwSetpoint |
| **데이터 끝점** | 가이던스 알고리즘 입력 (AgentState_rt) | PX4 fixed-wing controller |
| **변환 횟수** | 3 단계 (msg → mt struct → snapshot → AgentState_rt) | 4 단계 (out → queue → ZOH → builder → publish) |
| **큐 방향** | mt → rt | rt → mt |
| **큐 element 자료형** | `Total_state_for_Control_mt2rt` (~1KB) | `FwSetpointOutput_rt2mt` (~17 byte) |
| **N 대 동기화** | all-arrived 패턴 (5 대 모일 때까지 대기) | 단일 출력 (자기 setpoint 만) |
| **Atomic 사용** | wind_n/e (mt2rt scalar) | 종료 플래그 / reinit 신호만 |
| **ZOH 버퍼** | 없음 (callback 마다 갱신) | 있음 (`m_last_output_mt`) |
| **Fallback** | 없음 (입력 없으면 그냥 wait) | 3 단계 (ZOH → cruise) |
| **네이밍 컨벤션** | `_mt2rt` 접미사 | `_rt2mt` 접미사 |
| **Wrapper 사용** | rclcpp 의 `create_subscription` (그대로) | px4_ros2 의 `_fw_setpoint->update(sp)` |
| **메시지 포맷 처리** | 사용자가 직접 (msg → struct 복사) | wrapper 가 자동 |

→ **대칭적이지만 비대칭적이다**. 큐 방향과 자료형은 대칭. 단 publisher 쪽은 **fallback + ZOH** 가 추가로 필요 (PX4 가 setpoint 끊기면 안 되니까).

---

## 통합 정리 — 양방향 lock-free 패턴

### 변수명 접미사로 보는 통신 방향

```
m_<name>          ── 단일 thread 내부 (동기화 0)
                       예: m_state_for_control_mt, m_others_buf_rt, m_last_output_mt

m_<name>_mt         ── main thread 전용
                       예: m_agent_updated_mt, m_has_last_output_mt

m_<name>_rt         ── rt thread 전용
                       예: m_others_buf_rt, m_speed_setpoint_rt

m_<name>_mt2rt    ── main → rt 단방향
                       예: m_wind_n_mt2rt (atomic)
                           m_input_queue_mt2rt (queue)
                           m_reinit_vdesired_mt2rt (atomic flag)

m_<name>_rt2mt    ── rt → main 단방향
                       예: m_output_queue_rt2mt (queue)
```

### 통신 방식 결정 표

| 데이터 성격 | 동기화 도구 | 예시 |
|---|---|---|
| 단일 스칼라 (1~8 byte), 최신값만 필요 | `std::atomic<T>` | `m_wind_n_mt2rt`, `m_rt_running_mt2rt` |
| signal/flag (한 번 set, 한 번 read+clear) | `std::atomic<bool>` + `exchange()` | `m_reinit_vdesired_mt2rt` |
| 큰 구조체 (수십 byte ~ 수 KB), 한 시점 단위 | `SpscQueue<T, N>` | `m_input_queue_mt2rt`, `m_output_queue_rt2mt` |
| 한 thread 안에서만 접근 | 동기화 없음 (평범한 멤버) | `m_state_for_control_mt`, `m_last_output_mt` |

### 양방향 데이터 흐름 한눈에

```
                        ┌─────────────────────┐
   [PX4/external]       │     main thread      │      [PX4 controller]
        │               │                     │              ▲
        │  ROS sub       │  ┌────────────┐    │              │
        ├───────────────►│  │_mt callback │     │              │
        │               │  └─────┬──────┘    │              │
        │               │        │ atomic.store           │
        │               │        ▼                          │
        │               │  ┌────────────┐    │              │
        │               │  │_mt 누적 array│    │              │
        │               │  └─────┬──────┘    │              │
        │               │        │ all-arrived             │
        │               │        ▼                          │
        │               │  ╔═══════════╗     │              │
        │               │  ║Queue mt2rt ║──┐  │              │
        │               │  ╚═══════════╝  │  │              │
        │               └──────────────────┼──┘              │
        │                                 │                  │
        │                                 │   atomic     ╔════════╗
        │                                 │   release ─► ║rt thread║
        │                                 │              ╠════════╣
        │                                 │              ║Flocking ║
        │                                 │              ║compute  ║
        │                                 │              ╚═══╤════╝
        │                                 │                 │
        │                                 │                 ▼
        │                                 │              ╔═══════════╗
        │                                 │              ║Queue rt2mt ║
        │                                 │              ╚═══════════╝
        │                                 │                 │
        │               ┌──────────────────┼─────────────────┘
        │               │     main thread  │
        │               │                 │
        │               │  updateSetpoint(dt)│
        │               │  ┌────────────┐  │
        │               │  │try_pop      │  │
        │               │  └─────┬──────┘  │
        │               │        │         │
        │               │        ▼         │
        │               │  ┌────────────┐  │
        │               │  │ZOH buffer   │  │
        │               │  │_last_output │  │
        │               │  └─────┬──────┘  │
        │               │        │         │
        │               │        ▼         │
        │  DDS publish    │  ┌─────────────┐  │
        ◄────────────────┤  │_fw_setpoint │  │
                         │  │ ->update(sp)│  │
                         │  └─────────────┘  │
                         └──────────────────┘
```

### 한 줄 요약

> **데이터는 SPSC queue 한 군데에서만 thread 경계를 넘는다. 그 앞과 뒤는 모두 단일-thread 작업.**
>
> 이 설계 원칙 하나가:
> - mutex 0 (lock-free)
> - heap 할당 0 (정적 멤버 + 큐)
> - jitter 격리 (계산 폭증해도 main thread 영향 0)
> - race condition 0 (변수명 접미사로 강제)
>
> **를 동시에 달성한다.**
