# ROS 2 소스 구조

현재 소스는 실기체 탑재 대상과 SITL/HILS 검증 도구가 한 방향으로만 의존하도록
분리한다.

```text
ros2_ws/src/
├── collision_avoidance/                    # production ROS 2 package
│   ├── include/collision_avoidance/         # 외부에 공개되는 C++ API
│   │   ├── common/
│   │   ├── coordinate/
│   │   ├── estimation/
│   │   │   ├── trajectory_prediction/
│   │   │   └── reconstruction/
│   │   ├── guidance/
│   │   └── modes/
│   ├── src/                                # .cpp 구현
│   │   ├── coordinate/
│   │   ├── estimation/
│   │   │   ├── trajectory_prediction/
│   │   │   └── reconstruction/
│   │   ├── guidance/
│   │   ├── modes/
│   │   └── nodes/
│   ├── test/                               # production 알고리즘 unit test
│   └── config/
│
├── testing_module/                         # grouping directory; package.xml 없음
│   ├── trajectory_prediction_hils/         # SITL/HILS ROS 2 package
│   │   ├── include/trajectory_prediction_hils/
│   │   │   ├── common/
│   │   │   ├── logging/
│   │   │   ├── modes/
│   │   │   └── replay/
│   │   ├── src/
│   │   │   ├── logging/
│   │   │   ├── modes/
│   │   │   ├── nodes/
│   │   │   └── replay/
│   │   ├── config/
│   │   ├── scripts/
│   │   └── docs/
│   ├── formation_hils/                      # multi-VTOL simulator harness
│   │   ├── config/
│   │   └── scripts/
│   └── analysis_tools/                     # Python/shell offline tools
│
├── px4_msgs/                               # pinned external dependency
├── px4_ros_com/                            # pinned external dependency
└── px4-ros2-interface-lib/                 # pinned external dependency
```

## 의존 방향

```text
trajectory_prediction_hils
        │
        ├── collision_avoidance::trajectory_prediction_core
        └── collision_avoidance::trajectory_reconstruction
                              │
                              ▼
                     collision_avoidance

collision_avoidance  ──X──>  testing_module
```

Production 패키지는 시험 패키지나 분석 스크립트를 참조하지 않는다.

## C++ 파일 배치 규칙

- 다른 target/package가 include하는 공개 헤더는
  `include/<package>/<module>/`에 둔다.
- 구현 `.cpp`는 `src/<module>/`에 두고 공개 헤더의 상대 모듈 경로와 맞춘다.
- 한 target 내부에서만 사용하는 private 헤더는 해당 `src/<module>/`에 둘 수 있다.
- 실행 진입점은 `src/nodes/`에 둔다.
- template 구현은 헤더 또는 같은 공개 경로의 `.tpp`에 둔다.
- production unit test는 소유 패키지의 `test/`에 둔다.
- 시나리오 실행, CSV 기록, simulator 관리, 후처리는 `testing_module/`에 둔다.

## 빌드 target

| Target | 역할 |
| --- | --- |
| `collision_avoidance::trajectory_prediction_core` | heap-free RK4 궤적 예측 |
| `collision_avoidance::trajectory_reconstruction` | key sample 기반 cubic spline 복원 |
| `vtol_guidance_node` | PX4 preflight, formation, flocking runtime |
| `coordinate_transformer_node` | 다기체 공통 좌표 변환 |
| `trajectory_replay_node` | SITL/HILS setpoint replay 및 CSV 검증 |
