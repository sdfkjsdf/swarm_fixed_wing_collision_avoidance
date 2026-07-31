# Formation SITL/HILS

다기체 formation runtime을 검증하기 위한 simulator 실행 자산이다. Production
패키지의 `coordinate_transformer_node`와 `vtol_guidance_node`를 사용하지만 이
디렉터리의 파일은 실기체 배포 대상이 아니다.

- `config/spawn_config.yaml`: Gazebo 기체 수와 spawn 위치
- `scripts/launch_5vtol.sh`: Gazebo Classic과 PX4 인스턴스 실행
- `scripts/monitor_swarm.sh`: 기체별 상태 토픽 모니터링

`launch_5vtol.sh`는 설치된 `collision_avoidance/config/ros_params.yaml`과
`spawn_config.yaml`의 기체 수 및 offset이 같은지 실행 전에 검사한다.
