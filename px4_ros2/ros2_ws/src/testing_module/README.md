# Testing modules

이 디렉터리는 production에 배포하지 않는 SITL/HILS 검증 코드와 오프라인 분석
도구를 보관한다. 이 디렉터리 자체는 ROS 2 패키지가 아니므로 `package.xml`을
두지 않는다.

- `trajectory_prediction_hils/`: 단일 VTOL replay, CSV logging, 시험 case 실행
- `formation_hils/`: 다기체 Gazebo/PX4 실행과 swarm 상태 모니터링
- `analysis_tools/`: 생성된 CSV의 비교·통계·그래프 생성

시험 패키지는 `collision_avoidance`의 공개 API만 사용한다. Production 코드는
이 디렉터리를 include하거나 링크하지 않는다.
