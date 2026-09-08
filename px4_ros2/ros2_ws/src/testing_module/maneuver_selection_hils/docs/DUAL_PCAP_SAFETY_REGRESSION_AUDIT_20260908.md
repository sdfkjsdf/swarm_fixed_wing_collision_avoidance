# 10.294 m → 7.767 m 실행 비교: Git·소스·실행 경로 감사

## 판정

**두 실행 사이 제어 코드 변경으로 인한 회귀는 발견되지 않았다. 안전거리
미달은 실제 기록됐지만, 수정할 신규 코드 결함이나 단일 원인이 확정된 것은 아니다.**
따라서 이 감사에서 제어기·임계값·큐를 임의로 변경하거나 롤백하지 않았다.
추가한 것은 오프라인 `analysis/inspect_separation_event.py`뿐이다.

비교 대상:

- `hybrid_formation_partitioned_inbox_200s_20260908_01`: 최소 10.29373 m.
- `hybrid_formation_dual_pcap_200s_20260908_01`: 최소 7.76659 m, 약 0.8 s 미달.

## 1. 변경 이력 검증

두 실행의 기반 HEAD는 `2fbb5ef`이다. 그 뒤의 stopped-pipeline 계측과
partitioned ordered inbox는 둘 모두에 이미 들어 있었다. 이 변경은 아직
미커밋이므로 HEAD 비교만으로 동일 소스라고 판단하지 않았다.

첫 실행의 `collision_avoidance_source.tar.gz` 보존 파일 107개를 현재 파일과
바이트 비교: **차이 0개**. include/src/config/scripts/test 등 보존 범위의
검증이며, 모든 시스템 파일의 동일성을 뜻하지 않는다.

두 manifest의 PC 실행파일 SHA256:

`417338b54be753b17d1ecf7f9bfdcbe7d9817071f0d0287a5e144af587b73bab`

현재 canonical workspace 설치 파일도 위 값이다. 실제 launcher는
`/home/hmcl/workspace/swarm-fixed-wing/ros2_ws`의 overlay를 사용한다.
소스 저장소 내부의 별도 `px4_ros2/ros2_ws/install`은 다른 빌드이며 이 실행의
비교 대상으로 혼동하면 안 된다.

두 manifest의 Pi 이미지 ID:

`sha256:f07f9e1768e005f6e4f68c7ee86edff4964cd5daa2096a5307f8bdcdc1eca1be`

로그의 Formation/airframe/AMAC 시작 설정 및 manifest 비교에서도 제어 설정
변경은 발견하지 못했다. AD<0, B_comm=0, roll tau=0.5 s, graph on,
V4/positive filter/formation discrimination off를 유지했다.
이번 실행에 추가된 부하는 양쪽 호스트의 전체 UDP 패킷 캡처와 파일 저장이다.
그 관찰자 효과는 0이 아니지만 안전거리 악화의 원인이라고 입증하지 못했다.

Git 이력:

- `fd1698b`: component activation delivery 및 AD-only 기동 선택.
- `b1e15d4`: stopped-only 계측.
- `2fbb5ef`: rejoin metadata 정합성과 입력/출력 drain 상한.
- 이후 미커밋 inbox/계측 변경: **안전거리를 만족한 첫 실행에도 이미 포함**.

## 2. 실제 위반 구간

최소거리 pair는 PC에서 실행되는 **기체 1–2**이다. Pi0는 조합 공유에
참여하므로 이 사실만으로 Pi 통신의 영향을 완전히 배제할 수는 없다.
이하 시간은 공통 Formation 시작 기준 **bag 관측 시각**이다.
실제 PX4 actuator 적용 시각과 혼동하지 않는다.

| 관측 시각 | 사건 |
|---|---|
|84.4288 s|새 조합이 기체 1·2에서 확정됨. 아직 회피 비활성|
|84.5988 s|기체 1의 pair AD=+0.469 m|
|84.6289 s|기체 1의 pair AD=+0.244 m|
|84.6302 s|기체 2가 AD=-0.879 m로 회피 활성화|
|84.6306 s|기체 2 회피 ROS 명령이 관측됨 (+11.687 m/s²)|
|84.6688 s|기체 1이 AD=-1.153 m로 회피 활성화|
|84.6689 s|기체 1 회피 ROS 명령이 관측됨 (-11.687 m/s²)|
|85.1293 s|새 불안전 후보가 기존보다 약 0.941 m 개선되지만 변경 보류|
|85.3893 s|다음 평가에서는 약 1.903 m 개선되어 proposal 성립|
|85.4089 s|기체 1·2 새 조합 확정, 기체 2가 반대 방향으로 변경|
|86.8 s|관측 최소거리 7.76659 m|

기체 1·2는 활성화 후 이 최소거리까지 계속 active/execute였다.
이번 구간에 '회피 중단 → Formation 복귀 → 충돌 접근' 전이는 없다.
서로 회피 ROS 명령을 내보낸 wall timestamp 차이는 약 **39.826 ms**이며,
과거 문제로 언급됐던 0.42 s 실행 요청 차이와 동일한 사건이 아니다.

## 3. 확인된 기존 변경 억제 규칙

`ManeuverSelectionWorkerCoordination.cpp:170`의 `clearlySuperior()`는
두 조합이 모두 불안전하면 다음 조건을 사용한다.

`proposed.minimum_ad > current.minimum_ad + active_switch_minimum_ad_margin_m`

공통 YAML 값은 **1.0 m**이며 `fd1698b`에도 동일하게 존재한다.

- epoch 7155353239: current=-4.761322, proposed=-3.820239 m.
  개선 0.941083 m < 1 m → 보류.
- epoch 7155353240: current=-6.886677, proposed=-4.983604 m.
  개선 1.903073 m > 1 m → proposal/commit/실행 변경.

이 분기는 기록과 코드가 일치한다. 새 구현 오류라고 표현할 수 없으며,
변경하려면 **불안전 상태의 기동 변경 억제 정책 수정**임을 명시해야 한다.
그 수정이 최저거리 향상을 보장한다는 반사실 결과는 아직 없다.

## 4. 큰 AD 점프의 해석 주의

기체 2의 84.6687 s trace에는 peer candidate=3, AD=-12.339 m가 나온다.
이때 ownship은 이미 active지만 peer1의 active 정보는 아직 반영되지 않아
`buildActivationSample()`은 peer의 Formation intent를 사용한다.
후속 84.6887 s에는 peer candidate=0으로 바뀌고 AD=-0.994 m다.

이는 동일 궤적에 대한 계산이 갑자기 틀렸다는 증거가 아니다. 활성화 전에는
합의된 후보 조합, 활성화 후에는 관측된 실행 조합을 평가하는 기존 분기다.
서로 다른 가정의 AD를 단순 연결하여 모두 통신 손실이나 공분산 부족이라고
해석해서는 안 된다.

또 graph는 공통 epoch 시각/동결 후보 library를 평가하고 activation monitor는
최신 상태 기반 후보를 평가하므로 같은 wall 관측 시각의 두 AD가 동일해야
하는 것은 아니다. 예를 들어 graph epoch 7155353237의 평가시각은
1788838309500000 us이고 후보 source는 약 0.17–0.19 s 전이다.
이는 time-aligned 후보 예측이지 그 과거 구간 실제 입력 이력과 같다는 뜻이 아니다.

## 5. 검증 및 남은 수정 경계

기존 빌드의 관련 regression 4개 통과:

- Ordered inbox wrap/order.
- Ordered inbox concurrent producer.
- Clearly-superior AMAC tuple agreement/commit.
- Component activation edge survives invalid sample/heartbeat.

이 테스트는 해당 코드 계약을 확인할 뿐 DSD 안전성 증명이 아니다.
이번 감사에서 제어 변경·재빌드·새 SILS·commit/push는 하지 않았다.

다음 판단은 다음을 구분해야 한다.

1. 1 m 변경 억제 기준을 유지할 것인지 바꿀 것인지: 정책 선택.
2. 후보를 예측한 시점부터 실제 ROS/PX4 적용 및 선회까지의 residual:
   동일 명령을 실제 수행한 구간에 한정해 비교해야 하는 예측/실행 계약 검증.
3. 캡처 부하와 무선 지터의 영향: 같은 코드로 다른 실행 결과가 나온 이유의
   후보이나, 현재 자료만으로 단일 인과를 확정할 수 없음.

안전거리를 만족한 단 한 실행을 '항상 안전한 버전'으로 취급하거나, 같은
코드를 무작정 롤백하여 7.77 m 문제를 수정했다고 보고하지 않는다.

## 추가 점검: 캡처 배치와 관찰자 부하

사용자의 후속 요청에 따라 캡처 파일을 오프라인 재집계하고 Pi의 파일시스템과
실행 launcher를 확인했다. 제어 변경이나 새 실험은 하지 않았다.

- Pi pcap 크기 185,731,051 bytes, 관측 범위 약 204.135 s.
  파일 크기/시간은 약 0.868 MiB/s. 실제 저장장치 순간 쓰기량을 측정한 값은 아니다.
- Pi 파싱 가능한 비분할 UDP 472,485개 중 목적 belief 메시지는 10,089개,
  약 2.135%. 그 메시지의 UDP payload는 3,026,700 bytes로 전체 파싱된
  UDP payload의 약 1.92%. IPv4 fragment 629개는 이 분모에서 제외했다.
- PC 파일 186,913,420 bytes. 캡처 시작/종료 범위가 Pi와 달라 전체
  파일 개수나 전체 시간당 비율을 곧바로 양쪽 부하 차이로 해석하지 않는다.
- Pi `/tmp`는 `/dev/mmcblk0p2` ext4: tmpfs가 아니라 SD카드 파일시스템이다.
  페이지 캐시에 먼저 쌓일 수 있으므로 매 패킷마다 SD에 동기 쓰기했다는 뜻은 아니다.
- 캡처 명령에는 taskset/nice/ionice 분리가 없었고, 실험 컨테이너 launcher도
  CPU affinity/quota 분리가 없다. 별도 프로세스이지만 공유 CPU/cache/memory/I/O에
  미치는 영향이 0이라는 보장은 없다. 개발 컨테이너 inspect를 실험 컨테이너의
  당시 측정값으로 대신 사용하지 않았다.
- tcpdump 자체는 관측 패킷을 추가 전송하는 것이 아니다. 여기서 지적하는
  비용은 로컬 패킷 복사/버퍼 처리/저장/스케줄링 경쟁이다.

**실시간 영향 최소화 요구에 비추어 캡처 범위와 저장 배치가 과했다.**
하지만 당시 tcpdump CPU 사용률, block-I/O 지연, runqueue, IRQ 부하를 기록하지
않았으므로 포화나 특정 DSD 사건과의 인과는 여전히 미확정이다.

| Pi 관측량 | 캡처 전 실행 | 캡처 실행 |
|---|---:|---:|
|enqueue→dispatch 최대|20.961 ms|18.294 ms|
|입력 drain 최대|12.528 ms|10.236 ms|
|worker pass 최대|16.149 ms|19.138 ms|
|경로 완료 간격 최대|125.439 ms|109.183 ms|

PC 기체 1의 경로 완료 간격 최대는 71.045→75.619 ms, 기체 2는
73.863→72.703 ms다. 전반적 실행 정지가 일관되게 악화됐다는 결과는 아니며,
비행/후보 작업량도 서로 다르므로 이 표로 캡처의 순수 비용을 계산할 수 없다.

우선순위는 1 m 정책 수정이 아니라 **캡처 없이 동일 바이너리·설정으로 비교**다.
캡처가 다시 필요하면 검증된 대상 flow/writer로 범위를 좁히고 짧고 용량 제한된
메모리 저장을 검토한다. CPU 분리도 자동으로 무부하를 보장하지 않는다.
인과 분리에는 반복 A/B가 필요하며 한 번의 회복만으로 원인을 확정하지 않는다.
점검 종료 시 PC/Pi 모두 tcpdump 실행 프로세스는 없었다.
