# 계측 변경 이력과 실행 경로 점검 — 2026-09-08

## 목적과 범위

사용자 요청: 계측 추가 때 성능이 악화된 이유를 Git 이력부터 점검한다.
이번 점검에서는 제어 코드, 파라미터, Pi 배포물에 변경하지 않았으며 새 SILS를 실행하지 않았다.
커밋뿐 아니라 보관된 미커밋 패치, 현재 작업 트리, 기존 실행 결과를 함께 비교했다.

## 결론

1. 지금까지의 변경을 모두 ‘기록만 추가’로 설명하면 부정확하다. 계측 외에 입력 큐, 회차별 처리량, 출력 전달 순서의 변경이 포함됐다.
2. 현재 AD/CPA/coordination 판단 수식이 계측 값에 의존하도록 바뀐 증거는 발견하지 못했다. 그러나 비동기 실행에서 입력을 언제 처리하고 결정을 언제 전달하는지는 달라질 수 있다.
3. 최초 실시간 stage 계측은 제거됐다. 현재 stopped-only 계측과 혼동하면 안 된다.
4. 별도의 기존 MASD 진단 발행은 여전히 비행 중 실행된다. 패킷 캡처를 끈 실험도 무계측 실험이 아니다.
5. 같은 실행파일의 반복 결과도 달라졌다. 특정 패치 또는 tcpdump가 최소거리 악화의 단독 원인이라는 인과관계는 아직 입증되지 않았다.

## 변경 이력

|시점/버전|실제 변경|현재 상태와 판단|
|---|---|---|
|`a292fc6`|MASD budget trace 계측, 실제 발행 명령 이력을 이용한 EKF 과거 상태 보정 등|좋았던 `fd1698b` 이전부터 존재. 실제 명령 이력은 제어 기능이므로 계측과 함께 제거하면 안 됨|
|`fd1698b`|component activation 전달 수정, 기동 선택을 AD 목적에 맞춤|비교 기준 커밋|
|9월 6일 최초 stage 계측, 미커밋|실시간 ROS stage 메시지 추가, 기존 trace 큐 사용, 출력 콜백 내 진단 처리 순서 변경|stash에 보존 후 제거. 단순한 수동 기록만은 아니었음|
|`b1e15d4`|고정 크기 메모리에 worker 단계 시각 저장, worker 정지 후 출력|실시간 새 DDS 발행은 없음. 시계 조회·메모리 기록 비용은 남음|
|`2fbb5ef`|입력/출력 큐를 시작 시점의 개수까지만 처리, 오래된 입력 조기 배제, rejoin 메타데이터 모순 수정, INFO→DEBUG|계측만이 아닌 기능·스케줄링 수정. 무한 drain 방지에는 의미가 있으나 동시 도착 입력의 처리 회차가 달라짐|
|현재 미커밋 pipeline 계측|콜백·입력 전달·복원 처리 시각 기록, 고정 버퍼 확장|종료 후 출력. 약 10.25 MiB/node 버퍼 추가 및 시계 조회 비용. 실시간 DDS stage 발행은 추가하지 않음|
|현재 미커밋 ordered inbox|공유 64-slot 입력 큐를 local/peer별 64-slot, 총 320-slot으로 분리; 최대 64개를 먼저 꺼내고 처리|실제 입력 누락을 고친 변경. 수락된 입력의 전역 순서는 유지하지만 실시간 혼잡/수락 결과가 과거와 반드시 같지는 않음|
|dual tcpdump|PC/Pi에서 UDP 캡처와 파일 기록|외부 부하. 현재 중단. 위 소스와 실행파일은 이후 무캡처 재실험에서도 동일|

최초 제거 패치는 `stash@{0}`의 `d2cfa62c53d9ab65b513ba71bc0bd0bac913d41b`와
`result/summary/hybrid_formation_stage_timing_200s_20260906_01/source_changes.patch`에 보존돼 있다.
일반 `git log`만 보면 이 실험의 변경은 보이지 않는다.

## 현재 코드에서 확인된 시간 의존성

### 진단 발행이 제어 결정 전달 앞에 있음

`collision_avoidance/src/communication/DistributedManeuverSelectionRuntime.cpp`의
`drainWorkerOutput()`은 다음 순서다.

1. 시작 시 pending trace/output 개수를 저장한다(411–412행).
2. MASD trace를 ROS/DDS로 발행한다(413–416행).
3. 후보 궤적, graph 진단, 분산 decision을 발행한다.
4. 마지막에 local decision callback을 호출한다(883–884행).

따라서 진단 발행의 직렬화·DDS 호출 시간이 local decision 전달 앞에 놓여 있다.
이는 정적으로 확인되는 실행 의존성이지, 그 시간이 실제 몇 ms였다는 측정 결과는 아니다.
ROS 노드는 단일 executor로 spin하며 출력 timer의 명목 주기는 10 ms다.

특히 시작 시 output 개수가 0이고 trace 처리 중 새 decision이 도착하면,
현재 회차에서는 처리하지 않고 다음 출력 콜백 기회로 넘어갈 수 있다.
이것은 bounded drain의 트레이드오프이며, 무조건 잘못된 로직이라고 단정하지 않는다.
그러나 ‘제어 수식 불변’을 근거로 ‘제어 전달 타이밍 불변’까지 주장할 수는 없다.

### 무캡처 실행에도 남아 있는 진단

최신 `hybrid_formation_no_pcap_recheck_200s_20260908_01` bag에는
5대 합계 MASD budget trace 101,364건, 기록 구간 기준 약 561건/s가 있다.
직렬화된 trace payload 합계는 약 19.87 MB다.
이는 bag 레코드 수/크기이며 DDS 패킷 수나 네트워크 총 대역폭으로 해석하면 안 된다.
성공했던 stopped-timing 실행에도 같은 종류의 trace가 약 595건/s 있었으므로,
이 진단이 새로 생겨 성능 악화를 일으켰다고 단정할 수도 없다.

현재 stage/pipeline timing 자체는 종료 후 저장한다. 실시간 MASD trace와 별개다.
`FormationMode::recordPublishedSetpoint()`는 상태 보정에 필요한 실제 입력 이력이고,
`traceSetpoint()`는 관측용 발행이므로 향후 정리할 때도 구분해야 한다.

### 큐 변경은 단순 관측 변경이 아님

`ManeuverSelectionWorker::processPending()`은 현재 최대 64개 입력을 batch로 먼저
분리한 뒤 각 입력을 처리한다. 과거에는 하나를 pop하고 복원 연산을 하는 동안
나머지 입력이 큐 공간을 계속 점유했다. 새 구조는 공간을 먼저 확보해 누락을 줄인다.
이 수정의 목적은 타당하지만 라이브 입력 수락과 처리 시점까지 동일하게 만드는 변경은 아니다.

## 결과 비교

아래 값은 기존 200초 설정의 hybrid Formation pentagon 실행이다.
거리는 분석 grid에서 관측한 최소 3D 분리이며 연속시간 안전 보증이 아니다.
마지막 속도 표준편차는 5대의 속도벡터 분산 정도다. 작을수록 속도 정렬이 좋지만 단독으로 모든 편대 조건을 증명하지는 않는다.

|실행 suffix|최소 분리 m|종료 시 속도 표준편차 m/s|
|---|---:|---:|
|`stage_timing_200s_20260906_01` — 제거된 최초 계측|10.419|13.771|
|`fd1698b_restored_200s_20260906_01`|11.962|0.612|
|`stopped_timing_200s_20260906_01`|11.665|0.338|
|`bounded_input_200s_20260908_01`|12.611|0.244|
|`pipeline_timing_200s_20260908_01`|11.651|0.477|
|`wifi_ps_off_200s_20260908_01`|9.025|22.261|
|`partitioned_inbox_200s_20260908_01`|10.294|0.639|
|`dual_pcap_200s_20260908_01`|7.767|1.613|
|`no_pcap_recheck_200s_20260908_01`|10.847|22.632|

각 suffix 앞에 `hybrid_formation_`이 붙는다.
직전 3건은 같은 partitioned-inbox 소스/실행파일을 사용했다.
무캡처에서 최소거리는 회복됐지만 편대 수렴까지 회복된 것은 아니다.
따라서 ‘캡처 제거로 원인 해결’도, ‘계측 추가마다 반드시 악화’도 데이터와 정확히 일치하지 않는다.

Pi 캡처는 약 185.7 MB를 약 204초 동안 SD 카드 위 `/tmp`에 기록했다.
전체 UDP 중 대상 belief 패킷은 약 2.1%였다. 관측 목적보다 넓은 캡처였고
CPU/I/O 경쟁 가능성은 있지만 당시 CPU/iowait/IRQ 기여도를 측정하지 않아 원인으로 확정할 수 없다.
kernel capture drop 0은 부하 0을 뜻하지 않는다.

## 검증과 버전 관리의 한계

- `BudgetTracingDoesNotChangeControlResults` 테스트는 고정 입력 순서/시각에서 계측 ON/OFF의 계산 결과를 비교한다. DDS 도착 순서와 스케줄링 부하가 달라지는 폐루프 성능까지 보장하지 않는다.
- `fd1698b` 이후 core AD/CPA/coordination/Formation 제어법을 바꾼 diff는 이번 범위에서 확인되지 않았다. Worker 및 runtime의 전달/처리 방식은 변경됐다.
- 현재 HEAD는 `2fbb5ef`지만 tracked 7개 파일 변경과 새 inbox 파일 등이 미커밋이다. HEAD 이름만으로 실험 버전을 식별하면 안 된다.
- 최신 PC guidance SHA-256은 `417338b54be753b17d1ecf7f9bfdcbe7d9817071f0d0287a5e144af587b73bab`이다.
- Pi image는 `collision-avoidance:partitioned-inbox-20260908`, image ID는 `sha256:f07f9e1768e005f6e4f68c7ee86edff4964cd5daa2096a5307f8bdcdc1eca1be`다.
- 실제 hybrid overlay는 `/home/hmcl/workspace/swarm-fixed-wing/ros2_ws/install`이다. repo 내부의 별도 install을 비교 대상으로 혼동하면 안 된다.

## 권고 — 이번 점검에서는 적용하지 않음

1. 제어/통신 기능과 관측용 발행을 분리해 변경 목록을 관리한다. 입력 이력·candidate intent·coordination decision을 진단으로 오인해 제거하지 않는다.
2. 가장 먼저 검증할 정적 경계는 local decision 전달 앞의 관측용 DDS 발행이다. 향후 변경 시 합의에 필요한 발행 순서를 보존하고, 관측 실패가 제어를 지연시키지 않는 계약을 명시한다.
3. 기존 동일 실행파일에서 MASD trace와 stopped timing을 각각 독립적으로 ON/OFF 비교해야 부하의 기여도를 분리할 수 있다. 큐 변경, threshold 변경, 캡처 추가를 동시에 섞지 않는다.
4. 입력 누락 수정 전체를 되돌리거나 AD margin을 바꾸는 것은 이번 결과에서 정당화되지 않는다.
5. 현재로서는 안전거리와 편대 수렴의 반복 재현성이 미확인이다. 한 번의 좋은 결과를 안정성 합격으로 일반화하지 않는다.

이번 산출물은 이 점검 문서뿐이며 runtime 변경·빌드·배포·새 실험·커밋은 수행하지 않았다.
