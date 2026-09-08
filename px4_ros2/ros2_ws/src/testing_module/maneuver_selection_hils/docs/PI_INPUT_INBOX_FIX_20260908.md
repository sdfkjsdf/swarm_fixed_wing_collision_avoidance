# Pi 입력 큐 overflow — 정적 원인과 수정 범위

## 확인된 사실

대상 기록: `hybrid_formation_wifi_ps_off_200s_20260908_01`.
Pi(node 0)의 누적 누락은 ownship belief 82, remote intent 880,
remote decision 134이며 다른 네 노드의 해당 세 카운터는 0이다.
이 세 값의 합은 모든 입력 종류를 포함한 총 누락 수가 아니다.

`guidance_0.log`의 첫 remote-intent queue-full 경고는
`1788834841.014735259`이고 직후 belief/remote-decision 경고가 뒤따른다.
`1788834940.136651611`에는 실제 발행 명령의 history queue-full 경고도 있다.
이는 DDS에서 가져온 뒤 애플리케이션 enqueue가 실패한 기록이다.
DDS reliable 설정만으로 이 내부 누락이 복구되지는 않는다.

## 직접적인 코드 경로

1. 기존 Worker는 자기 상태·airspeed·nominal/실제 발행 명령과 네 상대의
   후보·결정을 **동일한 64칸 SPSC FIFO**에 넣었다.
2. Worker는 입력을 하나 pop한 뒤 `acceptRemoteIntent()`에서 후보 궤적 복원과
   공분산 전파를 수행했다. 나머지 입력은 계산이 끝날 때까지 슬롯을 점유했다.
3. 상대 입력이 공용 슬롯을 채우면 자기 상태와 실제 발행 명령도 거부됐다.
4. 실제 발행 명령 누락은 `m_published_input_history_lost`를 설정한다.
   Worker는 명령 이력과 `m_has_latest_state`를 무효화한다. 이후 과거 EKF 상태를
   현재로 보정할 실제 명령이 없으면 후보 갱신도 진행할 수 없다.

네 상대가 각각 7개 후보를 20 Hz로 보내면 remote intent만 명목상 560개/s다.
250 ms 분량이 한꺼번에 밀리면 140개이므로 기존 공유 64칸보다 많다.
이는 용량 취약성을 설명하는 산술 예시이며, 실제 모든 burst가 정확히
250 ms였다는 측정 결과는 아니다. 최초 burst의 원인이 Wi-Fi 절전, DDS,
스케줄링 중 무엇인지는 이 기록만으로 확정하지 않는다.

## 직접 수정

- 공용 입력 FIFO를 제거하고 `OrderedSpscInbox`로 대체한다.
- 자기 입력 1개 partition + 상대별 4개 partition, 각각 64칸을 예약한다.
  한 상대의 burst가 자기 입력이나 다른 상대의 공간을 소모하지 않는다.
- 단일 ROS executor producer가 부여하는 ingress sequence로 전체 수신 순서를
  보존한다. 후보 우선/자기 상태 우선 재정렬이나 최신값 덮어쓰기는 하지 않는다.
- 회차 시작 시 공개된 sequence cutoff까지만, 최대 64개를 미리 꺼내 고정 batch에
  보관한다. 슬롯을 먼저 반환한 뒤 기존 accept 함수들을 순서대로 실행한다.
- 저장 공간은 생성자에서 한 번 할당한다. push/drain 중 동적 할당·잠금·파일 기록·
  추가 통계 연산은 없다. head 탐색은 최대 64×5회로 제한된다.
- 입력 저장 공간은 증가하지만 회차당 복원/전파 개수 상한은 64개로 유지된다.
  무제한 큐, 전체 backlog 무제한 처리, 별도 우회 제어 경로는 추가하지 않는다.
- 실제 local partition overflow 시 기존 누락 카운터와 명령 이력 무효화는 유지한다.

## 변경하지 않는 계약

AD<0 activation, CPA 종료, 7개 후보, 20 Hz/4 Hz 설정, candidate library의
epoch/source timestamp 검증, batch 완성/freeze, proposal/commit 및 PX4 명령 로직은
변경하지 않는다. 계측은 기존 stopped-timing 옵션을 그대로 사용한다.
enqueue→dispatch 시간에는 큐 대기뿐 아니라 미리 꺼낸 batch 내부의 처리 대기도 포함된다.

단일 producer/단일 consumer 전제는 현재 `vtol_guidance_main.cpp`의
`rclcpp::spin(node)` 및 Worker 전용 thread와 일치한다. 향후 concurrent callback
executor로 변경하면 이 전제를 재검토해야 한다.

## 검증 및 한계

회귀 테스트는 partition 격리, 전체 FIFO/링 wrap, 10만 개 동시 송수신,
회차당 64개 상한, 5개 ownship ID 각각의 peer partition 포화에도 실제 명령·
belief가 보존되는지, 기존 실제 명령 누락 시 fail-closed 동작을 확인한다.

로컬 검증 완료: `Release (-O3 -DNDEBUG)` guidance 노드 및 대상 테스트 빌드 성공.
Worker/queue 57개, ROS runtime 2개(5-node 합의 포함), 기존 offline timing parser
8개가 통과했다. `git diff --check`도 통과했다. 초기 구현의 객체 내 큰 버퍼는
다중 Worker 테스트에서 스택 고갈을 유발하여 생성자 1회 할당으로 수정했고,
기존 스택 크기를 변경하지 않고 전체 대상 테스트가 통과했다.

정적 확인: 공용 FIFO 참조 제거, 모든 push의 새 저장 경로 연결,
동일 ingress 순서 유지, 기존 accept/검증 함수와 제어 조건 유지,
실제 local 명령 누락 시 fail-closed 보존을 확인했다.
Pi 소스/이미지 배포, install 갱신, 새 SILS 및 커밋/push는 수행하지 않았다.

## 후속 최종 정적 검토 및 실험 착수

사용자의 재검토·재실험 요청 후 위 경계를 다시 확인했다. partition head는
consumer가 pop하기 전 producer가 재사용할 수 없고, release/acquire로 공개한
sequence cutoff 이하의 입력만 병합하므로 partition 간 순서가 뒤집히지 않는다.
실패한 enqueue에는 공개 sequence를 부여하지 않는다. 실제 명령 이력 누락의
fail-closed 분기, 모든 accept 함수, 제어 조건은 유지된다.
추가 결함은 발견하지 못했으며, 57 Worker/queue + 2 ROS runtime 테스트를
다시 통과했다. 이 검토는 모든 실행 조건에서의 무결함 증명은 아니다.

동일 수정 소스를 PC/Pi에 반영하고 양쪽 Release 빌드를 확인했다. 소스 해시,
실행 파일, 이미지, 조건은 새 실험의 `run_manifest.json`에 기록한다.
실험 ID: `hybrid_formation_partitioned_inbox_200s_20260908_01`.
기존 이미지와 Pi 호스트 Git/개발 컨테이너는 보존하고 별도 실험 이미지를 쓴다.

후속 실행 완료: 200초 기록(공통 Formation 평가180.5초), 모든 기체의 보고된
상태/후보/결정 입력 누락0, 계측 기록 손실0. 최소 관측3D 거리10.29373m,
10Hz 평가상 DSD 위반0. Pi 평균 갱신19.99180Hz/선택3.99891Hz,
최대 완료 간격125.439ms/297.925ms이므로 매 주기50ms/250ms 충족은 아니다.
자세한 조건·비교·한계는 해당 실험 `run_notes.md`에 기록했다.

이 수정은 수신 burst를 처리하는 내부 계약의 수정이다. 지속 유입량이 처리량을
초과하거나 한 partition 자체가 포화되는 상황까지 무손실을 보장하지 않는다.
Pi의 200초 무누락 및 매 주기 마감시간 충족 여부는 새 바이너리로 재시험해야 한다.
이번 변경만으로 Wi-Fi 원인 규명 또는 DSD 안전 성능 개선을 주장하지 않는다.
