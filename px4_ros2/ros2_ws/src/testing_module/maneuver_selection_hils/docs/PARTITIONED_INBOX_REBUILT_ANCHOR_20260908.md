# 입력 큐 수정 + 기존 종료 후 계측 보존 기준점

사용자 요청에 따라 입력 큐 수정 버전을 재빌드·재실험한 뒤 커밋으로 보존한다.
이 기준점에는 기존 MASD 실시간 진단과 stopped stage/pipeline 계측 옵션이 포함된다.
패킷 캡처 코드·프로세스는 포함하지 않으며 계측 완전 제거 버전이라고 부르지 않는다.

## 소스와 빌드

- 출발 커밋: `2fbb5ef`; partitioned ordered inbox 및 stopped pipeline 계측 변경 포함.
- PC/Pi Release clean-first 빌드 성공. PC Worker/queue 회귀 57개 통과.
- PC guidance SHA256: `417338b54be753b17d1ecf7f9bfdcbe7d9817071f0d0287a5e144af587b73bab`.
- Pi ELF Build ID: `742a7d17fbf052d5ab049f7f56807d89f11a33a1`.
- PC/Pi include/src/config/scripts/test 집합 SHA256: `722a3a99b9354e8f578170c8826145c81f7c97df893f5b4d91481a1146b64776`.
- Pi 재빌드 이미지: `collision-avoidance:partitioned-inbox-rebuilt-20260908`.
- 이미지 ID: `sha256:9bd3e0a08198ccf7d5686792396866bb27febe0fa1d70f397ccbf080fc0c9c99`.
- 기존 Pi 이미지/개발 컨테이너 보존. 코드/임계값은 재실험 중 변경하지 않았다.

## 200초 실행

실행 ID: `hybrid_formation_rebuilt_no_pcap_200s_20260908_01`.
Pi agent0 + PC agent1–4, Formation pentagon, AD<0, DSD10m, Bcomm0,
V4/filter/formation discrimination OFF, Wi-Fi power save OFF.
기존 MASD 진단 및 stopped timing ON. 패킷 캡처와 동영상 생성 OFF.

- 공통 Formation 평가 181.0초, 기존 10Hz bag 분석 grid.
- 최소 3D 분리 12.15204m, 10m 미만 표본 0개.
- 최종 속도벡터 표준편차 0.22465m/s, 최대 pair 헤딩 차이 0.78577°.
- 최종 위치 표준편차 54.24495m. 마지막 30초에는 45.217→54.245m로 증가.
- 안전거리 및 방향·속도 정렬은 확인. 고정된 대형 간격의 완전한 수렴은 미확인.
- Pi 평균 완료율 경로19.99098Hz/선택3.99893Hz; 최대 완료간격124.64080ms/319.79897ms.
- 평균 주기 충족을 매 주기 마감시간 준수나 hard-real-time 보장으로 해석하지 않는다.

원자료는 HILS `result/summary/<실행 ID>/`의 `summary.json`, `run_manifest.json`,
`formation_tail_metrics.json`, `vehicle_0_pipeline_timing.json`, `run_notes.md`와
`result/plot/<실행 ID>/actual_maneuver_overview.png`에 로컬 보존한다.
대형 rosbag·이미지·생성 결과는 기존 result ignore 정책을 유지하고 이 추적 문서를 커밋한다.

동일 실행파일의 이전 무캡처 반복에서 다른 수렴 결과가 관찰됐다.
따라서 이 커밋은 복구 가능한 실험 기준점이지 모든 실행의 안전성/수렴 보증이 아니다.
