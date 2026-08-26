# Trajectory prediction HILS results

이 디렉터리는 자동 생성되는 SILS/rosbag 검증 결과의 고정 루트이다.

```text
result/
├── rosbag/<batch_id>/<case_id>_rNN/   # 원본 rosbag2
├── raw/<batch_id>/<case_id>_rNN/      # bag에서 사후 생성한 CSV, NPZ, JSON
├── plot/<batch_id>/<case_id>_rNN/     # case별 PNG와 plot_manifest.json
├── summary/<batch_id>/                 # cases.csv, batch_summary.json, 비교 PNG
└── log/<batch_id>/                     # case별 launch 로그
```

결과 데이터는 용량이 크고 재생성 가능하므로 Git에 포함하지 않는다. 각 하위 폴더의
`.gitkeep`만 형상 관리한다.

실행 중에는 rosbag만 기록하고, 전체 수집이 끝난 뒤 자동으로 오프라인 분석과 plot을
수행한다. 정식 coverage 분석은 공통 NED로 평행이동된 `/common/px4_N/...` topic을
사용한다. 분석기의 기본값도 `--coordinate-frame common`이며, `local` 모드는 공통
좌표 topic이 없는 과거 bag을 진단할 때만 사용한다.

기본 반복 검증은 다음 명령으로 시작한다.

```bash
./scripts/run_cone_scenarios.sh --profile coverage_core
```

실행 없이 계획만 확인하려면 `--dry-run`을 추가한다.

수집과 분석을 완전히 분리하려면 다음과 같이 실행한다.

```bash
./scripts/run_cone_scenarios.sh --profile coverage_core \
  --batch-id coverage_01 --collect-only
./scripts/process_cone_batch.sh --batch-id coverage_01
```

시나리오와 반복 횟수는 `config/cone_scenario_matrix.yaml`에서만 관리한다. 셸
스크립트에 case ID를 다시 나열하지 않는다.

동역학 전파 자체를 점검할 때는 replay batch보다 FixedWing 트림 후 ZOH 전용 노드를
먼저 사용한다.

```bash
TEST_CASE_ID=ZOH_ALATP2P628_V20 \
TEST_V_CMD=20.0 TEST_ALAT_CMD=2.628 \
./scripts/launch_propagation_test.sh --record-bag
```

이 경로는 전환 후 실측 CAS·수직속도·roll이 2초간 정착된 뒤 수평 입력을 적용하고,
`trajectory_prediction_debug`도 기록한다. plot에서는 horizon 0 절대 정렬과
start-aligned 전파 오차를 분리한다. 상세 계약과 smoke 결과는
`md_file/TRAJECTORY_CONE_ROSBAG_PIPELINE.md`를 따른다.

대표 궤적 그림은 prediction과 ground truth에 동일한 sample index marker를 표시한다.
회색 점선은 같은 `i`와 같은 `time_offsets_s[i]`의 두 점을 연결하며, 시작·종점은 별도
marker로 표시한다. `i, t` 표기는 궤적 선과 겹치지 않도록 같은-index 연결점 아래에
고정된 화면 offset으로 배치한다. 따라서 서로 다른 시각의 가까운 점을 같은 시각의
오차로 오해하지 않아야 한다. `trajectory_cone_example.png`는 EKF 초기 오차를 포함한
절대좌표 비교이고, `trajectory_start_aligned.png`는 각 시작점을 제거한 순수 전파
비교다. `trajectory_vertical_error.png`는 절대 및 start-aligned 고도 오차를 별도로
표시한다.

`trajectory_cone_3d.png`는 선택한 시간 단면마다 full 3x3 위치 공분산을 고유분해하고
principal symmetric square root로 재구성하여 실제 nominal 95% 3D covariance ellipsoid
표면을 그린다. `trajectory_cone_mesh.png`는 동일한 3D ellipsoid mesh를 East/North 평면에서 본
직교투영이다. 더 이상 예측 궤적에 수직인 평면을 임의로 선택하지 않는다. 시간 방향
generator는 모든 시점에 동일한 unit-sphere direction을 적용한 표면점들을 연결하는
렌더링 선일 뿐이다. 따라서 전체 궤적이 동시에 95% 확률로 머무는 confidence tube를
뜻하지 않으며, holdout calibration을 그림 자체가 입증하지도 않는다. Auto ACAS Figure
8의 시간에 따라 성장하는 uncertainty-envelope 개념에서 시각적 영감을 얻었지만, 해당
출처가 단면 형상이나 mesh topology를 정의한 것은 아니다. 실제 형상은 이 프로젝트가
propagation한 full position covariance로 계산하며 논문 내부의 전용 cone 수학을 복제한
것은 아니다. 특히 Auto ACAS의 MASD나 Figure 8에 열거된 전체 uncertainty source를
재구성한 결과가 아니다. 3D 그림은 East/North/altitude의 물리 축 비율을 수직 과장 없이
유지한다.
`plot_manifest.json`에는 ellipsoid의 polar/azimuth 표본, 시간 단면·generator 개수와
기록된 `time_offsets_s`로 검증한 평균
궤적·공분산의 공통 적분 주기 및 최대 간격 편차를 함께 저장한다. `predict_call_hz`는 새
cone 생성·발행 주기이므로 horizon 내부 적분 주기와 구분한다.

회전 가능한 3D HTML은 전체 batch에 자동 생성하지 않는다. 검토할 run 하나를 선택한 뒤
다음 명령으로만 명시적으로 생성한다.

```bash
python3 analysis/plot_selected_trajectory_cone_html.py \
  --input result/raw/<batch_id>/<run_id> \
  --output result/plot/<batch_id>/<run_id>/trajectory_cone_interactive.html \
  --title "<run_id>"
```

이 그림은 각 horizon의 full 3x3 위치 공분산에서 경로 횡단 방향의 ellipsoid support
point를 구하고, 인접 horizon의 support ring을 연결한 표시용 swept shell이다. 마지막
ring은 최종 pointwise ellipsoid의 전방-normal half cap과 동일한 vertex를 공유한다.
내부 pointwise ellipsoid surface는 반복해서 그리지 않는다. 이는 정확한 연속시간
ellipsoid 합집합 경계나 전체 궤적에 대한 simultaneous 95% confidence tube가 아니다.
Auto ACAS Figure 8의 시간 성장 envelope에서 시각적 아이디어만 차용했으며 MASD나
논문 고유 uncertainty roll-up을 재구성한 결과가 아니다. Plotly를 고정 버전 CDN에서
읽으므로 생성 자체는 오프라인이지만 결과 HTML을 처음 열 때는 네트워크 연결이 필요하다.
HTML 생성기는 `process_cone_batch.sh`에서 호출하지 않으므로 선택하지 않은 run에는
추가 결과를 만들지 않는다. 특정 cone sample을 직접 보려면 `--array-index N`을 사용한다.

전용 반복 검증 matrix는 다음 명령으로 확인한다.

```bash
./scripts/run_propagation_scenarios.sh --list-profiles
./scripts/run_propagation_scenarios.sh \
  --profile model_identification --dry-run
```

실행 순서는 `wiring_smoke → model_identification → model_holdout → 모델 고정 →
cone_calibration → cone_holdout`이다. calibration과 holdout은 case와 Gazebo seed가
분리되어 있으며, model holdout 전에는 Q 보정을 진행하지 않는다. 운용 회피 후보는
속도와 횡가속도만 사용하므로 matrix resolver가 모든 case의 `h_dot_cmd=0`을 강제한다.
선회 중 실제 고도 변화는 예측 상태와 cone에서 계속 평가한다.
