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
수행한다. 기본 반복 검증은 다음 명령으로 시작한다.

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
