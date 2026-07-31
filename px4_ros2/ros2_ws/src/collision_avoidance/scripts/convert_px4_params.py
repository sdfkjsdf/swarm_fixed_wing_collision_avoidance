#!/usr/bin/env python3
"""
convert_px4_params.py
─────────────────────
QGC 에서 export 한 PX4 파라미터 파일 (.params) 을 읽어서
우리 collision_avoidance 패키지가 쓰는 airframe_spec.yaml 로 변환.

사용법:
    1. QGC → Vehicle Setup → Parameters → Tools → Save to file
       → px4_params_exported.params 같은 이름으로 저장
    2. 이 스크립트 실행:
         python3 convert_px4_params.py <input.params> [output.yaml]
       기본 output: ../config/airframe_spec.yaml

예시:
    python3 convert_px4_params.py ~/Downloads/qgc_params.params
    python3 convert_px4_params.py ~/Downloads/qgc_params.params custom_spec.yaml

변환 매핑:
    PX4 파라미터                 →  우리 yaml 필드
    ─────────────────────────────────────────────────────────────
    FW_AIRSPD_MIN                →  airspeed_min
    FW_AIRSPD_TRIM               →  airspeed_cruise
    FW_AIRSPD_MAX                →  airspeed_max
    FW_T_CLMB_MAX                →  height_rate_max_climb
    FW_T_SINK_MAX                →  height_rate_max_sink
    FW_R_LIM                     →  max_roll_deg  (참조용)
    (PX4 에 대응 없음)            →  accel_max_horizontal (우리가 정함, default 5.0)
    (PX4 에 대응 없음)            →  course_min_speed     (우리가 정함, default 1.0)

QGC .params 파일 형식:
    # 주석
    # Vehicle-Id Component-Id Name Value Type
    1 1 FW_AIRSPD_MIN 10.000000 9
    1 1 FW_AIRSPD_MAX 20.000000 9
    ...
"""

import sys
import os
from pathlib import Path


# 추출할 PX4 파라미터 → 우리 yaml 필드 매핑
PX4_TO_YAML = {
    "FW_AIRSPD_MIN":   ("airspeed_min",            "최소 운용 airspeed (stall 여유)"),
    "FW_AIRSPD_TRIM":  ("airspeed_cruise",         "순항 airspeed"),
    "FW_AIRSPD_MAX":   ("airspeed_max",            "최대 운용 airspeed"),
    "FW_T_CLMB_MAX":   ("height_rate_max_climb",   "최대 climb rate [m/s]"),
    "FW_T_SINK_MAX":   ("height_rate_max_sink",    "최대 sink rate [m/s]"),
    "FW_R_LIM":        ("max_roll_deg",            "최대 roll [deg] (참조용)"),
}

# PX4 에 대응이 없어서 우리가 정하는 값들 (Spherical pipeline 기준)
OUR_DEFAULTS = {
    "max_pitch_deg":               (30.0, "[deg] |γ| 한계 (Spherical state pitch 자유도)"),
    "max_pitch_rate_deg_per_sec":  (30.0, "[deg/s] dγ/dt 한계 (pitch angular rate)"),
}


def parse_params_file(path: Path) -> dict:
    """QGC .params 파일을 읽어서 {name: value} 딕셔너리 반환."""
    result = {}
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            tokens = line.split()
            if len(tokens) < 5:
                continue
            # Format: vehicle_id component_id name value type
            name = tokens[2]
            try:
                value = float(tokens[3])
            except ValueError:
                continue
            result[name] = value
    return result


def generate_yaml(params: dict, output_path: Path):
    """추출한 PX4 값 + 우리 기본값을 airframe_spec.yaml 로 씀."""

    lines = []
    lines.append("# Airframe spec — PX4 파라미터 + 우리 연구 safety limit")
    lines.append("# 자동 생성됨 by scripts/convert_px4_params.py")
    lines.append("# source: QGC 에서 export 한 .params 파일")
    lines.append("# ")
    lines.append("# 수동 편집 시 주의: 스크립트 재실행하면 덮어써짐")
    lines.append("")
    lines.append('"/**":')
    lines.append("  ros__parameters:")
    lines.append("")
    lines.append("    # ═══════════════════════════════════════════════════════════")
    lines.append("    # PX4 airframe 에서 읽어온 값 (QGC 과 일치)")
    lines.append("    # ═══════════════════════════════════════════════════════════")

    for px4_name, (yaml_field, comment) in PX4_TO_YAML.items():
        if px4_name in params:
            value = params[px4_name]
            lines.append(f"    {yaml_field}: {value:.4f}  # PX4 {px4_name} — {comment}")
        else:
            # 파라미터가 .params 파일에 없으면 경고 주석
            lines.append(f"    # {yaml_field}: <not found>  # PX4 {px4_name} — {comment}")

    lines.append("")
    lines.append("    # ═══════════════════════════════════════════════════════════")
    lines.append("    # PX4 에 대응 파라미터 없음 — 우리가 직접 정함")
    lines.append("    # ═══════════════════════════════════════════════════════════")

    for field, (default, comment) in OUR_DEFAULTS.items():
        lines.append(f"    {field}: {default}  # {comment}")

    lines.append("")

    with open(output_path, "w") as f:
        f.write("\n".join(lines))

    print(f"[OK] airframe_spec.yaml 생성: {output_path}")

    # 찾지 못한 PX4 param 경고
    missing = [name for name in PX4_TO_YAML if name not in params]
    if missing:
        print(f"[WARN] .params 파일에서 찾지 못한 PX4 파라미터:")
        for name in missing:
            print(f"         - {name}")
        print(f"       해당 필드는 yaml 에 주석 처리됨. 수동으로 추가하거나")
        print(f"       QGC 에서 해당 파라미터 활성화 후 재export 바람.")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    input_path = Path(sys.argv[1]).expanduser().resolve()
    if not input_path.exists():
        print(f"[ERROR] 입력 파일 없음: {input_path}")
        sys.exit(1)

    if len(sys.argv) >= 3:
        output_path = Path(sys.argv[2]).expanduser().resolve()
    else:
        script_dir = Path(__file__).parent.resolve()
        output_path = (script_dir / ".." / "config" / "airframe_spec.yaml").resolve()

    output_path.parent.mkdir(parents=True, exist_ok=True)

    print(f"[..] reading {input_path}")
    params = parse_params_file(input_path)
    print(f"[..] parsed {len(params)} PX4 parameters")

    generate_yaml(params, output_path)


if __name__ == "__main__":
    main()
