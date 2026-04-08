#!/bin/bash
# 사용법: ./launch_5vtol.sh [show|noshow] (기본값: noshow)
# 스폰 위치는 config/spawn_config.yaml에서 읽음

GUI_MODE=${1:-noshow}
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CONFIG_FILE="${SCRIPT_DIR}/../config/spawn_config.yaml"
ROS_PARAMS_FILE="${SCRIPT_DIR}/../config/ros_params.yaml"

if [ ! -f "$CONFIG_FILE" ]; then
    echo "ERROR: config file not found: $CONFIG_FILE"
    exit 1
fi
if [ ! -f "$ROS_PARAMS_FILE" ]; then
    echo "ERROR: ros params file not found: $ROS_PARAMS_FILE"
    exit 1
fi

# ========== yaml 2개 일치 여부 검증 ==========
NUM_VEHICLES=$(grep "^total_agent_num:" "$CONFIG_FILE" | awk "{print \$2}")
ROS_NUM=$(grep "total_agent_num:" "$ROS_PARAMS_FILE" | awk "{print \$2}")

if [ "$NUM_VEHICLES" != "$ROS_NUM" ]; then
    echo "WARNING: total_agent_num mismatch! spawn_config=${NUM_VEHICLES}, ros_params=${ROS_NUM}"
    echo "Please check both yaml files!"
    exit 1
fi

# spawn_config.yaml에서 오프셋 읽기
declare -a SPAWN_X SPAWN_Y SPAWN_Z
for i in $(seq 0 $((NUM_VEHICLES-1))); do
    line=$(grep "px4_${i}:" "$CONFIG_FILE")
    SPAWN_X[$i]=$(echo "$line" | sed "s/.*x: *\([0-9.]*\).*/\1/")
    SPAWN_Y[$i]=$(echo "$line" | sed "s/.*y: *\([0-9.]*\).*/\1/")
    SPAWN_Z[$i]=$(echo "$line" | sed "s/.*z: *\([0-9.]*\).*/\1/")
done

# ros_params.yaml에서 오프셋 읽기
ROS_X=($(grep "spawn_offset_x:" "$ROS_PARAMS_FILE" | sed "s/.*\[//;s/\]//;s/,/ /g"))
ROS_Y=($(grep "spawn_offset_y:" "$ROS_PARAMS_FILE" | sed "s/.*\[//;s/\]//;s/,/ /g"))
ROS_Z=($(grep "spawn_offset_z:" "$ROS_PARAMS_FILE" | sed "s/.*\[//;s/\]//;s/,/ /g"))

# 각 기체별 오프셋 값 비교
MISMATCH=0
for i in $(seq 0 $((NUM_VEHICLES-1))); do
    if [ "${SPAWN_X[$i]}" != "${ROS_X[$i]}" ] || \
       [ "${SPAWN_Y[$i]}" != "${ROS_Y[$i]}" ] || \
       [ "${SPAWN_Z[$i]}" != "${ROS_Z[$i]}" ]; then
        echo "WARNING: px4_${i} offset mismatch!"
        echo "  spawn_config: x=${SPAWN_X[$i]}, y=${SPAWN_Y[$i]}, z=${SPAWN_Z[$i]}"
        echo "  ros_params:   x=${ROS_X[$i]}, y=${ROS_Y[$i]}, z=${ROS_Z[$i]}"
        MISMATCH=1
    fi
done

if [ "$MISMATCH" = "1" ]; then
    echo "ERROR: Offset mismatch between yaml files! Please fix and re-run."
    exit 1
fi

echo "=== Spawn Config (validated) ==="
for i in $(seq 0 $((NUM_VEHICLES-1))); do
    echo "  px4_${i}: x=${SPAWN_X[$i]}, y=${SPAWN_Y[$i]}, z=${SPAWN_Z[$i]}"
done

# ========== 시뮬레이션 시작 ==========
cd ~/PX4-Autopilot

pkill -9 gz; pkill -9 gzserver; pkill -9 gzclient; pkill -9 px4
sleep 2

source Tools/simulation/gazebo-classic/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default

if [ "$GUI_MODE" = "show" ]; then
    gazebo Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world --verbose &
else
    gzserver Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world --verbose &
fi
sleep 5

build_path=~/PX4-Autopilot/build/px4_sitl_default
src_path=~/PX4-Autopilot

for i in $(seq 0 $((NUM_VEHICLES-1))); do
    mkdir -p $build_path/rootfs/$i
    pushd $build_path/rootfs/$i > /dev/null

    PX4_HOME_LAT=47.397742 PX4_HOME_LON=8.545594 PX4_HOME_ALT=488.0 \
    PX4_UXRCE_DDS_NS=px4_$i PX4_UXRCE_DDS_PORT=$((8888+i)) PX4_SIM_MODEL=gazebo-classic_standard_vtol \
    $build_path/bin/px4 -i $i -d $build_path/etc &

    python3 $src_path/Tools/simulation/gazebo-classic/sitl_gazebo-classic/scripts/jinja_gen.py \
        $src_path/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/standard_vtol/standard_vtol.sdf.jinja \
        $src_path/Tools/simulation/gazebo-classic/sitl_gazebo-classic \
        --mavlink_tcp_port $((4560+i)) \
        --mavlink_udp_port $((14560+i)) \
        --mavlink_id $((i+1)) \
        --gst_udp_port $((5600+i)) \
        --video_uri $((5600+i)) \
        --mavlink_cam_udp_port $((14530+i)) \
        --output-file /tmp/standard_vtol_${i}.sdf

    gz model --spawn-file=/tmp/standard_vtol_${i}.sdf --model-name=standard_vtol_${i} \
        -x ${SPAWN_X[$i]} -y ${SPAWN_Y[$i]} -z ${SPAWN_Z[$i]}

    popd > /dev/null
    sleep 2
done

# ROS2 좌표 변환 노드 실행
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run collision_avoidance coordinate_transformer_node --ros-args --params-file ${SCRIPT_DIR}/../config/ros_params.yaml &
sleep 1

echo "${NUM_VEHICLES} VTOLs spawned (GUI: $GUI_MODE). Press Ctrl+C to stop."
trap "pkill -x px4; pkill gzserver; pkill gzclient" SIGINT SIGTERM
wait
