#!/bin/bash
# 5대의 InternalState 를 한 화면에서 모니터링
# 사용법:
#   ./monitor_swarm.sh         # 실시간 (1Hz, watch)
#   ./monitor_swarm.sh once    # 한 번만 출력

NUM_AGENTS=${NUM_AGENTS:-5}
INTERVAL=${INTERVAL:-1}

# ROS2 source
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

print_states() {
    echo "=== Swarm Status ($(date +%T)) ==="
    for i in $(seq 0 $((NUM_AGENTS-1))); do
        line=$(timeout 0.5 ros2 topic echo --once /swarm_status/vehicle_$i 2>/dev/null \
               | grep "data:" | sed 's/data: //;s/"//g')
        if [ -z "$line" ]; then
            printf "vehicle_%d: (no data)\n" "$i"
        else
            echo "$line"
        fi
    done
}

if [ "$1" == "once" ]; then
    print_states
else
    # 실시간 모드 (watch 처럼)
    while true; do
        clear
        print_states
        sleep $INTERVAL
    done
fi
