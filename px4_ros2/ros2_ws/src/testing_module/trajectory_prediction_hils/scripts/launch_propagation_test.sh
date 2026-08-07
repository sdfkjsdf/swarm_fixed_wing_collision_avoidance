#!/usr/bin/env bash

set -o pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
export HILS_NODE_EXECUTABLE=trajectory_prediction_sils_test_node
exec bash "${SCRIPT_DIR}/launch_1vtol_replay.sh" "$@"
