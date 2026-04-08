#!/usr/bin/env bash

set -eo pipefail

WORKSPACE_DIR="${WORKSPACE_DIR:-/home/Paesano/ros2_ws_pi}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
WS_SETUP="${WS_SETUP:-${WORKSPACE_DIR}/install/setup.bash}"
MAP_YAML="${MAP_YAML:-${WORKSPACE_DIR}/maps/my_map_real.yaml}"
MAP_SAVE_PREFIX="${MAP_SAVE_PREFIX:-${WORKSPACE_DIR}/maps/my_map_real}"
HTTP_HOST="${HTTP_HOST:-0.0.0.0}"
HTTP_PORT="${HTTP_PORT:-8000}"
STARTUP_MODE="${STARTUP_MODE:-mapping}"
SIM="${SIM:-false}"

source "${ROS_SETUP}"
source "${WS_SETUP}"

exec ros2 launch paesano_mobile_bridge paesano_mobile_bridge.launch.py \
  sim:="${SIM}" \
  startup_mode:="${STARTUP_MODE}" \
  map_yaml:="${MAP_YAML}" \
  map_save_prefix:="${MAP_SAVE_PREFIX}" \
  http_host:="${HTTP_HOST}" \
  http_port:="${HTTP_PORT}"
