#!/bin/sh
# Native DDS acceptance only: no GPU model, SDK or physical camera acquisition.
set -eu
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
BUILD_ROOT=${WORKBENCH_ROS_BUILD_ROOT:-"$ROOT/.evidence/ros-cpu-build"}
ROS_SETUP=${WORKBENCH_ROS_SETUP:-/opt/ros/humble/setup.sh}
test -f "$ROS_SETUP" || { echo 'ROS 2 Humble setup is required for native DDS acceptance' >&2; exit 2; }
# ROS setup scripts intentionally read unset optional environment variables.
set +u
. "$ROS_SETUP"
set -u
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-126}
export CUDA_VISIBLE_DEVICES=-1
export WORKBENCH_NATIVE_ROS_TEST=1
export PYTHONDONTWRITEBYTECODE=1
mkdir -p "$BUILD_ROOT"
colcon --log-base "$BUILD_ROOT/log" build --base-paths "$ROOT/ros" \
  --build-base "$BUILD_ROOT/build" --install-base "$BUILD_ROOT/install" \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
set +u
. "$BUILD_ROOT/install/setup.sh"
set -u
export PYTHONPATH="$ROOT:$ROOT/tests:$ROOT/calibration:$ROOT/calibration/infra:$ROOT/rule_based_workbench${PYTHONPATH:+:$PYTHONPATH}"
python -m unittest test_ros_data test_ros_inputs test_ros_inference test_ros_recording test_ros_driver test_ros_run
