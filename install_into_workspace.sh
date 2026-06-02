#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4}"
ROS_WS="${ROS_WS:-$HOME/px4_ws}"

echo "Installing ArUco precision landing subset"
echo "PX4_DIR=${PX4_DIR}"
echo "ROS_WS=${ROS_WS}"
echo "This will overwrite matching ArUco landing files in those workspaces."

if [[ ! -d "${PX4_DIR}" ]]; then
  echo "ERROR: PX4_DIR does not exist: ${PX4_DIR}" >&2
  exit 1
fi

mkdir -p "${ROS_WS}/src"

install -D \
  "${ROOT_DIR}/px4/Tools/simulation/gz/worlds/aruco_landing.sdf" \
  "${PX4_DIR}/Tools/simulation/gz/worlds/aruco_landing.sdf"

mkdir -p "${PX4_DIR}/Tools/simulation/gz/models/arucotag"
cp -a "${ROOT_DIR}/px4/Tools/simulation/gz/models/arucotag/." \
  "${PX4_DIR}/Tools/simulation/gz/models/arucotag/"

mkdir -p "${PX4_DIR}/Tools/simulation/gz/models/x500_gimbal"
cp -a "${ROOT_DIR}/px4/Tools/simulation/gz/models/x500_gimbal/." \
  "${PX4_DIR}/Tools/simulation/gz/models/x500_gimbal/"

mkdir -p "${ROS_WS}/src/px4_offboard"
cp -a "${ROOT_DIR}/ros2_ws/src/px4_offboard/." \
  "${ROS_WS}/src/px4_offboard/"

echo
echo "Installed files. Rebuild ROS 2 package:"
echo "  cd ${ROS_WS}"
echo "  source /opt/ros/humble/setup.bash"
echo "  colcon build --packages-select px4_offboard --symlink-install"
echo "  source ${ROS_WS}/install/setup.bash"
