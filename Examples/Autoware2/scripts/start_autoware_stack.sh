#!/usr/bin/env bash
# Step 4: Launch the classical Autoware 2.1.0 stack inside the Docker container,
# initialize localization via GNSS + NDT scan matching, and stay running cleanly.
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /opt/autoware/setup.bash

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file:///carla_example/configs/cyclonedds/cyclonedds_container.xml}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"

LAUNCH_FILE="${AUTOWARE_CARLA_LAUNCH:-/carla_example/configs/autoware_2_1/carla_autoware_2_1.launch.xml}"
MAP_PATH="${AUTOWARE_MAP_PATH:-/carla_example/maps/Town10HD}"
LOG_FILE="${AUTOWARE_LOG_FILE:-/tmp/autoware_2_1.log}"
RVIZ_FLAG="${AUTOWARE_RVIZ:-false}"

AUTOWARE_PID=""
cleanup() {
  if [[ -n "${AUTOWARE_PID}" ]] && kill -0 "${AUTOWARE_PID}" 2>/dev/null; then
    echo ""
    echo "[stop] Shutting down Autoware 2.1.0 stack (pid=${AUTOWARE_PID})..."
    kill -INT "${AUTOWARE_PID}" 2>/dev/null || true
    wait "${AUTOWARE_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "======================================================================"
echo " Starting Classical Autoware 2.1.0 (CARLA 1.0 Native ROS 2)"
echo "   Map path      : ${MAP_PATH}"
echo "   Launch file   : ${LAUNCH_FILE}"
echo "   CycloneDDS    : ${CYCLONEDDS_URI}"
echo "   Log file      : ${LOG_FILE}"
echo "======================================================================"

ros2 launch "${LAUNCH_FILE}" \
  map_path:="${MAP_PATH}" \
  rviz:="${RVIZ_FLAG}" \
  > "${LOG_FILE}" 2>&1 &
AUTOWARE_PID=$!

echo "[wait] Waiting for Autoware ADAPI, map loader, and NDT/EKF localization..."
python3 /carla_example/scripts/autoware_helper.py init-localization --timeout 120 --retries 15 &
INIT_PID=$!
while kill -0 "${INIT_PID}" 2>/dev/null; do
  if ! kill -0 "${AUTOWARE_PID}" 2>/dev/null; then
    echo "[FAIL] ros2 launch exited early. Last 40 lines of ${LOG_FILE}:" >&2
    tail -n 40 "${LOG_FILE}" >&2 || true
    kill "${INIT_PID}" 2>/dev/null || true
    exit 1
  fi
  sleep 1
done
if ! wait "${INIT_PID}"; then
  echo "[FAIL] Autoware localization did not initialize. Last 40 lines of ${LOG_FILE}:" >&2
  tail -n 40 "${LOG_FILE}" >&2 || true
  exit 1
fi

echo ""
echo "======================================================================"
echo " [READY] Classical Autoware 2.1.0 is localized and closed-loop ready."
echo " Leave this container process running and execute Step 5 on macOS."
echo "======================================================================"

wait "${AUTOWARE_PID}"
