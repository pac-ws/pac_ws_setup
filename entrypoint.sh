#!/usr/bin/env bash
date
echo "ROS_NAMESPACE: $ROS_NAMESPACE"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "ROS_DISTRO: $ROS_DISTRO"
echo "PYTHON_VERSION: $PYTHON_VERSION"

# Avoid cluttering the output with bridge messages but save the logs
ZENOH_LOG_DIR=/workspace/log/zenoh
mkdir -p $ZENOH_LOG_DIR

while true; do
  # Extract the IP (IPv4) address for wlan0 from ifconfig output.
  # Depending on your OS, 'inet ' might appear as 'inet addr:' – adjust as needed.
  IP=$(ifconfig wlan0 2>/dev/null \
        | grep 'inet ' \
        | awk '{print $2}' \
        | sed 's/addr://')

  if [[ -n "$IP" &&  "$IP" =~ ^192\.168\.0\. ]]; then
    echo "Assigned IP: $IP"
    break
  fi
  sleep 5
done

# Once we reach here, hostname -I should return something like '172.17.0.2'
echo "IP address detected: $(hostname -I)"
if [[ "$ROS_NAMESPACE" =~ ^r[0-9]+$ ]]; then
  source /opt/ros/$ROS_DISTRO/setup.bash
  source /opt/ros/extra/install/local_setup.bash
  export PYTHONPATH="/opt/venv/lib/python${PYTHON_VERSION}/site-packages:$PYTHONPATH"
  # bash /workspace/pac_ws_setup/starling_build.bash
  source /workspace/install/local_setup.bash

  # Starling bridge
  ros2 run zenoh_vendor zenoh-bridge-ros2dds -c /workspace/src/zenoh_vendor/configs/zenoh_starling.json5 > $ZENOH_LOG_DIR/zenoh_bridge.log 2>&1 &

  # While check if mission_control_enable is set to true or not defined
  while [ -z "${mission_control_enable+x}" ] || [ "${mission_control_enable}" != "1" ]; do
    mission_control_enable=$(ros2 topic echo /mission_control --once 2>/dev/null | grep -A 1 "data:" | tail -n 1 | awk '{print $2}')
    sleep 2
  done
  echo "Mission Control Enabled, Launching Starling Offboard"
  ros2 launch /workspace/launch/starling_offboard.yaml

else
  # GCS bridge
  ros2 run zenoh_vendor zenoh-bridge-ros2dds -c /workspace/src/zenoh_vendor/configs/zenoh_gcs.json5 > $ZENOH_LOG_DIR/zenoh_bridge.log 2>&1 &
fi

# Keep script alive
while true; do
  sleep 1
done
