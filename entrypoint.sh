#!/usr/bin/env bash

ROSInit() {
  source /opt/ros/$ROS_DISTRO/setup.bash
  source /opt/ros/extra/install/local_setup.bash
  export PYTHONPATH="/opt/venv/lib/python${PYTHON_VERSION}/site-packages:$PYTHONPATH"
  # bash /workspace/pac_ws_setup/starling_build.bash
  source /workspace/install/local_setup.bash
}

GetFieldMissionControl() {
  echo $(ros2 topic echo /mission_control --once --field $1 2>/dev/null | sed '/^---$/d')
}

LaunchController() {
  local controller_type=""
  while true; do
    if [ "$(GetFieldMissionControl pac_offboard_only)" = "True" ]; then
      controller_type="OFFBOARD_ONLY"
      break
    fi
    if [ "$(GetFieldMissionControl pac_lpac_l1)" = "True" ]; then
      controller_type="LPAC_ONE"
      break
    fi
    if [ "$(GetFieldMissionControl pac_lpac_l2)" = "True" ]; then
      controller_type="LPAC_TWO"
      break
    fi
    sleep 2
  done
  
  if [ "$controller_type" = "OFFBOARD_ONLY" ]; then
    echo "Launching Starling Offboard"
    ros2 launch /workspace/launch/starling/starling_offboard.yaml
  elif [ "$controller_type" = "LPAC_ONE" ]; then
    echo "Launching LPAC L1"
    ros2 launch /workspace/launch/starling/lpac_l1.yaml
  elif [ "$controller_type" = "LPAC_TWO" ]; then
    echo "Launching LPAC L2"
    ros2 launch /workspace/launch/starling/lpac_l2.yaml
  fi
}

LaunchVision() {
    # Todo combine in a launch file
    ros2 run starling_cams tracking_cam_sync.py &
    ros2 run apriltag_ros apriltag_node --ros-args -r image_rect:=/camera_down/image_raw -r camera_info:=/camera_down/camera_info --params-file `ros2 pkg prefix apriltag_ros`/share/apriltag_ros/cfg/tags_36h11.yaml &
}

date
echo "ROS_NAMESPACE: $ROS_NAMESPACE"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "ROS_DISTRO: $ROS_DISTRO"
echo "PYTHON_VERSION: $PYTHON_VERSION"
echo "USE_ZENOH: ${USE_ZENOH:-1}"

# Avoid cluttering the output with bridge messages but save the logs
ZENOH_LOG_DIR=/workspace/log/zenoh
mkdir -p "$ZENOH_LOG_DIR"
################################################################################
## STARLING ##
################################################################################
if [[ "$ROS_NAMESPACE" =~ ^r[0-9]+$ ]]; then
  # Only need to starling
  while true; do
    # Extract the IP (IPv4) address for wlan0 or eth0 from ifconfig output.
    # Depending on your OS, 'inet ' might appear as 'inet addr:' – adjust as needed.
    IP=$( (ifconfig wlan0 2>/dev/null || ifconfig eth0 2>/dev/null) \
      | grep 'inet ' \
      | awk '{print $2}' \
      | sed 's/addr://')

    if [[ -n "$IP" &&  ( "$IP" =~ ^192\.168\.0\.  || "$IP" =~ ^10\.223\.1\. ) ]]; then
      echo "Assigned IP: $IP"
      break
    fi
    sleep 5
  done
  # Once we reach here, hostname -I should return something like '172.17.0.2'
  echo "IP address detected: $(hostname -I)"
  ROSInit

  # Starling bridge
  if [[ "${USE_ZENOH:-1}" == "1" ]]; then
    echo "Starting Starling bridge"
    ros2 run zenoh_vendor zenoh-bridge-ros2dds -c /workspace/src/zenoh_vendor/configs/zenoh_starling.json5 > "$ZENOH_LOG_DIR"/zenoh_bridge.log 2>&1 &
  else
    echo "Zenoh bridge disabled (USE_ZENOH=0)"
  fi

  LaunchVision

  # While check if mission_control_hw_enable is set to true or not defined
  while [ -z "${mission_control_hw_enable+x}" ] || [ "${mission_control_hw_enable}" != "True" ]; do
    mission_control_hw_enable=$(GetFieldMissionControl hw_enable)
    sleep 2
  done
  echo "Mission Control Enabled, Waiting for controller type"
  LaunchController

################################################################################
## PX4 Simulator ##
################################################################################
elif [[ "$ROS_NAMESPACE" =~ ^px4_[0-9]+$ ]]; then
  echo "This is a PX4 simulator container"
  ROSInit

  # Starling bridge
  if [[ "${USE_ZENOH:-1}" == "1" ]]; then
    echo "Starting Starling bridge"
    ros2 run zenoh_vendor zenoh-bridge-ros2dds -c /workspace/src/zenoh_vendor/configs/zenoh_starling.json5 > "$ZENOH_LOG_DIR"/zenoh_bridge.log 2>&1 &
  else
    echo "Zenoh bridge disabled (USE_ZENOH=0)"
  fi

  echo "Waiting for controller type"

  LaunchController

################################################################################
## Everything else ##
################################################################################
else
  ROSInit
  #
  # GCS bridge
  if [[ "${USE_ZENOH:-1}" == "1" ]]; then
    echo "Starting GCS bridge"
    ros2 run zenoh_vendor zenoh-bridge-ros2dds -c /workspace/src/zenoh_vendor/configs/zenoh_gcs.json5 > "$ZENOH_LOG_DIR"/zenoh_bridge.log 2>&1 &
  else
    echo "Zenoh bridge disabled (USE_ZENOH=0)"
  fi
fi

# Keep script alive
while true; do
  sleep 10
done
