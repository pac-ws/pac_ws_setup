#!/bin/bash

date
echo "ROS_NAMESPACE: $ROS_NAMESPACE"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "ROS_DISTRO: $ROS_DISTRO"
echo "PYTHON_VERSION: $PYTHON_VERSION"

while true; do
  # Extract the IP (IPv4) address for wlan0 from ifconfig output.
  # Depending on your OS, 'inet ' might appear as 'inet addr:' – adjust as needed.
  IP=$(ifconfig wlan0 2>/dev/null \
        | grep 'inet ' \
        | awk '{print $2}' \
        | sed 's/addr://')

  sleep 5
done

# Once we reach here, hostname -I should return something like '172.17.0.2'
echo "IP address detected: $(hostname -I)"
if [[ "$ROS_NAMESPACE" =~ ^r[0-9]+$ ]]; then
  source /opt/ros/$ROS_DISTRO/setup.bash
  source /opt/ros/extra/install/local_setup.bash
  export PYTHONPATH="/opt/venv/lib/python${PYTHON_VERSION}/site-packages:$PYTHONPATH"
  bash /workspace/pac_ws_setup/starling_build.bash
  source /workspace/install/local_setup.bash
   
  # While check if mission_control_enable is set to true or not defined
  while [ -z "${mission_control_enable+x}" ] || [ "${mission_control_enable}" != "True" ]; do
    mission_control_enable=$(ros2 param get --hide-type /mission_control enable 2>/dev/null)
    sleep 2
  done
  echo "Mission Control Enabled, Launching Starling Offboard"
  ros2 launch /workspace/launch/starling_offboard.yaml
fi

# Keep script alive
while true; do
  sleep 1
done
