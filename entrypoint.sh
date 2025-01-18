#!/bin/bash

echo "ROS_NAMESPACE: $ROS_NAMESPACE"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "PYTHON_VERSION: $PYTHON_VERSION"

if ! source /workspace/install/setup.bash; then
  echo "Failed to source /workspace/install/setup.bash"
  exit 1
fi

export PYTHONPATH="/opt/venv/lib/python${PYTHON_VERSION}/site-packages:$PYTHONPATH"

# While check if mission_control_enable is set to true or not defined
while [ -z "${mission_control_enable+x}" ] || [ "${mission_control_enable}" != "true" ]; do
  mission_control_enable=$(ros2 param get --hide-type /mission_control enable 2>/dev/null)
  sleep 1
done

# If ROS_NAMESPACE is px4_$ROBOT_ID or starts with r#, launch px4_origin.yaml
if [[ "$ROS_NAMESPACE" == "px4_$ROBOT_ID" ]] || [[ "$ROS_NAMESPACE" =~ ^r[0-9]+$ ]]; then
  ros2 launch /workspace/launch/starling_offboard.yaml
fi

# Keep script alive
while true; do
  sleep 1
done
