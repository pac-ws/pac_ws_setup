#!/bin/bash
echo "ROS_NAMESPACE: $ROS_NAMESPACE"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "PYTHON_VERSION: $PYTHON_VERSION"
source /workspace/install/setup.bash
export PYTHONPATH=/opt/venv/lib/python${PYTHON_VERSION}/site-packages:$PYTHONPATH
# if ROS_NAMESPACE is gcs, then launch gcs_origin.yaml
#if [ "$ROS_NAMESPACE" == "gcs" ]; then
#  ros2 launch /workspace/launch/gcs_origin.yaml
#fi

# if ROS_NAMESPACE is px4_$ROBOT_ID or starts with r#, where # is a number, then launch px4_origin.yaml
# if [[ "$ROS_NAMESPACE" == "px4_$ROBOT_ID" ]] || [[ "$ROS_NAMESPACE" =~ ^r[0-9]+$ ]]; then
#   ros2 launch /workspace/launch/starling_offboard.yaml
# fi
while [ 1 ]; do
  sleep 1
done
