```bash
PAC_WS=${HOME}/pac_ws
mkdir -p ${PAC_WS}
git clone https://github.com/pac-ws/pac_ws_setup.git ${PAC_WS}/pac_ws_setup
cd ${PAC_WS}/pac_ws_setup
```

```bash
# Clone repositories
bash setup_pac_ws.bash -d ${PAC_WS}
```

```bash
# Create container
# Requires ROS_NAMESPACE to be set
bash pac_create_container.sh -d ${PAC_WS} --ns ${ROS_NAMESPACE}
```

```bash
# Build ros2 packages
colcon build --packages-select coveragecontrol_sim async_pac_gnn_py cc_rviz px4_homify
```

