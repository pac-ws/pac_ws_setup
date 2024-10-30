Set environment variables PAC_WS and ROS_NAMESPACE in `~/.bashrc` file.
```bash
export PAC_WS=/data/pac_ws
export ROS_NAMESPACE=r0
```

Close and reopen terminal.

```bash
# Clone pac_ws_setup
mkdir -p ${PAC_WS}
git clone https://github.com/pac-ws/pac_ws_setup.git ${PAC_WS}/pac_ws_setup
```

```bash
# Clone repositories (Use this to also update the repositories)
cd ${PAC_WS}/pac_ws_setup
bash setup_pac_ws.bash -d ${PAC_WS}
```

```bash
# Create container
cd ${PAC_WS}/pac_ws_setup
bash pac_create_container.sh -d ${PAC_WS} --ns ${ROS_NAMESPACE}
```

```bash
# Build ros2 packages
cd ${PAC_WS}
colcon build --packages-select coveragecontrol_sim async_pac_gnn_py cc_rviz px4_homify starling_offboard_cpp starling_demos_cpp
# Alternatively, run the build bash files in the pac_ws_setup directory
bash pac_ws_setup/gcs_build.bash -d ${PAC_WS}
```
