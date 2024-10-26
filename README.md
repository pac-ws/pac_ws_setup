Set environment variables PAC_WS and ROS_NAMESPACE in `~/.bashrc` file.
```bash
export PAC_WS=/data/pac_ws
export ROS_NAMESPACE=r0
```

Close and reopen terminal.

```bash
# Clone pac_ws_setup (Use this to also update the repositories)
mkdir -p ${PAC_WS}
git clone https://github.com/pac-ws/pac_ws_setup.git ${PAC_WS}/pac_ws_setup
```

```bash
# Clone repositories
cd ${PAC_WS}/pac_ws_setup
bash setup_pac_ws.bash -d ${PAC_WS}
```

```bash
# Create container
bash pac_create_container.sh -d ${PAC_WS} --ns ${ROS_NAMESPACE}
```

```bash
# Build ros2 packages
colcon build --packages-select coveragecontrol_sim async_pac_gnn_py cc_rviz px4_homify
```

