## Setting up the laptop (Ground control station GCS)

Set environment variables `PAC_WS` and `ROS_NAMESPACE` in `~/.bashrc` file.
```bash
export PAC_WS=${HOME}/pac_ws
export ROS_NAMESPACE=gcs
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
bash pac_create_container.sh -d ${PAC_WS} --ns ${ROS_NAMESPACE} -n gcs
# Exit the container by pressing ctrl + D
```

```bash
# Enter the container
docker exec -it gcs bash
# Build ros2 packages
cd ${PAC_WS}
bash pac_ws_setup/gcs_build.bash -d ${PAC_WS}
```

## Setting up the robot

Set environment variables `PAC_WS` and `ROS_NAMESPACE` in `~/.bashrc` file.
```bash
export PAC_WS=${HOME}/pac_ws
export ROS_NAMESPACE=sobek # Change as required
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
# Exit the container by pressing ctrl + D
```

```bash
# Enter the container
docker exec -it pac-$HOSTNAME bash
# Build ros2 packages
cd ${PAC_WS}
bash pac_ws_setup/build.bash -d ${PAC_WS}
```

## Parameters to understand on GCS

Open `${PAC_WS}/launch/rviz.yaml` and note the following parameters.
```yaml
  - arg:
      name: 'env_dir'
      default: 'env3_f2_r2'
  - arg:
      name: 'namespaces_of_robots'
      default: '["fake1", "fake2"]'
  - arg:
      name: 'env_scale_factor'
      default: '50.0'
  - arg:
      name: 'vel_scale_factor'
      default: '0.2'
```

### env_dir
This is the directory inside `${PAC_WS}/configs`. The only relevant file is `idf_features` which speicifies how the importance points are distributed in the environment. Note that this has to be for $1024 \times 1024$ environment. So scale according to `env_scale_factor` (see below).

### namespaces_of_robots
Set the namespaces of all robots that are part of the experiment. Don't put non-experiment robots. The system will wait for the `/ns/pose` to be published for each robot before doing anything.

### env_scale_factor
The model is for $1024 \times 1024$ environment. So we scale the actual positions of robots.

### vel_scale_factor
The model predicts max velocity to be $5 m/s$. The systems scales the predicted velocity by `vel_scale_factor`. So a predicted velocity of $5 m/s$ will be published as $1 m/s$ for 0.2 scale. Note the final should put a bound check on the velocities. Also, feel free to scale the velocities further.

## Running ros2 launch files

### GCS
```bash
# Run rviz and coverage control sim
xhost + # might not work on wayland
docker exec -it gcs bash
# You should be inside /workspace
export DISPLAY=:0 # maybe :1, check echo $DISPLAY outside docker
ros2 launch launch/rviz.yaml
```

```bash
# Run status_pac
ros2 run gcs status_pac
# Enter 0 when you want to move robots
# Enter 1 for sending zero velocities
# Enter 2 for stop lpac from publishing to cmd_vel
```

### Robots
```bash
docker exec -it pac-$HOSTNAME bash
ros2 launch launch/lpac_l1.yaml
```
