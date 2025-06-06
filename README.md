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
# Add --dev for cloning using ssh instead of https
```

Add the following to `~/.bashrc` file for easy execution of commands
```bash
source ${PAC_WS}/bin/setup.bash
```

```bash
# Create container (auto)
pac create
# See `pac create -h` for more options
# Uses rmw_cyclonedds_cpp by default
# add --rmw rmw_fastrtps_cpp for fastrtps
```


```bash
# Build ros2 pac packages on gcs
pac build
# To update the packages at anytime on gcs
pac update
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
bash pac_create_container.sh -d ${PAC_WS} --ns ${ROS_NAMESPACE} -n pac-$HOSTNAME --jazzy
# See `pac_create_container.sh -h` for more options
```

```bash
# Build ros2 pac packages
docker exec -it pac-$HOSTNAME bash -ci pac_ws_setup/build.bash
```

## Parameters to understand on GCS

Open `${PAC_WS}/launch/lpac.yaml` and note the following parameters.
```yaml
  - arg:
      name: 'env_dir'
      default: 'tests'
  - arg:
      name: 'idf_file'
      default: '$(var full_path)/1.env'
  - arg:
      name: 'namespaces_of_robots'
      default: '["fake1", "fake2"]'
  - arg:
      name: 'env_scale_factor'
      default: '4.0'
  - arg:
      name: 'vel_scale_factor'
      default: '0.2'
```

### env_dir
This is the directory inside `${PAC_WS}/configs`. The only relevant file is `idf_features` which specifies how the importance points are distributed in the environment. Note that this has to be for $1024 \times 1024$ environment. So scale according to `env_scale_factor` (see below).  

Format of `idf_features` file: recommended sigma: 50, recommended importance: 5
```
CircularBND
x y sigma importance
CircularBND
x y sigma importance
```

### namespaces_of_robots
Set the namespaces of all robots that are part of the experiment. Don't put non-experiment robots. The system will wait for the `/ns/pose` to be published for each robot before doing anything.

### env_scale_factor
The model is for $1024 \times 1024$ environment. So we scale the actual positions of robots.

### vel_scale_factor
The model predicts max velocity to be $5 m/s$. The systems scales the predicted velocity by `vel_scale_factor`. So a predicted velocity of $5 m/s$ will be published as $1 m/s$ for 0.2 scale. Note the final should put a bound check on the velocities. Also, feel free to scale the velocities further.

## Running ros2 launch files

### GCS
Run `pac` without any arguments to see list of supported commands.

### For PX4 simulation
The following commands will create docker containers and automatically launch `starling_offboard.yaml`.
```bash
cd ${PAC_WS}/launch/offboard_sim
bash create_docker_compose.bash <number_of_robots>
docker compose up
# Use docker compose up -d to run in background
# Use docker compose down to stop
```
