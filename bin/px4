#!/bin/bash

# Exit immediately if a command exits with a non-zero status,
# treat unset variables as errors, and ensure pipelines fail correctly.
set -euo pipefail

RED='\033[0;31m'    # Red
GREEN='\033[0;32m'  # Green
YELLOW='\033[0;33m' # Yellow
NC='\033[0m'        # No Color

print_usage() {
  cat <<EOF
Usage: bash $(basename "$0") [OPTIONS]

Options:
  -d, --directory <workspace directory>  Specify the workspace directory.
  -n, --name <container name>            Specify the container name.
  -c, --create                           Create a new container.
      --gpu                              Enable GPU support.
  -b, --bash                             Start an interactive bash shell.
  -x, --xrce                             Start MicroXRCEAgent.
  -s, --sim                              Start Gazebo simulator.
      --headless                         Run in headless mode (valid only with -s|--sim).
      --world <world name>               Specify the Gazebo world (valid only with -s|--sim).
  -r, --robots                           Launch multiple robots (see robots_execs.sh)
      --delete                           Delete the container.
  -h, --help                             Show this help message and exit.

Note:
  - Only one of the following options can be specified: -x, -s, -r, -c, -b, --delete.
EOF
}

# Function to check if a command exists
command_exists() {
  command -v "$1" >/dev/null 2>&1
}

# Function to handle errors with colored output
error_exit() {
  echo -e "${RED}Error: $1${NC}" >&2
  exit 1
}

# Function to display informational messages
info_message() {
  echo -e "${GREEN}$1${NC}"
}

# Function to display warnings
warning_message() {
  echo -e "${YELLOW}Warning: $1${NC}"
}

# Ensure required commands are available
for cmd in docker getopt; do
  if ! command_exists "$cmd"; then
    error_exit "'$cmd' command is not found. Please install it before running this script."
  fi
done

export ROS_DOMAIN_ID=10

if [[ $# -eq 0 ]]; then
  print_usage
  exit 1
fi

# Define short and long options
SHORT_OPTS="d:n:w:xcbshr"
LONG_OPTS="directory:,name:,world:,help,create,gpu,xrce,bash,sim,headless,robots,delete"

# Parse options using getopt
PARSED_PARAMS=$(getopt -o "$SHORT_OPTS" -l "$LONG_OPTS" -n "$(basename "$0")" -- "$@") || {
  error_exit "Failed to parse arguments."
}

# Evaluate the parsed options
eval set -- "$PARSED_PARAMS"

# Initialize variables with default values
WS_DIR=""
CONTAINER_NAME="px4_main"
USE_GPU=false
CREATE=false
BASH_MODE=false
XRCE=false
SIM=false
EXEC_MULTIPLE_ROBOTS=false
HEADLESS=""
CONTANER_OPTIONS=""
WORLD="grid"
# WORLD="simple_baylands"
EXCLUSIVE_OPTION_COUNT=0

# Process parsed options
while true; do
  case "$1" in
    -d|--directory)
      WS_DIR="$2"
      shift 2
      ;;
    -n|--name)
      CONTAINER_NAME="$2"
      shift 2
      ;;
    --gpu)
      USE_GPU=true
      shift
      ;;
    -c|--create)
      CREATE=true
      ((++EXCLUSIVE_OPTION_COUNT))
      shift
      ;;
    -b|--bash)
      BASH_MODE=true
      ((++EXCLUSIVE_OPTION_COUNT))
      shift
      ;;
    -x|--xrce)
      XRCE=true
      ((++EXCLUSIVE_OPTION_COUNT))
      shift
      ;;
    --delete)
      DELETE=true
      ((++EXCLUSIVE_OPTION_COUNT))
      shift
      ;;
    -s|--sim)
      SIM=true
      ((++EXCLUSIVE_OPTION_COUNT))
      shift
      ;;
    -r|--robots)
      EXEC_MULTIPLE_ROBOTS=true
      ((++EXCLUSIVE_OPTION_COUNT))
      shift
      ;;
    --headless)
      HEADLESS="--headless"
      shift
      ;;
    -w|--world)
      WORLD="$2"
      shift 2
      ;;
    -h|--help)
      print_usage
      exit 0
      ;;
    --)
      shift
      break
      ;;
    *)
      error_exit "Unknown option: $1"
      ;;
  esac
done

# Enforce that only one exclusive option is specified
if [[ $EXCLUSIVE_OPTION_COUNT -gt 1 ]]; then
  error_exit "Only one of the following options can be specified: -x, -s, -r, -c, -b --delete."
elif [[ $EXCLUSIVE_OPTION_COUNT -eq 0 ]]; then
  error_exit "At least one of the following options must be specified: -x, -s, -r, -c, -b --delete."
fi

IMAGE_NAME="agarwalsaurav/px4-dev-ros2-humble:latest"
# if [[ "$USE_GPU" == true ]]; then
#   IMAGE_NAME="agarwalsaurav/px4-dev-ros2-humble:cuda"
# fi
PX4_DIR="/opt/px4_ws/src/PX4-Autopilot"

# Ensure robots_execs.sh is executable
if [[ ! -x "robots_execs.sh" ]]; then
  if [[ -f "robots_execs.sh" ]]; then
    chmod +x "robots_execs.sh"
  else
    warning_message "'robots_execs.sh' not found. Some functions may not work properly."
  fi
fi

# Check if robots_poses.sh exists
if [[ ! -f "robots_poses.sh" ]]; then
  error_exit "'robots_poses.sh' not found. Please create the file with robot poses."
fi

# Set volume option if WS_DIR is provided
if [[ -n "$WS_DIR" ]]; then
  VOLUME_OPTION="-v ${WS_DIR}:/workspace:rw"
else
  VOLUME_OPTION=""
fi

create_container() {
  # Pull the Docker image
  info_message "Pulling Docker image: ${IMAGE_NAME}"
  if ! docker pull "${IMAGE_NAME}"; then
    error_exit "Failed to pull Docker image: ${IMAGE_NAME}"
  fi

  if docker ps -q -f name="$CONTAINER_NAME" | grep -q .; then
    warning_message "Container '${CONTAINER_NAME}' is already running."
    return 0
  fi
  # Add gpu support
  if [[ "$USE_GPU" == true ]]; then
    info_message "Enabling GPU support..."
    CONTANER_OPTIONS="--gpus all"
    CONTANER_OPTIONS="${CONTANER_OPTIONS} --env NVIDIA_VISIBLE_DEVICES=all"
    CONTANER_OPTIONS="${CONTANER_OPTIONS} --env NVIDIA_DRIVER_CAPABILITIES=all"
    # Get group id of video group
    VIDEO_GID=$(getent group video | cut -d: -f3)
    RENDER_GID=$(getent group render | cut -d: -f3)
    CONTANER_OPTIONS="${CONTANER_OPTIONS} --env VIDEO_GID=${VIDEO_GID}"
    CONTANER_OPTIONS="${CONTANER_OPTIONS} --env RENDER_GID=${RENDER_GID}"
  fi
  info_message "Creating Docker container '${CONTAINER_NAME}'..."
  docker run -d -it --rm --init --privileged \
    --env LOCAL_USER_ID="$(id -u)" \
    --env ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
    --env PX4_GZ_STANDALONE=1 \
    --env PX4_SYS_AUTOSTART=4001 \
    --env PX4_GZ_MODEL=x500 \
    --env DISPLAY="${DISPLAY:-:0}" \
    --net=host \
    --ipc=host \
    --pid=host \
    -v "$(pwd)":/px4_scripts:rw \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    ${CONTANER_OPTIONS} \
    ${VOLUME_OPTION} \
    --name="${CONTAINER_NAME}" "${IMAGE_NAME}" bash
}

exec_xrce() {
  if docker ps -q -f name="$CONTAINER_NAME" | grep -q .; then
    info_message "Starting MicroXRCEAgent in container '${CONTAINER_NAME}'..."
    docker exec -it "${CONTAINER_NAME}" MicroXRCEAgent udp4 -p 8888
  else 
    warning_message "Container '${CONTAINER_NAME}' is not running. Creating container..."
    create_container
    exec_xrce
  fi
}

exec_bash() {
  if docker ps -q -f name="$CONTAINER_NAME" | grep -q .; then
    info_message "Starting bash shell in container '${CONTAINER_NAME}'..."
    docker exec -it "${CONTAINER_NAME}" bash
  else 
    warning_message "Container '${CONTAINER_NAME}' is not running. Creating container..."
    create_container
    exec_bash
  fi
}

exec_gazebo_sim() {
  xhost +
  if docker ps -q -f name="$CONTAINER_NAME" | grep -q .; then
    info_message "Starting Gazebo simulator in container '${CONTAINER_NAME}'..."
    docker exec -it "${CONTAINER_NAME}" python "${PX4_DIR}/Tools/simulation/gz/simulation-gazebo" ${HEADLESS} --world "${WORLD}"
  else 
    error_exit "Container '${CONTAINER_NAME}' is not running. Please create the container first."
  fi
}

exec_multiple_robots() {
  if docker ps -q -f name="$CONTAINER_NAME" | grep -q .; then
    info_message "Launching multiple robots in container '${CONTAINER_NAME}'..."
    docker exec -it "${CONTAINER_NAME}" bash /px4_scripts/robots_execs.sh /px4_scripts/robots_poses.sh
  else 
    error_exit "Container '${CONTAINER_NAME}' is not running. Please create the container first."
  fi
}

if [[ "$CREATE" == true ]]; then
  create_container
  exit 0
fi

if [[ "$BASH_MODE" == true ]]; then
  exec_bash
  exit 0
fi

if [[ "$XRCE" == true ]]; then
  exec_xrce
  exit 0
fi

if [[ "$SIM" == true ]]; then
  exec_gazebo_sim
  exit 0
fi

if [[ "$EXEC_MULTIPLE_ROBOTS" == true ]]; then
  exec_multiple_robots
  exit 0
fi

if [[ "$DELETE" == true ]]; then
  if docker ps -q -f name="$CONTAINER_NAME" | grep -q .; then
    docker stop "${CONTAINER_NAME}"
  fi
fi
