#!/bin/bash

# Exit immediately if a command exits with a non-zero status,
# Treat unset variables as an error, and ensure pipelines fail correctly.
set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print usage information
print_usage() {
  cat <<EOF
Usage: bash $(basename "$0") [OPTIONS]

Options:
  -d, --directory <workspace directory>  Specify the workspace directory
  -n, --name <container name>            Specify the container name
      --ns <ROS namespace>               Specify the ROS namespace
      --noble                            Use 'noble' image tag (default)
      --humble                           Use 'humble' image tag
  -i, --id                               Specify ROBOT_ID (default: 1) 
  -h, --help                             Display this help message

Examples:
  bash $(basename "$0") -d \${PAC_WS} --ns \${ROS_NAMESPACE} -n gcs --noble
  bash $(basename "$0") -d \${PAC_WS} --ns \${ROS_NAMESPACE} -n pac-${HOSTNAME} --humble
EOF
}

# Function to check if a command exists
command_exists() {
  command -v "$1" >/dev/null 2>&1
}

# Function to handle errors
error_exit() {
  echo -e "${RED}Error: $1${NC}" >&2
  exit 1
}
#
# Function to display informational messages
info_message() {
  echo -e "${BLUE}$1${NC}"
}

# Ensure required commands are available
for cmd in docker getopt; do
  if ! command_exists "$cmd"; then
    error_exit "'$cmd' command is not found. Please install it before running this script."
  fi
done

# Define short and long options
SHORT_OPTS="d:n:h"
LONG_OPTS="directory:,name:,ns:,noble,humble,help"

# Parse options using getopt
PARSED_PARAMS=$(getopt --options "$SHORT_OPTS" --long "$LONG_OPTS" --name "$(basename "$0")" -- "$@") || {
  echo "Failed to parse arguments." >&2
  print_usage
  exit 1
}

# Evaluate the parsed options
eval set -- "$PARSED_PARAMS"

# Initialize variables with default values
IMAGE_TAG=""
WS_DIR=""
CONTAINER_NAME=""
ROS_NAMESPACE=""
ROBOT_ID=1

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
    -i|--id)
      ROBOT_ID="$2"
      shift 2
      ;;
    --noble)
      IMAGE_TAG="noble"
      shift
      ;;
    --humble)
      if [[ "$IMAGE_TAG" == "noble" ]]; then
        error_exit "Cannot use both 'noble' and 'humble' image tags."
      fi
      IMAGE_TAG="humble"
      shift
      ;;
    --ns)
      ROS_NAMESPACE="$2"
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
      echo "Internal error: unexpected option '$1'" >&2
      print_usage
      exit 1
      ;;
  esac
done

if [[ -z "$IMAGE_TAG" ]]; then
  info_message "Using 'noble' image tag by default."
  IMAGE_TAG="noble"
fi

IMAGE_BASE_NAME="agarwalsaurav/pac"

ARCH=$(uname -m)
if [[ "$ARCH" == "aarch64" ]]; then
  IMAGE_TAG="arm64-${IMAGE_TAG}"
fi

IMAGE_NAME="${IMAGE_BASE_NAME}:${IMAGE_TAG}"

# Pull the Docker image
echo "Pulling Docker image: ${IMAGE_NAME}"
if ! docker pull "${IMAGE_NAME}"; then
  error_exit "Failed to pull Docker image: ${IMAGE_NAME}"
fi

# Determine the workspace directory
if [[ -z "$WS_DIR" ]]; then
  if [[ -z "${PAC_WS:-}" ]]; then
    echo "Workspace directory not provided via -d/--directory and PAC_WS is not set."
    print_usage
    exit 1
  else
    echo "Using workspace directory from PAC_WS: ${PAC_WS}"
    WS_DIR="${PAC_WS}"
  fi
fi

# Verify that the workspace directory exists
if [[ ! -d "$WS_DIR" ]]; then
  error_exit "Workspace directory '$WS_DIR' does not exist."
fi

CONTAINER_CC_WS="/workspace"

# Set volume option with proper quoting
VOLUME_OPTION="-v \"${WS_DIR}:${CONTAINER_CC_WS}:rw\""

# Set container name if not provided
if [[ -z "$CONTAINER_NAME" ]]; then
  CONTAINER_NAME="pac-${HOSTNAME}"
fi

# Check if a container with the same name already exists
if docker ps -a --format '{{.Names}}' | grep -Eq "^${CONTAINER_NAME}\$"; then
  echo "A container named '${CONTAINER_NAME}' already exists."
  read -rp "Do you want to remove the existing container and create a new one? [y/N]: " response
  case "$response" in
    [Yy]* )
      docker stop "${CONTAINER_NAME}" && docker rm "${CONTAINER_NAME}" || error_exit "Failed to remove existing container."
      ;;
    * )
      echo "Exiting without creating a new container."
      exit 0
      ;;
  esac
fi

# Initialize Docker run command as an array for safety
DOCKER_RUN_CMD=(
  docker run -it
  --name "${CONTAINER_NAME}"
  --net=host
  --privileged
  --ipc=host
  --restart=always
  --pid=host
  --env "RCUTILS_COLORIZED_OUTPUT=1"
  --env "PAC_WS=${CONTAINER_CC_WS}"
  --workdir "${CONTAINER_CC_WS}"
)

# Add ROS_NAMESPACE if provided
if [[ -n "$ROS_NAMESPACE" ]]; then
  DOCKER_RUN_CMD+=(--env "ROS_NAMESPACE=${ROS_NAMESPACE}")
fi

# Add volume option
DOCKER_RUN_CMD+=(-v "${WS_DIR}:${CONTAINER_CC_WS}:rw")

# Add ROBOT_ID
DOCKER_RUN_CMD+=(--env "ROBOT_ID=${ROBOT_ID}")

# Append the image name and the command to run inside the container
DOCKER_RUN_CMD+=("${IMAGE_NAME}" bash)


echo "Running Docker container with the following command:"
printf "  %q " "${DOCKER_RUN_CMD[@]}"
echo

"${DOCKER_RUN_CMD[@]}"

