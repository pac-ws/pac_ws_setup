#!/bin/bash

# Function to display usage information
print_usage() {
  echo "Usage: $0 [-d|--directory <workspace directory>] [-n|--name <container name>] [--ns <ROS namespace>] [--noble|--humble]"
}

# Parse and validate input parameters
params="$(getopt -o d:n: -l directory:,ns:,name:,noble,humble --name "$(basename "$0")" -- "$@")"
if [ $? -ne 0 ]; then
  echo "Error parsing parameters."
  print_usage
  exit 1
fi

eval set -- "$params"
unset params

# Default values
IMAGE_TAG="noble"
CONTAINER_NAME=""
WS_DIR=""
ROS_NAMESPACE=""

# Parse options
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
    --noble)
      IMAGE_TAG="noble"
      shift
      ;;
    --humble)
      IMAGE_TAG="humble"
      shift
      ;;
    --ns)
      ROS_NAMESPACE="$2"
      shift 2
      ;;
    --)
      shift
      break
      ;;
    *)
      echo "Unknown option: $1"
      print_usage
      exit 1
      ;;
  esac
done

# Set base image name and handle architecture-specific tags
IMAGE_BASE_NAME="agarwalsaurav/pac"
if [ "$(uname -m)" == "aarch64" ]; then
  IMAGE_TAG="arm64-${IMAGE_TAG}"
fi
IMAGE_NAME="${IMAGE_BASE_NAME}:${IMAGE_TAG}"

# Pull the Docker image
echo "Pulling Docker image: ${IMAGE_NAME}"
docker pull "${IMAGE_NAME}" || {
  echo "Failed to pull Docker image: ${IMAGE_NAME}"
  exit 1
}

# Validate workspace directory
if [ -z "${WS_DIR}" ]; then
  if [ -z "${PAC_WS}" ]; then
    echo "Error: Workspace directory must be specified using -d or --directory, or the PAC_WS environment variable must be set."
    print_usage
    exit 1
  else
    echo "Using workspace directory from PAC_WS: ${PAC_WS}"
    WS_DIR="${PAC_WS}"
  fi
fi

# Set default container name if not provided
if [ -z "${CONTAINER_NAME}" ]; then
  CONTAINER_NAME="pac-${HOSTNAME}"
fi

# Configure volume option for Docker
if [ -n "${WS_DIR}" ]; then
  VOLUME_OPTION="-v ${WS_DIR}:/workspace:rw"
else
  VOLUME_OPTION=""
fi

# Run the Docker container
echo "Starting Docker container: ${CONTAINER_NAME}"
docker run -it \
  --name="${CONTAINER_NAME}" \
  --net=host \
  --privileged \
  --ipc=host \
  --restart=always \
  --pid=host \
  --env=ROS_NAMESPACE="${ROS_NAMESPACE}" \
  --env=RCUTILS_COLORIZED_OUTPUT=1 \
  --env=PAC_WS="/workspace" \
  ${VOLUME_OPTION} \
  --workdir="/workspace" \
  "${IMAGE_NAME}" \
  bash || {
  echo "Failed to start Docker container: ${CONTAINER_NAME}"
  exit 1
}

echo "Docker container '${CONTAINER_NAME}' started successfully."
