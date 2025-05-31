#!/bin/bash

# Exit immediately if a command exits with a non-zero status,
# Treat unset variables as an error, and ensure pipelines fail correctly.
set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_usage() {
  cat <<EOF

Options:
  -h, help                            Display this help message
  -c, create                          Create a new GCS container
  -b, build                           Run the build script

  mission                             Launch GCS mission_control
  rviz                                Launch RViz without LPAC
  lpac                                Run LPAC
  bag <bag_file>                      Saves pose and cmd_vel to a bag file

  ------------------------------------------------------------------------
  <sys_name>: Use one of the following names 
            gcs.*       :   GCS container
            px4_[0-9]+  :   Simulation container
            r[0-9]+#    :   Robot container (pac-m0054)
            .r[0-9]+#   :   Robot bash (* = not applicable)

  If the system name is gcs, you will be prompted to enter the namespace.
  Otherwise, the namespace will be the same as the system name.
  ------------------------------------------------------------------------

  cmd <sys_name> <command>            Run a command

  list <sys_name>                     List topics
  logs <sys_name>                     Get docker logs
  delete <sys_name>                   Delete the container
  restart <sys_name>                  Restart the container
  restart-px4 <sys_name>              Restart the px4 service
  bash <sys_name>                     Start bash
  gps <sys_name>                      Run voxl-inspect-gps
  batt <sys_name>                     Run voxl-inspect-battery
  journal <sys_name>                  Run journalctl -u voxl-px4 -f
  update <sys_name>                   Run setup_pac_ws.bash (gcs | r*)

  pose <sys_name>                     Echo /<ns>/pose topic
  vel <sys_name>                      Echo /<ns>/cmd_vel topic
  vlp <sys_name>                      Echo /<ns>/fmu/out/vehicle_local_position topic
  vgps <sys_name>                     Echo /<ns>/fmu/out/vehicle_gps_position topic

  === Deprecated ===
  origin                               Launch GCS mission_origin
  pac                                  Run PAC status script
  rqt                                  Launch rqt

EOF

}

error_exit() {
  echo -e "${RED}Error: $1${NC}" >&2
  exit 1
}

info_message() {
  echo -e "${BLUE}$1${NC}"
}

check_args() {
  if [[ -z "${1:-}" ]]; then
    error_exit "Missing argument."
  fi
}

sys_pattern='^(gcs.*|px4_[0-9]+|r[0-9]+|\.r[0-9]+)$'
check_sys_name() {
  if [[ -z "${1:-}" ]]; then
    echo -n "gcs"
    return
  fi
  if [[ ! "$1" =~ $sys_pattern ]]; then
    error_exit "Not a valid system name."
  fi
  echo -n "$1"
}

if [[ $# -eq 0 ]]; then
  print_usage
  error_exit "No arguments provided."
fi


if [[ -z "${PAC_WS:-}" ]]; then
  error_exit "PAC_WS environment variable is not set. Please set it before running this script."
fi

if [[ ! -d "$PAC_WS" ]]; then
  error_exit "PAC_WS points to '$PAC_WS', but it is not a valid directory."
fi

GPU_FLAG=""

GCS_CONTAINER_NAME="gcs"
ROBOT_CONTAINER_NAME="pac-m0054"

get_ns_gcs() {
  if [[ "$1" =~ ^gcs.*$ ]]; then
    read -p "Enter namespace: " namespace
    echo -n "$namespace"
  else
    echo -n "$1"
  fi
}

docker_cmd() {
  if [[ -z "${1:-}" || -z "${2:-}" ]]; then
    error_exit "Missing container name or command. Usage: docker_cmd <container_name> <command>"
  fi
  CONTAINER_NAME="${1}"
  shift
  info_message "Running command '$*' in container '${CONTAINER_NAME}'..."
  docker exec -it "${CONTAINER_NAME}" bash -ci "$*"
}

gcs_cmd() {
  docker_cmd "gcs" "$@"
}

robot_cmd() {
  if [[ -z "${1:-}" || -z "${2:-}" ]]; then
    error_exit "Missing robot SSH name or command. Usage: robot_cmd <robot_ssh_name> <command>"
  fi
  SSH_NAME="${1}"
  # Remove leading dot
  SSH_NAME="${SSH_NAME#.}"
  shift
  info_message "Running command '$*' on '$SSH_NAME'..."
  ssh -t "${SSH_NAME}" "$*"
}

robot_docker_cmd() {
  if [[ -z "${1:-}" || -z "${2:-}" ]]; then
    error_exit "Missing robot SSH name or command. Usage: robot_docker_cmd <robot_ssh_name> <command>"
  fi
  echo "Args: $@"
  SSH_NAME="${1}"
  SSH_NAME="${SSH_NAME#.}"
  shift
  info_message "Running command '$*' in container '${ROBOT_CONTAINER_NAME}' on '${SSH_NAME}'..."
  ssh -t "${SSH_NAME}" "docker exec -it ${ROBOT_CONTAINER_NAME} bash -ci '$*'"
}

process_cmd() {
  case "$1" in
    gcs*|px4_*)
      docker_cmd $1 "${@:2}"
      ;;
    .r*)
      robot_cmd $1 "${@:2}"
      ;;
    r*)
      robot_docker_cmd $1 "${@:2}"
      ;;
    *)
      error_exit "Invalid system name."
      ;;
  esac
}

case "$1" in
  -h|help)
    print_usage
    exit 0
    ;;
  -c|create)
    info_message "Creating PAC container..."
    if [ "$(command -v nvidia-smi)" ]; then
      GPU_FLAG="--gpu"
    fi
    bash ${PAC_WS}/pac_ws_setup/pac_create_container.sh -d "${PAC_WS}" --ns ${GCS_CONTAINER_NAME} -n ${GCS_CONTAINER_NAME} --jazzy ${GPU_FLAG}
    ;;
  mission)
    gcs_cmd "pip install pyqt5"
    xhost +
    gcs_cmd "rm -f /root/.config/ros.org/rqt_gui.ini"
    gcs_cmd "export DISPLAY='$DISPLAY'; ros2 launch /workspace/launch/mission_control.py"
    ;;
  rviz)
    xhost +
    gcs_cmd "export DISPLAY='$DISPLAY'; ros2 launch launch/rviz.yaml"
    ;;
  rviz-only)
    xhost +
    gcs_cmd "export DISPLAY='$DISPLAY'; ros2 launch launch/rviz_only.yaml"
    ;;
  lpac)
    info_message "Running LPAC status script..."
    gcs_cmd "export DISPLAY='$DISPLAY'; ros2 launch launch/lpac.yaml"
    ;;
  bag)
    BAG_FILE=${2:-}
    gcs_cmd "ros2 bag record -o "/workspace/bags/${BAG_FILE}" -e '.*\/(pose|cmd_vel|sim)'"
    ;;
  list)
    SYS=$(check_sys_name "${2:-}")
    process_cmd "${SYS}" "ros2 topic list"
    ;;
  logs)
    SYS=$(check_sys_name "${2:-}")
    case "${SYS}" in
      gcs*|px4_*)
        docker logs -f "${SYS}"
        ;;
      r*|.r*)
        robot_cmd "${SYS}" "docker logs -f ${ROBOT_CONTAINER_NAME}"
        ;;
    esac
    ;;
  delete)
    SYS=$(check_sys_name "${2:-}")
    case "${SYS}" in
      gcs*|px4_*)
        docker stop "${SYS}"
        docker rm "${SYS}"
        ;;
      r*|.r*)
        robot_cmd "${SYS}" "docker stop ${ROBOT_CONTAINER_NAME}"
        robot_cmd "${SYS}" "docker rm ${ROBOT_CONTAINER_NAME}"
        ;;
    esac
    ;;
  -b|build)
    SYS=$(check_sys_name "${2:-}")
    case "${SYS}" in
      gcs)
        gcs_cmd "/workspace/pac_ws_setup/gcs_build.bash"
        ;;
      r*|.r*)
        robot_docker_cmd "${SYS}" '/workspace/pac_ws_setup/starling_build.bash'
        ;;
    esac
    ;;
  update)
    SYS=$(check_sys_name "${2:-}")
    case "${SYS}" in
      gcs)
        bash ${PAC_WS}/pac_ws_setup/setup_pac_ws.bash
        ;;
      r*|.r*)
        robot_docker_cmd "${SYS}" '/workspace/pac_ws_setup/setup_pac_ws.bash -d /workspace/'
        read -p "Do you want to restart the container? (y/n): " response
        if [[ "$response" =~ ^([yY][eE][sS]|[yY])+$ ]]; then
          robot_cmd "${SYS}" "docker restart ${ROBOT_CONTAINER_NAME}"
        fi
        ;;
    esac
    ;;
  restart)
    SYS=$(check_sys_name "${2:-}")
    case "${SYS}" in
      gcs*|px4_*)
        docker restart "${SYS}"
        ;;
      r*|.r*)
        robot_cmd "${SYS}" "docker restart ${ROBOT_CONTAINER_NAME}"
        ;;
    esac
    ;;
  restart-px4)
    SYS=$(check_sys_name "${2:-}")
    case "${SYS}" in
      r*|.r*)
        robot_cmd "${SYS}" "systemctl restart voxl-px4"
        ;;
    esac
    ;;
  bash)
    SYS=$(check_sys_name "${2:-}")
    process_cmd "${SYS}" "bash"
    ;;
  gps)
    SYS=$(check_sys_name "${2:-}")
    case "${SYS}" in
      gcs*)
        error_exit "Not applicable for GCS."
        ;;
      px4_*)
        docker_cmd "${SYS}" "voxl-inspect-gps"
        ;;
      r*|.r*)
        robot_cmd "${SYS}" "voxl-inspect-gps"
        ;;
      *)
        process_cmd "${SYS}" "ros2 run volx_inspect_gps volx_inspect_gps"
        ;;
    esac
    ;;
  batt)
    SYS=$(check_sys_name "${2:-}")
    case "${SYS}" in
      gcs*)
        error_exit "Not applicable for GCS."
        ;;
      px4_*)
        docker_cmd "${SYS}" "voxl-inspect-battery"
        ;;
      r*|.r*)
        robot_cmd "${SYS}" "voxl-inspect-battery"
        ;;
      *)
        process_cmd "${SYS}" "ros2 run voxl_inspect_battery voxl_inspect_battery"
        ;;
    esac
    ;;
  journal)
    SYS=$(check_sys_name "${2:-}")
    case "${SYS}" in
      gcs*)
        error_exit "Not applicable for GCS."
        ;;
      px4_*)
        docker_cmd "${SYS}" "journalctl -u voxl-px4 -f"
        ;;
      r*|.r*)
        robot_cmd "${SYS}" "journalctl -u voxl-px4 -f"
        ;;
      *)
        process_cmd "${SYS}" "journalctl -u voxl-px4 -f"
        ;;
    esac
    ;;
  cmd)
    SYS=$(check_sys_name "${2:-}")
    process_cmd "${SYS}" "${@:3}"
    ;;
  pose)
    SYS=$(check_sys_name "${2:-}")
    ns=$(get_ns_gcs "${SYS}")
    process_cmd "${SYS}" "ros2 topic echo /${ns}/pose"
    ;;
  vel)
    SYS=$(check_sys_name "${2:-}")
    ns=$(get_ns_gcs "${SYS}")
    process_cmd "${SYS}" "ros2 topic echo /${ns}/cmd_vel"
    ;;
  vlp)
    SYS=$(check_sys_name "${2:-}")
    ns=$(get_ns_gcs "${SYS}")
    process_cmd "${SYS}" "ros2 topic echo /${ns}/fmu/out/vehicle_local_position"
    ;;
  vgps)
    SYS=$(check_sys_name "${2:-}")
    ns=$(get_ns_gcs "${SYS}")
    process_cmd "${SYS}" "ros2 topic echo /${ns}/fmu/out/vehicle_gps_position"
    ;;
  origin)
    info_message "Launching GCS origin..."
    gcs_cmd "ros2 launch /workspace/launch/extra/gcs_origin.yaml"
    ;;
  pac)
    info_message "Running PAC status script..."
    gcs_cmd "ros2 run gcs status_pac"
    ;;
  rqt)
    info_message "Launching rqt..."
    xhost +
    gcs_cmd "export DISPLAY='$DISPLAY'; rqt"
    ;;
  *)
    info_message "Internal error: unexpected option '$1'" >&2
    print_usage
    ;;
esac
