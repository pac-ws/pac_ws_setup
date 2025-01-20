#!/bin/bash

# Exit immediately if a command exits with a non-zero status,
# treat unset variables as an error, and ensure pipelines fail correctly.
set -euo pipefail

# ----------------------------
# Color Definitions
# ----------------------------
RED='\033[0;31m'    # Red
GREEN='\033[0;32m'  # Green
YELLOW='\033[0;33m' # Yellow
NC='\033[0m'        # No Color

# ----------------------------
# Function Definitions
# ----------------------------

# Function to display usage information
usage() {
  echo "Usage: $0 -d <directory>"
  exit 1
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

# ----------------------------
# Initial Checks
# ----------------------------

# Ensure required commands are available
for cmd in git realpath; do
  if ! command_exists "$cmd"; then
    error_exit "'$cmd' command is not found. Please install it before running this script."
  fi
done

# ----------------------------
# Parse Command-line Arguments
# ----------------------------

# if [[ $# -eq 0 ]]; then
#   usage
# fi

# Initialize PAC_WS
# PAC_WS=""

DEV_MODE=0  # Default: development mode off

while getopts ":d:-:" opt; do
  case $opt in
    d)
      PAC_WS="$OPTARG"
      ;;
    -)  # Handle long options
      case "$OPTARG" in
        dev)
          DEV_MODE=1
          ;;
        *)
          echo -e "${RED}Invalid option: --$OPTARG${NC}" >&2
          usage
          ;;
      esac
      ;;
    \?)
      echo -e "${RED}Invalid option: -$OPTARG${NC}" >&2
      usage
      ;;
    :)
      echo -e "${RED}Option -$OPTARG requires an argument.${NC}" >&2
      usage
      ;;
  esac
done

# Check if $ROS_NAMESPACE is set
if [ -z "${ROS_NAMESPACE}" ]; then
  info_message "ROS_NAMESPACE is not set. Can't check if it is gcs."
else
  if [ "${ROS_NAMESPACE}" == "gcs" ]; then
    # Check if user wants to set DEV_MODE to 1
    if [ $DEV_MODE -eq 0 ]; then
      read -p "ROS_NAMESPACE is set to 'gcs'. Do you want to enable development mode? [y/N]: " response
      if [[ "$response" =~ ^([yY][eE][sS]|[yY])+$ ]]; then
        DEV_MODE=1
      fi
    fi
  fi
fi

info_message "PAC_WS: $PAC_WS"

# Debugging or additional actions based on the --dev flag
if [ $DEV_MODE -eq 1 ]; then
  echo "Development mode enabled."
fi

# Check if PAC_WS is set
if [ -z "${PAC_WS}" ]; then
  error_exit "The -d <directory> argument is required."
fi

# Ensure PAC_WS is an absolute path
PAC_WS=$(realpath "${PAC_WS}")

# Check if PAC_WS is a directory
if [ ! -d "${PAC_WS}" ]; then
  error_exit "'${PAC_WS}' is not a directory."
fi

# Ensure PAC_WS/src exists
if [ ! -d "${PAC_WS}/src" ]; then
  mkdir -p "${PAC_WS}/src"
fi

# ----------------------------
# Repositories List
# ----------------------------

# List of repositories and their relative target directories
# Each entry is an array with two elements: REPO_URL and RELATIVE_TARGET_DIR
REPOS_PREFIX="https://github.com/"
if [ $DEV_MODE -eq 1 ]; then
  REPOS_PREFIX="git@github.com:"
fi
REPOS=(
  "${REPOS_PREFIX}pac-ws/pac_ws_setup.git pac_ws_setup"
  "${REPOS_PREFIX}pac-ws/pt.git pt"
  "${REPOS_PREFIX}pac-ws/launch.git launch"
  "${REPOS_PREFIX}pac-ws/configs.git configs"
  "${REPOS_PREFIX}pac-ws/px4_homify.git src/px4_homify"
  "${REPOS_PREFIX}pac-ws/async_pac_gnn_py.git src/async_pac_gnn_py"
  "${REPOS_PREFIX}pac-ws/async_pac_gnn_interfaces.git src/async_pac_gnn_interfaces"
  "${REPOS_PREFIX}pac-ws/coveragecontrol_sim.git src/coveragecontrol_sim"
  "${REPOS_PREFIX}pac-ws/starling_offboard_cpp.git src/starling_offboard_cpp"
  "${REPOS_PREFIX}pac-ws/starling_demos_cpp.git src/starling_demos_cpp"
)

if [ $DEV_MODE -eq 1 ]; then
  REPOS+=(
    "${REPOS_PREFIX}pac-ws/docker.git docker"
    "${REPOS_PREFIX}pac-ws/starling_scripts.git starling_scripts"
    "${REPOS_PREFIX}pac-ws/px4_multi_sim.git px4_multi_sim"
    "${REPOS_PREFIX}pac-ws/cc_rviz.git src/cc_rviz"
    "${REPOS_PREFIX}pac-ws/gcs.git src/gcs"
    "${REPOS_PREFIX}pac-ws/rviz_pac.git src/rviz_pac"
  )
fi

# ----------------------------
# Processing Repositories
# ----------------------------

for ENTRY in "${REPOS[@]}"; do
  # Skip empty lines or lines starting with '#'
  [[ -z "$ENTRY" || "$ENTRY" == \#* ]] && continue

  # Read the REPO_URL and RELATIVE_TARGET_DIR from the entry
  read -r REPO_URL RELATIVE_TARGET_DIR <<< "$ENTRY"

  # Combine PAC_WS and RELATIVE_TARGET_DIR to get the absolute target directory
  TARGET_DIR="${PAC_WS}/${RELATIVE_TARGET_DIR}"

  info_message "Processing repository '$REPO_URL' at '$TARGET_DIR'..."

  if [[ -d "$TARGET_DIR/.git" ]]; then
    info_message "Repository already exists. Checking for updates..."
    cd "$TARGET_DIR" || error_exit "Failed to navigate to '$TARGET_DIR'."

    # Check for local changes
    if [[ -n $(git status --porcelain) ]]; then
      warning_message "There are local changes in '$TARGET_DIR'."
    fi

    git fetch

    LOCAL=$(git rev-parse @)
    REMOTE=$(git rev-parse @{u} || echo "no_upstream")
    BASE=$(git merge-base @ @{u} 2>/dev/null || echo "no_merge_base")

    if [ "$LOCAL" = "$REMOTE" ]; then
      info_message "Repository '$TARGET_DIR' is up-to-date."
    elif [ "$LOCAL" = "$BASE" ]; then
      info_message "Updating repository '$TARGET_DIR'..."
      if git pull --rebase; then
        info_message "Successfully updated repository at '$TARGET_DIR'."
        # If the directory was pac_ws_setup, print warning to re-run setup_pac_ws.bash
        if [[ "$RELATIVE_TARGET_DIR" == "pac_ws_setup" ]]; then
          echo -e "${RED}Please re-run the setup_pac_ws.bash script to ensure the changes are applied.${NC}"
        fi
      else
        error_exit "Failed to update repository at '$TARGET_DIR'."
      fi
    elif [ "$REMOTE" = "$BASE" ]; then
      warning_message "Local commits found in '$TARGET_DIR' that are not in the remote repository."
    else
      warning_message "Git branches have diverged in '$TARGET_DIR'. Manual intervention is required."
    fi
  else
    info_message "Cloning repository..."
    # Create the target directory's parent directories if they don't exist
    mkdir -p "$(dirname "$TARGET_DIR")"

    # Clone the repository
    if git clone "$REPO_URL" "$TARGET_DIR"; then
      info_message "Successfully cloned '$REPO_URL' into '$TARGET_DIR'."
    else
      error_exit "Failed to clone '$REPO_URL' into '$TARGET_DIR'."
    fi
  fi

  echo "----------------------------------------"
done
