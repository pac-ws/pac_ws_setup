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

# Check if PAC_WS is set
if [ -z "${PAC_WS}" ]; then
  error_exit "PAC_WS not set: Either set PAC_WS as environment variable or use -d <directory> argument"
fi

info_message "PAC_WS: $PAC_WS"

# Check if $ROS_NAMESPACE is set
if [ -z "${ROS_NAMESPACE}" ]; then
  info_message "ROS_NAMESPACE is not set. Can't check if it is gcs."
else
  if [[ "$ROS_NAMESPACE" =~ ^gcs.*$ ]]; then
    # Check if user wants to set DEV_MODE to 1
    if [ $DEV_MODE -eq 0 ]; then
      read -t 30 -p "Do you want to enable development mode? [y/N]: " response || response="n"
      if [[ "$response" =~ ^([yY][eE][sS]|[yY])+$ ]]; then
        DEV_MODE=1
      fi
    fi
  fi
fi

# Debugging or additional actions based on the --dev flag
if [ $DEV_MODE -eq 1 ]; then
  echo "Development mode enabled."
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
  "${REPOS_PREFIX}pac-ws/zenoh_vendor.git src/zenoh_vendor"
)

if [ $DEV_MODE -eq 1 ]; then
  REPOS+=(
  "${REPOS_PREFIX}pac-ws/docker.git docker"
  "${REPOS_PREFIX}pac-ws/starling_scripts.git starling_scripts"
  "${REPOS_PREFIX}pac-ws/px4_multi_sim.git px4_multi_sim"
  "${REPOS_PREFIX}pac-ws/gcs.git src/gcs"
  "${REPOS_PREFIX}pac-ws/rviz_pac.git src/rviz_pac"
)
fi

# ----------------------------
# Processing Repositories
# ----------------------------

MAIN_DIRS=""
SRC_DIRS=""

for ENTRY in "${REPOS[@]}"; do
  # Skip empty lines or lines starting with '#'
  [[ -z "$ENTRY" || "$ENTRY" == \#* ]] && continue

  # Read the REPO_URL and RELATIVE_TARGET_DIR from the entry
  # Handle entries that might contain spaces by using array assignment
  ENTRY_ARRAY=($ENTRY)
  if [ ${#ENTRY_ARRAY[@]} -ne 2 ]; then
    error_exit "Invalid repository entry format: '$ENTRY'. Expected: 'REPO_URL TARGET_DIR'"
  fi
  REPO_URL="${ENTRY_ARRAY[0]}"
  RELATIVE_TARGET_DIR="${ENTRY_ARRAY[1]}"

  # Combine PAC_WS and RELATIVE_TARGET_DIR to get the absolute target directory
  TARGET_DIR="${PAC_WS}/${RELATIVE_TARGET_DIR}"
  # if target directory starts with src, add it to the SRC_DIRS list
  # else add it to the MAIN_DIRS list
  if [[ "$RELATIVE_TARGET_DIR" == src* ]]; then
    SRC_DIRS="$SRC_DIRS $TARGET_DIR"
  else
    MAIN_DIRS="$MAIN_DIRS $TARGET_DIR"
  fi

  info_message "Processing repository '$REPO_URL' at '$TARGET_DIR'..."

  if [[ -d "$TARGET_DIR/.git" ]]; then
    info_message "Repository already exists. Checking for updates..."
    pushd "$TARGET_DIR" >/dev/null || error_exit "Failed to navigate to '$TARGET_DIR'."

    # Check for local changes
    if [[ -n $(git status --porcelain) ]]; then
      warning_message "There are local changes in '$TARGET_DIR'."
    fi

    if ! git fetch; then
      error_exit "Failed to fetch updates for repository at '$TARGET_DIR'."
    fi

    LOCAL=$(git rev-parse @ 2>/dev/null) || error_exit "Failed to get local commit hash for '$TARGET_DIR'."
    REMOTE=$(git rev-parse @{u} 2>/dev/null || echo "no_upstream")
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
          read -t 30 -p "Do you want to exit now? [y/N]: " response || response="n"
          if [[ "$response" =~ ^([yY][eE][sS]|[yY])+$ ]]; then
            exit 0
          fi
        fi
      else
        error_exit "Failed to update repository at '$TARGET_DIR'."
      fi
    elif [ "$REMOTE" = "$BASE" ]; then
      warning_message "Local commits found in '$TARGET_DIR' that are not in the remote repository."
    else
      warning_message "Git branches have diverged in '$TARGET_DIR'. Manual intervention is required."
    fi
    
    popd >/dev/null
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

# Do the following if ROS_NAMESPACE starts with gcs.*
if [[ "$ROS_NAMESPACE" =~ ^gcs.*$ ]]; then
  WARNING_FLAG=0
  MAIN_DIRS="${MAIN_DIRS} ${PAC_WS} ${PAC_WS}/src ${PAC_WS}/bin ${PAC_WS}/build ${PAC_WS}/install ${PAC_WS}/log"
  ALL_DIRS="${MAIN_DIRS} ${SRC_DIRS}"

  LOCAL_DIRS=$(find "${PAC_WS}" "${PAC_WS}/src" -maxdepth 1 -type d 2>/dev/null)
  for DIR in $LOCAL_DIRS; do
    if [[ ! " ${ALL_DIRS} " =~ " ${DIR} " ]]; then
      warning_message "Directory '$DIR' is not part of the setup directories list."
      WARNING_FLAG=1
    fi
  done
  if [ $WARNING_FLAG -eq 1 ]; then
      warning_message "Be cautious when deleting the workspace."
  fi
  if [ -d "${PAC_WS}/pac_ws_setup/bin" ]; then
    # Create temporary directory for atomic update to avoid self-modification issues
    TEMP_BIN_DIR=$(mktemp -d)
    cp -r "${PAC_WS}/pac_ws_setup/bin"/* "${TEMP_BIN_DIR}/"
    
    # Backup current bin if it exists
    if [ -d "${PAC_WS}/bin" ]; then
      mv "${PAC_WS}/bin" "${PAC_WS}/bin.backup.$(date +%s)"
    fi
    
    # Atomically move new bin into place
    mv "${TEMP_BIN_DIR}" "${PAC_WS}/bin"
    
    info_message "Scripts updated. Please restart any running pac commands."
  else
    error_exit "Source directory '${PAC_WS}/pac_ws_setup/bin' does not exist. Cannot copy setup scripts."
  fi
  info_message ""
  info_message "Add source \${PAC_WS}/bin/setup.bash to your .bashrc file, if not already added."
  info_message ""
fi
