#!/bin/bash

# Common utility functions and color definitions for PAC workspace scripts
# Source this file in other scripts with: source "$(dirname "${BASH_SOURCE[0]}")/common.bash"

# Exit immediately if a command exits with a non-zero status,
# treat unset variables as an error, and ensure pipelines fail correctly.
set -euo pipefail

# ----------------------------
# Color Definitions
# ----------------------------
RED='\033[0;31m'    # Red
GREEN='\033[0;32m'  # Green
YELLOW='\033[0;33m' # Yellow
BLUE='\033[0;34m'   # Blue
NC='\033[0m'        # No Color

# ----------------------------
# Message Functions
# ----------------------------

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
  echo -e "${YELLOW}Warning: $1${NC}" >&2
}

# Function to display debug messages
debug_message() {
  echo -e "${BLUE}Debug: $1${NC}" >&2
}

# ----------------------------
# Utility Functions
# ----------------------------

# Function to check if a command exists
command_exists() {
  command -v "$1" >/dev/null 2>&1
}

# Function to validate that required commands are available
require_commands() {
  local missing_commands=()
  for cmd in "$@"; do
    if ! command_exists "$cmd"; then
      missing_commands+=("$cmd")
    fi
  done
  
  if [ ${#missing_commands[@]} -ne 0 ]; then
    error_exit "Required commands not found: ${missing_commands[*]}"
  fi
}

# Function to validate SSH name format
validate_ssh_name() {
  local ssh_name="${1}"
  if [[ ! "${ssh_name}" =~ ^[a-zA-Z0-9._-]+$ ]]; then
    error_exit "Invalid SSH name format. Only alphanumeric characters, dots, underscores, and hyphens are allowed."
  fi
}

# Function to validate filename (no path traversal, special chars)
validate_filename() {
  local filename="$1"
  if [[ "$filename" =~ [^a-zA-Z0-9._-] ]] || [[ "$filename" == *".."* ]]; then
    error_exit "Invalid filename: $filename"
  fi
}

# Function to read user input with timeout and default
read_with_timeout() {
  local prompt="$1"
  local timeout="${2:-30}"
  local default="${3:-}"
  local response
  
  if read -t "$timeout" -p "$prompt" response; then
    echo "${response:-$default}"
  else
    echo "$default"
  fi
}

# Function to confirm user action with timeout
confirm_action() {
  local prompt="${1:-Do you want to continue?}"
  local timeout="${2:-30}"
  local response
  
  response=$(read_with_timeout "$prompt [y/N]: " "$timeout" "n")
  [[ "$response" =~ ^([yY][eE][sS]|[yY])$ ]]
}

# Function to backup a file/directory with timestamp
backup_with_timestamp() {
  local path="$1"
  local backup_path="${path}.backup.$(date +%s)"
  
  if [[ -e "$path" ]]; then
    if mv "$path" "$backup_path"; then
      info_message "Backed up '$path' to '$backup_path'"
      echo "$backup_path"
    else
      error_exit "Failed to backup '$path'"
    fi
  fi
}

# Function to create temporary directory with cleanup trap
create_temp_dir() {
  local temp_dir
  temp_dir=$(mktemp -d)
  
  # Set up cleanup trap if not already set
  if [[ -z "${TEMP_CLEANUP_SET:-}" ]]; then
    trap 'cleanup_temp_dirs' EXIT
    TEMP_CLEANUP_SET=1
  fi
  
  # Add to cleanup list
  TEMP_DIRS="${TEMP_DIRS:-} $temp_dir"
  echo "$temp_dir"
}

# Function to cleanup temporary directories
cleanup_temp_dirs() {
  if [[ -n "${TEMP_DIRS:-}" ]]; then
    for dir in $TEMP_DIRS; do
      [[ -d "$dir" ]] && rm -rf "$dir"
    done
  fi
}

# Function to validate environment variable is set
require_env_var() {
  local var_name="$1"
  local var_value="${!var_name:-}"
  
  if [[ -z "$var_value" ]]; then
    error_exit "Environment variable '$var_name' is not set"
  fi
}

# Function to validate directory exists
require_directory() {
  local dir_path="$1"
  local error_msg="${2:-Directory '$dir_path' does not exist}"
  
  if [[ ! -d "$dir_path" ]]; then
    error_exit "$error_msg"
  fi
}

# Function to validate file exists
require_file() {
  local file_path="$1"
  local error_msg="${2:-File '$file_path' does not exist}"
  
  if [[ ! -f "$file_path" ]]; then
    error_exit "$error_msg"
  fi
}