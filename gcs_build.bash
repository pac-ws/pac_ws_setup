#!/bin/bash

# Source common utilities
source "$(dirname "${BASH_SOURCE[0]}")/common.bash"

# Validate required commands
require_commands colcon

# Define packages to build for GCS
PACKAGES=(
    "coveragecontrol_sim"
    "async_pac_gnn_py"
    "async_pac_gnn_interfaces"
    "px4_homify"
    "starling_offboard_cpp"
    "starling_demos_cpp"
    "rviz_pac"
    "gcs"
)

info_message "Starting GCS build for packages: ${PACKAGES[*]}"

# Check if we're in a colcon workspace
if [[ ! -f "package.xml" && ! -d "src" ]]; then
    warning_message "Not in a colcon workspace root. Current directory: $(pwd)"
fi

# Build with proper error handling
if colcon build --packages-select "${PACKAGES[@]}" --cmake-args -DCMAKE_BUILD_TYPE=Release; then
    info_message "GCS build completed successfully for all packages"
else
    error_exit "GCS build failed. Check the output above for details."
fi
