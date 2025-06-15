#!/bin/bash

# Source common utilities
source "$(dirname "${BASH_SOURCE[0]}")/common.bash"

# Validate required commands
require_commands colcon

# Define packages to build
PACKAGES=(
    "coveragecontrol_sim"
    "async_pac_gnn_py"
    "px4_homify"
    "async_pac_gnn_interfaces"
    "zenoh_vendor"
)

info_message "Starting build for packages: ${PACKAGES[*]}"

# Check if we're in a colcon workspace
if [[ ! -f "package.xml" && ! -d "src" ]]; then
    warning_message "Not in a colcon workspace root. Current directory: $(pwd)"
fi

# Build with proper error handling
if colcon build --packages-select "${PACKAGES[@]}" --cmake-args -DCMAKE_BUILD_TYPE=Release; then
    info_message "Build completed successfully for all packages"
else
    error_exit "Build failed. Check the output above for details."
fi
