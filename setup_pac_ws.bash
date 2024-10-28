#!/bin/bash
#
# Function to display usage information
usage() {
    echo "Usage: $0 -d <directory>"
    exit 1
}

# Parse command-line arguments
while getopts ":d:" opt; do
    case $opt in
        d)
            PAC_WS="$OPTARG"
            ;;
        \?)
            echo "Invalid option: -$OPTARG" >&2
            usage
            ;;
        :)
            echo "Option -$OPTARG requires an argument." >&2
            usage
            ;;
    esac
done

# Check if PAC_WS is set
if [ -z "${PAC_WS}" ]; then
    echo "Error: The -d <directory> argument is required."
    usage
fi

# Ensure PAC_WS is an absolute path
PAC_WS=$(realpath "${PAC_WS}")

# Check if PAC_WS is a directory
if [ ! -d "${PAC_WS}" ]; then
    echo "Error: '${PAC_WS}' is not a directory."
    exit 1
fi

# Check if $PAC_WS/src exists, if not, create it
if [ ! -d "${PAC_WS}/src" ]; then
    mkdir -p "${PAC_WS}/src"
fi

# List of repositories and their relative target directories
# Format: "REPO_URL RELATIVE_TARGET_DIR"
REPOS=(
    "https://github.com/pac-ws/pac_ws_setup.git pac_ws_setup"
    "https://github.com/pac-ws/pt.git pt"
    "https://github.com/pac-ws/launch.git launch"
    "https://github.com/pac-ws/configs.git configs"
    "https://github.com/wvat/px4_homify.git src/px4_homify"
    "https://github.com/pac-ws/cc_rviz.git src/cc_rviz"
    "https://github.com/pac-ws/async_pac_gnn_py.git src/async_pac_gnn_py"
    "https://github.com/pac-ws/coveragecontrol_sim.git src/coveragecontrol_sim"
    "https://github.com/wvat/starling_offboard_cpp.git src/starling_offboard_cpp"
    "https://github.com/wvat/starling_demos_cpp.git src/starling_demos_cpp"
)

# Loop through each repository in the list
for ENTRY in "${REPOS[@]}"; do
    # Skip empty lines or lines starting with '#'
    [[ -z "$ENTRY" || "$ENTRY" == \#* ]] && continue

    # Read the REPO_URL and RELATIVE_TARGET_DIR from the entry
    REPO_URL=$(echo "$ENTRY" | awk '{print $1}')
    RELATIVE_TARGET_DIR=$(echo "$ENTRY" | awk '{print $2}')

    # Combine PAC_WS and RELATIVE_TARGET_DIR to get the absolute target directory
    TARGET_DIR="${PAC_WS}/$RELATIVE_TARGET_DIR"

    echo "Processing repository '$REPO_URL' at '$TARGET_DIR'..."

    if [[ -d "$TARGET_DIR/.git" ]]; then
        echo "Repository already exists. Attempting to update..."
        cd "$TARGET_DIR" || { echo "Failed to navigate to '$TARGET_DIR'."; exit 1; }

        # Check for local changes
        if [[ -n $(git status --porcelain)  ]]; then
          echo -e "\e[33mWarning: There are local changes in '$TARGET_DIR'.\e[0m"
        fi

        # Attempt to update the repository
        git pull
        if [[ $? -ne 0 ]]; then
            echo "Failed to update repository at '$TARGET_DIR'. Exiting."
            exit 1
        else
            echo "Successfully updated repository at '$TARGET_DIR'."
        fi
    else
        echo "Cloning repository..."
        # Create the target directory's parent directories if they don't exist
        mkdir -p "$(dirname "$TARGET_DIR")"

        # Clone the repository
        git clone "$REPO_URL" "$TARGET_DIR"
        if [[ $? -ne 0 ]]; then
            echo "Failed to clone '$REPO_URL' into '$TARGET_DIR'. Exiting."
            exit 1
        else
            echo "Successfully cloned '$REPO_URL' into '$TARGET_DIR'."
        fi
    fi

    echo "----------------------------------------"
done
