#!/bin/bash
set -e

# source base ROS environment
source /opt/ros/${ROS_DISTRO}/setup.bash

# dynamic workspace path (set from WORKSPACE_NAME env var passed by docker-compose)
WORKSPACE_PATH="/home/ros/${WORKSPACE_NAME}"

# source workspace if already compiled
if [ "$ROS_VERSION" = "1" ] && [ -f "${WORKSPACE_PATH}/devel/setup.bash" ]; then
    echo "Sourcing ROS 1 workspace: ${WORKSPACE_PATH}/devel/setup.bash"
    source "${WORKSPACE_PATH}/devel/setup.bash"
elif [ "$ROS_VERSION" = "2" ] && [ -f "${WORKSPACE_PATH}/install/setup.bash" ]; then
    echo "Sourcing ROS 2 workspace: ${WORKSPACE_PATH}/install/setup.bash"
    source "${WORKSPACE_PATH}/install/setup.bash"
else
    echo "Workspace not built yet. Run: colcon build --cmake-args -DCMAKE_CXX_FLAGS=\"-w\" --executor sequential"
fi

# --- install workspace dependencies ---
echo "Updating ROS dependencies..."

SUDO_CMD=""
if [ "$(id -u)" -ne 0 ]; then
    SUDO_CMD="sudo"
fi

$SUDO_CMD apt-get update -y && \
$SUDO_CMD apt-get upgrade -y || echo "Apt update/upgrade failed, continuing..."

if command -v rosdep > /dev/null; then
    rosdep update || echo "rosdep update failed, continuing..."
    if [ -d "${WORKSPACE_PATH}/src" ]; then
        rosdep install --from-paths "${WORKSPACE_PATH}/src" --ignore-src -r -y \
            --skip-keys "moveit_resources" || \
        echo "rosdep install encountered issues, continuing..."
    fi
else
    echo "rosdep not found, skipping rosdep steps."
fi

# execute the user command (e.g., bash)
exec "$@"
