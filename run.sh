#!/bin/bash

# Author: Do Won Park
# Istituto Italiano di Tecnologia (IIT)
# Email: do.park@iit.it

# =================================================================== #
#          workspace config and build, run and stop docker            #
# =================================================================== #

# XAUTH setup for GUI forwarding
export XAUTH=/tmp/.docker.xauth
echo "Setting up XAUTH at $XAUTH"
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

if [ ! -f "$XAUTH" ]; then
    echo "ERROR: XAUTH file not created."
    exit 1
fi
echo "XAUTH configured."
echo "------------------------------------------------"

# --- ROS config ---
ROS_DISTRO="jazzy"
ROS_VERSION="2"

# --- Docker parameters (automatic from current directory) ---
export WORKSPACE_NAME=$(basename "$PWD")
IMAGE_NAME="${WORKSPACE_NAME}_ws"
CONTAINER_NAME="${WORKSPACE_NAME}_container"

# =================================================================== #

# ensure required directories exist
mkdir -p ./docker
mkdir -p ./src

# --- export variables for docker-compose ---

# UID/GID (match host user to avoid permission issues on mounted volumes)
export MYUID=$(id -u)
export MYGID=$(id -g)

# ROS variables
export ROS_DISTRO
export ROS_VERSION
export IMAGE_NAME

# GUI variables
export DISPLAY=$DISPLAY
export XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

# Audio variables
export XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR}
export PULSE_SERVER=unix:${XDG_RUNTIME_DIR}/pulse/native

# Bash history persistence
export BASH_HISTORY_PATH=~/.bash_history
touch $BASH_HISTORY_PATH

# Hardware group IDs (ensure host user is in 'audio' and 'video' groups)
export AUDIO_GID=$(getent group audio | cut -d: -f3)
export VIDEO_GID=$(getent group video | cut -d: -f3)

# VSCode integration
export VSCODE_PATH=~/.vscode
export VSCODE_SERVER_PATH=~/.vscode-server

# function to show usage
usage() {
    echo "Usage: $0 [build|rebuild|run|down]"
    echo "  build:   build docker image (uses cache, fast)."
    echo "  rebuild: rebuild docker image from scratch (no cache, slow)."
    echo "  run:     run interactive container."
    echo "  down:    stop and remove container and networks."
}

case "$1" in
    build)
        echo "Building docker image '$IMAGE_NAME' with ROS${ROS_VERSION} ${ROS_DISTRO}..."
        docker compose -f ./docker-compose.yaml build
        ;;
    rebuild)
        echo "Rebuilding docker image '$IMAGE_NAME' from scratch (no cache)..."
        docker compose -f ./docker-compose.yaml build --no-cache
        ;;
    run)
        echo "Running container '$CONTAINER_NAME'..."
        xhost +local:docker
        docker compose -f ./docker-compose.yaml run --rm --name ${CONTAINER_NAME} ros_dev
        ;;
    down)
        echo "Stopping and removing container..."
        docker compose -f ./docker-compose.yaml down
        ;;
    *)
        usage
        exit 1
        ;;
esac

exit 0
