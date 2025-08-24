#!/bin/bash

# Custom Docker build script for Pixhawk 6X Pro UWB
set -e

# Use the local px4-dev image
PX4_DOCKER_REPO="px4-dev:latest"

echo "Building holybro_pixhawk-6x-pro-uwb_default with Docker"
echo "Using PX4_DOCKER_REPO: $PX4_DOCKER_REPO"

# Get script directory
SCRIPT_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
SRC_DIR="$SCRIPT_DIR/.."

# Setup ccache directory
CCACHE_DIR=${HOME}/.ccache
mkdir -p "${CCACHE_DIR}"

# Run docker build with all operations inside container
echo "Starting Docker build..."
docker run --rm \
    -w /workspace \
    --env=CCACHE_DIR=/workspace/.ccache \
    --env=HOME=/root \
    --volume="${CCACHE_DIR}:/workspace/.ccache:rw" \
    --volume="${SRC_DIR}:/workspace:rw" \
    ${PX4_DOCKER_REPO} \
    bash -c "
        cd /workspace && \
        echo 'Cleaning previous build...' && \
        rm -rf build/holybro_pixhawk-6x-pro-uwb_default && \
        make holybro_pixhawk-6x-pro-uwb_default && \
        echo 'Fixing file ownership...' && \
        chown -R $(id -u):$(id -g) build
    "

echo "Build completed!"
