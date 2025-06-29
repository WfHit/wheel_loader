#!/bin/bash

# Custom Docker build script for HKUST wheel loader
set -e

# Use the local px4-dev image
PX4_DOCKER_REPO="px4-dev:latest"

echo "Building hkust_nxt-dual-wl-rear_default with Docker"
echo "Using PX4_DOCKER_REPO: $PX4_DOCKER_REPO"

# Get script directory
SCRIPT_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
SRC_DIR="$SCRIPT_DIR/.."

# Setup ccache directory
CCACHE_DIR=${HOME}/.ccache
mkdir -p "${CCACHE_DIR}"

# Clean any existing build
echo "Cleaning previous build..."
rm -rf "$SRC_DIR/build/hkust_nxt-dual-wl-rear_default"

# Run docker build with root permissions to avoid ownership issues
echo "Starting Docker build..."
docker run --rm \
    -w /workspace \
    --env=CCACHE_DIR=/tmp/ccache \
    --env=HOME=/tmp \
    --volume="${CCACHE_DIR}:/tmp/ccache:rw" \
    --volume="${SRC_DIR}:/workspace:rw" \
    --volume=/tmp:/tmp:rw \
    ${PX4_DOCKER_REPO} \
    bash -c "
        cd /workspace && \
        git config --global --add safe.directory /workspace && \
        git config --global --add safe.directory /workspace/platforms/nuttx/NuttX/nuttx && \
        git config --global --add safe.directory /workspace/src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client && \
        git config --global --add safe.directory /workspace/src/modules/mavlink/mavlink && \
        git config --global --add safe.directory /workspace/src/drivers/gps/devices && \
        make hkust_nxt-dual-wl-rear_default && \
        chown -R $(id -u):$(id -g) /workspace/build
    "

echo "Build completed!"
