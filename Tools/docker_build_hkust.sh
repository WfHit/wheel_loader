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

# Run docker build as root - simplest working solution
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
        make hkust_nxt-dual-wl-rear_default
    "

# Fix ownership of build artifacts after build completes
echo "Fixing file ownership..."
sudo chown -R $(id -u):$(id -g) "$SRC_DIR/build"

echo "Build completed!"
