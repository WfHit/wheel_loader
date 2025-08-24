#!/bin/bash

# Docker build script for HKUST NXT Dual WL Front board
# Based on the rear board script

set -e

echo "Building hkust_nxt-dual-wl-front_default with Docker"
echo "Using PX4_DOCKER_REPO: px4-dev:latest"
echo "Starting Docker build..."

# Use the same Docker setup as the rear board
docker run --rm -v "$(pwd):/workspace" -w /workspace px4-dev:latest bash -c "
set -e
echo '[docker-entrypoint.sh] Starting'
echo '[docker-entrypoint.sh] (x86_64)'

# Clean previous build
rm -rf build/hkust_nxt-dual-wl-front_default && \
make hkust_nxt-dual-wl-front_default && \
echo 'Build completed successfully!'
"

echo "Build script completed."
