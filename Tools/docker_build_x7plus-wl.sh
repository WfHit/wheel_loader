#!/bin/bash

# Docker build script for CUAV X7Plus-WL board
# WK2132 I2C-to-UART bridge enabled wheel loader configuration

set -e

echo "Building cuav_x7plus-wl_default with Docker"
echo "Using PX4_DOCKER_REPO: px4-dev:latest"
echo "Starting Docker build for Wheel Loader with WK2132 support..."

docker run --rm -v "$(pwd):/workspace" -w /workspace px4-dev:latest bash -c "
set -e
echo '[docker-entrypoint.sh] Starting'
echo '[docker-entrypoint.sh] Building for CUAV X7Plus-WL (Wheel Loader)'

# Clean previous build
echo 'Cleaning previous build...'
rm -rf build/cuav_x7plus-wl_default || true

# Build the target
echo 'Building cuav_x7plus-wl_default...'
make cuav_x7plus-wl_default

echo 'Build completed successfully!'
echo 'Binary location: build/cuav_x7plus-wl_default/cuav_x7plus-wl_default.px4'
ls -la build/cuav_x7plus-wl_default/cuav_x7plus-wl_default.px4 || echo 'Binary not found in expected location'
"

echo "Build script completed."
echo "To flash the firmware, use: make cuav_x7plus-wl_default upload"
