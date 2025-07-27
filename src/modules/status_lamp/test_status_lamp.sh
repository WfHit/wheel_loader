#!/bin/sh
#
# Test script for Status Lamp Controller
#

echo "Testing Status Lamp Controller..."

# Start the module
status_lamp start

# Wait a moment
sleep 1

# Check status
status_lamp status

# Test manual state changes
echo "Testing manual state changes..."
status_lamp set 1  # GREEN
sleep 1
status_lamp set 3  # RED
sleep 1
status_lamp set 0  # OFF

echo "Manual control test complete."
echo "You can also send commands using: uorb publish status_lamp_command '{\"timestamp\":0,\"target_state\":1}'"
