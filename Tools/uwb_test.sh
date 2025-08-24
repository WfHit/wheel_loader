#!/bin/bash

# UWB LinkTrack Test Script
# This script helps test the UWB positioning system

echo "=== UWB LinkTrack Test Script ==="
echo ""

# Check if driver is loaded
echo "1. Checking if UWB driver is running..."
if px4-listener sensor_uwb -n 1 > /dev/null 2>&1; then
    echo "   ✓ UWB driver is publishing data"
else
    echo "   ✗ UWB driver not running or no data"
    echo "   Try: nooploop_linktrack start -d /dev/ttyS6"
    exit 1
fi

# Show current parameters
echo ""
echo "2. Current UWB parameters:"
echo "   SENS_UWB_EN: $(param show SENS_UWB_EN 2>/dev/null || echo 'not set')"
echo "   SENS_UWB_BAUD: $(param show SENS_UWB_BAUD 2>/dev/null || echo 'not set')"
echo "   SENS_UWB_TAG_ID: $(param show SENS_UWB_TAG_ID 2>/dev/null || echo 'not set')"
echo "   SENS_UWB_MIN_RSSI: $(param show SENS_UWB_MIN_RSSI 2>/dev/null || echo 'not set')"

# Show EKF2 UWB parameters
echo ""
echo "3. EKF2 UWB parameters:"
echo "   EKF2_UWB_EN: $(param show EKF2_UWB_EN 2>/dev/null || echo 'not set')"
echo "   EKF2_UWB_NOISE: $(param show EKF2_UWB_NOISE 2>/dev/null || echo 'not set')"
echo "   EKF2_UWB_GATE: $(param show EKF2_UWB_GATE 2>/dev/null || echo 'not set')"

# Check anchor positions
echo ""
echo "4. Anchor positions:"
for i in {0..3}; do
    x=$(param show EKF2_UWB_A${i}_X 2>/dev/null || echo '0.0')
    y=$(param show EKF2_UWB_A${i}_Y 2>/dev/null || echo '0.0')
    z=$(param show EKF2_UWB_A${i}_Z 2>/dev/null || echo '0.0')
    echo "   Anchor $i: [$x, $y, $z]"
done

# Monitor UWB data for 10 seconds
echo ""
echo "5. Monitoring UWB sensor data for 10 seconds..."
timeout 10s px4-listener sensor_uwb | head -20

echo ""
echo "6. UWB driver status:"
nooploop_linktrack status 2>/dev/null || echo "   Driver status not available"

echo ""
echo "=== Test Complete ==="
echo ""
echo "Next steps:"
echo "1. Ensure anchor positions are correctly configured"
echo "2. Set EKF2_UWB_EN=1 to enable fusion"
echo "3. Monitor vehicle_local_position topic for positioning updates"
echo "4. Check for multipath/NLOS issues if positioning is unstable"
