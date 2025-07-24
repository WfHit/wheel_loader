#!/bin/bash

# WK2132 Serial Port Test Script
# Tests all 4 WK2132 UART ports on CUAV X7Plus-WL

echo "WK2132 Serial Port Test"
echo "======================"
echo "Testing 4 WK2132 UART ports: /dev/ttyS10 - /dev/ttyS13"
echo "Make sure to connect loopback (TX->RX) on each port you want to test"
echo ""

# Test configuration
BAUD_RATES=(9600 115200 921600)
PORTS=(/dev/ttyS10 /dev/ttyS11 /dev/ttyS12 /dev/ttyS13)

for port in "${PORTS[@]}"; do
    echo "Testing port: $port"
    echo "------------------------"
    
    # Check if port exists
    if [ ! -e "$port" ]; then
        echo "ERROR: Port $port does not exist!"
        echo "Check WK2132 initialization and I2C connection"
        continue
    fi
    
    # Test different baud rates
    for baud in "${BAUD_RATES[@]}"; do
        echo "Testing $port at $baud baud..."
        
        # Send test bytes and check response
        timeout 5 serial_test -p "$port" -b "$baud" -y 0x55 -z 0xAA
        
        if [ $? -eq 0 ]; then
            echo "✓ $port at $baud baud: OK"
        else
            echo "✗ $port at $baud baud: FAILED"
        fi
    done
    
    echo ""
done

echo "WK2132 Port Test Complete"
echo ""
echo "For interactive testing, use:"
echo "serial_test -e -b 921600 -p /dev/ttyS10 -c -s -R ascii"
echo ""
echo "Press Ctrl+C or ESC to exit interactive test"
