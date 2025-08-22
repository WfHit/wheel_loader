#!/usr/bin/env python3
"""
UART test client for VLA proxy module
"""

import serial
import struct
import time
import numpy as np
import threading
import argparse

class VLAUartTestClient:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.running = False

    def connect(self):
        """Connect to the VLA proxy via UART"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
            self.running = True
            print(f"Connected to VLA proxy via UART {self.port} at {self.baudrate} baud")
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False

    def _calculate_checksum(self, data):
        """Calculate XOR checksum"""
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum

    def receive_status(self):
        """Receive robot status from PX4"""
        try:
            # Look for frame header (0xAA, 0x01 for status)
            while self.ser.in_waiting > 0:
                header = self.ser.read(1)
                if len(header) == 0:
                    break

                if header[0] == 0xAA:
                    # Read frame type and length
                    frame_info = self.ser.read(2)
                    if len(frame_info) != 2:
                        continue

                    frame_type, data_length = frame_info

                    if frame_type == 0x01:  # Status frame
                        # Read data and checksum
                        remaining_data = self.ser.read(data_length + 1)
                        if len(remaining_data) != data_length + 1:
                            continue

                        data = remaining_data[:-1]
                        received_checksum = remaining_data[-1]

                        # Verify checksum
                        calculated_checksum = self._calculate_checksum(data)
                        if calculated_checksum == received_checksum:
                            # Parse status data (RobotStatus struct)
                            if len(data) >= 66:  # Expected size
                                status = struct.unpack('<Q3f3f4f3fBBf', data[:66])
                                return {
                                    'timestamp': status[0],
                                    'position': list(status[1:4]),
                                    'velocity': list(status[4:7]),
                                    'quaternion': list(status[7:11]),
                                    'angular_vel': list(status[11:14]),
                                    'armed': bool(status[14]),
                                    'flight_mode': status[15],
                                    'battery': status[16]
                                }
                        else:
                            print(f"Checksum mismatch: calculated={calculated_checksum}, received={received_checksum}")
        except Exception as e:
            print(f"Error receiving status: {e}")
        return None

    def send_waypoint(self, position, velocity, acceleration, yaw, yaw_rate, future_time_ms=100):
        """Send trajectory waypoint to PX4"""
        try:
            # Calculate future timestamp
            timestamp_us = int((time.time() + future_time_ms / 1000.0) * 1e6)

            # Pack waypoint data (VLAWaypoint struct):
            # position(12) + velocity(12) + acceleration(12) + yaw(4) + yaw_rate(4) + timestamp_us(8) = 52 bytes
            waypoint_data = struct.pack('<3f3f3fffQ',
                *position, *velocity, *acceleration,
                yaw, yaw_rate, timestamp_us
            )

            # Create frame: [HEADER][TYPE][LENGTH][DATA][CHECKSUM]
            frame_header = 0xAA
            frame_type = 0x02  # Trajectory frame
            data_length = len(waypoint_data)

            # Calculate checksum
            checksum = self._calculate_checksum(waypoint_data)

            # Build complete frame
            frame = struct.pack('<BBB', frame_header, frame_type, data_length) + waypoint_data + struct.pack('<B', checksum)

            self.ser.write(frame)
            return True
        except Exception as e:
            print(f"Error sending waypoint: {e}")
            return False

    def status_monitor_thread(self):
        """Thread to continuously monitor robot status"""
        while self.running:
            status = self.receive_status()
            if status:
                print(f"Status - Pos: [{status['position'][0]:.2f}, {status['position'][1]:.2f}, {status['position'][2]:.2f}], "
                      f"Armed: {status['armed']}, Mode: {status['flight_mode']}")
            time.sleep(0.1)

    def run_circle_trajectory(self, radius=5.0, height=-10.0, duration=20.0, angular_velocity=0.314):
        """Generate and send circular trajectory"""
        print(f"Sending circular trajectory: radius={radius}m, height={height}m, duration={duration}s")

        start_time = time.time()

        while self.running and (time.time() - start_time) < duration:
            t = time.time() - start_time
            angle = angular_velocity * t

            # Circular trajectory
            position = [
                radius * np.cos(angle),
                radius * np.sin(angle),
                height
            ]

            # Velocity for circular motion
            velocity = [
                -radius * angular_velocity * np.sin(angle),
                radius * angular_velocity * np.cos(angle),
                0.0
            ]

            # Acceleration for circular motion
            acceleration = [
                -radius * angular_velocity**2 * np.cos(angle),
                -radius * angular_velocity**2 * np.sin(angle),
                0.0
            ]

            yaw = angle
            yaw_rate = angular_velocity

            if not self.send_waypoint(position, velocity, acceleration, yaw, yaw_rate):
                break

            time.sleep(0.05)  # 20Hz trajectory updates

    def run_figure_eight(self, size=3.0, height=-5.0, duration=30.0):
        """Generate and send figure-8 trajectory"""
        print(f"Sending figure-8 trajectory: size={size}m, height={height}m, duration={duration}s")

        start_time = time.time()

        while self.running and (time.time() - start_time) < duration:
            t = time.time() - start_time
            omega = 2 * np.pi / 10.0  # Complete figure-8 in 10 seconds

            # Lemniscate (figure-8) parametric equations
            cos_t = np.cos(omega * t)
            sin_t = np.sin(omega * t)
            denom = 1 + sin_t**2

            position = [
                size * cos_t / denom,
                size * sin_t * cos_t / denom,
                height
            ]

            # Calculate derivatives for velocity
            velocity = [
                -size * omega * sin_t * (1 - sin_t**2) / denom**2,
                size * omega * (cos_t**2 - sin_t**2) / denom**2,
                0.0
            ]

            acceleration = [0.0, 0.0, 0.0]  # Simplified
            yaw = np.arctan2(velocity[1], velocity[0])
            yaw_rate = 0.0

            if not self.send_waypoint(position, velocity, acceleration, yaw, yaw_rate):
                break

            time.sleep(0.05)  # 20Hz trajectory updates

    def close(self):
        """Close connection"""
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()

def main():
    parser = argparse.ArgumentParser(description='VLA Proxy UART Test Client')
    parser.add_argument('--port', default='/dev/ttyUSB0', help='UART device port')
    parser.add_argument('--baudrate', type=int, default=115200, help='UART baud rate')
    parser.add_argument('--trajectory', choices=['circle', 'figure8', 'monitor'],
                      default='monitor', help='Trajectory type to send')
    parser.add_argument('--duration', type=float, default=30.0, help='Trajectory duration (seconds)')

    args = parser.parse_args()

    client = VLAUartTestClient(args.port, args.baudrate)

    if not client.connect():
        return

    try:
        if args.trajectory == 'monitor':
            # Just monitor status
            print("Monitoring robot status (press Ctrl+C to exit)...")
            client.status_monitor_thread()

        elif args.trajectory == 'circle':
            # Start status monitoring in background
            status_thread = threading.Thread(target=client.status_monitor_thread)
            status_thread.daemon = True
            status_thread.start()

            # Send circular trajectory
            client.run_circle_trajectory(duration=args.duration)

        elif args.trajectory == 'figure8':
            # Start status monitoring in background
            status_thread = threading.Thread(target=client.status_monitor_thread)
            status_thread.daemon = True
            status_thread.start()

            # Send figure-8 trajectory
            client.run_figure_eight(duration=args.duration)

        print("Trajectory complete")

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        client.close()

if __name__ == "__main__":
    main()
