#!/bin/bash

# Test VESC Drive Script
# This script runs the test drive node that sends 1.5 m/s commands for 3 seconds

echo "=== VESC Test Drive ==="
echo "This will command the robot to drive at 1.5 m/s for 3 seconds"
echo "Make sure:"
echo "1. vesc_driver_node is running"
echo "2. Robot has clearance to move forward"
echo "3. Emergency stop is available"
echo ""
echo "Press Enter to continue, or Ctrl+C to cancel..."
read

# Source the ROS2 workspace
source install/setup.bash

echo "Starting test drive node..."
ros2 run test_vesc test_drive_node