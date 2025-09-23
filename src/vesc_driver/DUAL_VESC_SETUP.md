# Dual VESC Skid-Steer Robot Setup Guide

## Problem Solved

The original VescInterface implementation uses global static variables that prevent multiple instances from running simultaneously. This new architecture uses separate nodes for each VESC motor controller.

## Architecture

**Three-Node Architecture:**
1. **Left VESC Driver** (`single_vesc_driver_node`) - Controls left motor at `/dev/ttyACM0`
2. **Right VESC Driver** (`single_vesc_driver_node`) - Controls right motor at `/dev/ttyACM1`  
3. **Skid-Steer Controller** (`skid_steer_controller_node`) - Coordinates both motors for differential drive

## Hardware Requirements

### VESC Controllers
- **Left VESC**: Connected to `/dev/ttyACM0`
- **Right VESC**: Connected to `/dev/ttyACM1`

### Verify Your Connections
```bash
# Check both VESCs are connected
ls /dev/ttyACM*
# Should show: /dev/ttyACM0  /dev/ttyACM1
```

## Build and Run

### 1. Build the Package
```bash
cd /home/arnav/yantr
colcon build --packages-select vesc_driver test_vesc
source install/setup.bash
```

### 2. Run Complete System (Recommended)
```bash
# Launch all three nodes at once
ros2 launch vesc_driver skid_steer_vesc.launch.py
```

### 3. Or Run Nodes Individually (for debugging)
```bash
# Terminal 1: Left motor  
ros2 run vesc_driver single_vesc_driver_node --ros-args -p serial_port:=/dev/ttyACM0 -p motor_side:=left

# Terminal 2: Right motor
ros2 run vesc_driver single_vesc_driver_node --ros-args -p serial_port:=/dev/ttyACM1 -p motor_side:=right

# Terminal 3: Coordinate both motors
ros2 run vesc_driver skid_steer_controller_node --ros-args -p wheel_track:=0.3175
```

## Critical Parameter

**Wheel Track Distance**: Measure the distance between your left and right wheel centers and update in the launch file:

```python
# In skid_steer_vesc.launch.py
'wheel_track': 0.5  # Replace with your measurement in meters
```

## ROS2 Topic Structure

### Published Topics
- `left_vesc/rpm` (std_msgs/Float64) - Left motor RPM
- `right_vesc/rpm` (std_msgs/Float64) - Right motor RPM  
- `/odom` (nav_msgs/Odometry) - Combined robot odometry with pose

### Subscribed Topics  
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands
  - `linear.x`: Forward/backward speed (m/s)
  - `angular.z`: Turning rate (rad/s)

### Internal Topics (automatically handled)
- `left_vesc/target_velocity` (std_msgs/Float64) - Target velocity for left motor
- `right_vesc/target_velocity` (std_msgs/Float64) - Target velocity for right motor

## Testing

### 1. Test Individual Motors (Debug)
```bash
# Test left motor only
ros2 topic pub left_vesc/target_velocity std_msgs/msg/Float64 "data: 0.5" --once

# Test right motor only  
ros2 topic pub right_vesc/target_velocity std_msgs/msg/Float64 "data: 0.5" --once
```

### 2. Test Complete Skid-Steer System
```bash
# Run the comprehensive test (all motion patterns)
ros2 run test_vesc test_drive_node
```

### 3. Manual Control Examples
```bash
# Forward at 1 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 1.0}, angular: {z: 0.0}" --once

# Turn left in place at 1 rad/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.0}, angular: {z: 1.0}" --once

# Curve right while moving forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 1.0}, angular: {z: -0.5}" --once

# Emergency stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.0}, angular: {z: 0.0}" --once
```

## Motor Direction Troubleshooting

If one motor spins in the wrong direction:

### Option 1: Hardware Fix (Recommended)
Swap any two of the three motor phase wires on the problematic motor.

### Option 2: Software Fix
Modify the ERPM calculation in the appropriate `single_vesc_driver_node.cpp`:
```cpp
// For reversed motor, multiply by -1
double erpm = -1.0 * SPEED_TO_ERPM_GAIN * target_velocity_ + SPEED_TO_ERPM_OFFSET;
```

## Monitoring and Debugging

### Watch All Topics
```bash
# Monitor motor RPMs
ros2 topic echo left_vesc/rpm &
ros2 topic echo right_vesc/rpm &

# Monitor odometry  
ros2 topic echo /odom &

# Monitor target velocities
ros2 topic echo left_vesc/target_velocity &
ros2 topic echo right_vesc/target_velocity &
```

### Node Status
```bash
# Check if all nodes are running
ros2 node list

# Should show:
# /left_vesc
# /right_vesc  
# /skid_steer_controller
```

### Connection Status
Check log output for connection status:
```bash
# Should see messages like:
# [INFO] [left_vesc]: Connected to left VESC at /dev/ttyACM0
# [INFO] [right_vesc]: Connected to right VESC at /dev/ttyACM1
```

## Safety Features

1. **Command Timeout**: 0.5 seconds - individual motors stop if no commands received
2. **Speed Limits**: ±2.0 m/s maximum linear speed per motor
3. **Individual Connection Monitoring**: Each VESC monitored independently
4. **Node Isolation**: Failure of one motor node doesn't crash the others

## Troubleshooting

### "Already connected" Error (Fixed)
The new architecture solves this by using separate processes for each VESC.

### Common Issues
1. **One motor not responding**: Check individual node logs and serial connections
2. **Robot turns when commanding straight**: Check motor directions and wheel track measurement  
3. **No response to cmd_vel**: Ensure skid_steer_controller_node is running
4. **Connection errors**: Verify `/dev/ttyACM0` and `/dev/ttyACM1` exist and are accessible