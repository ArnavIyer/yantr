# Test VESC Package

This ROS2 package provides a test node to verify your VESC driver functionality.

## What it does

The `test_drive_node` sends velocity commands to your robot to:
1. Drive forward at **1.5 m/s for 3 seconds**
2. Send explicit stop commands (0 m/s) for 1 second
3. Stop sending commands (rely on VESC driver's 0.5s timeout safety)

## Prerequisites

1. **VESC driver must be running**:
   ```bash
   ros2 launch vesc_driver skid_steer_vesc.launch.py
   ```

2. **Robot safety**:
   - Ensure the robot has at least 5 meters of clear space in front
   - Have emergency stop ready
   - Robot should be on the ground (not on a table!)

## Usage

### Method 1: Direct run
```bash
# Source your workspace
source install/setup.bash

# Run the test
ros2 run test_vesc test_drive_node
```

### Method 2: Using the helper script
```bash
# Navigate to the package directory
cd src/test_vesc

# Run the interactive script (includes safety prompts)
./run_test.sh
```

## Topics

**Published:**
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands to the robot
- `/test_drive_status` (std_msgs/String) - Status messages about test progress

**Expected from VESC driver:**
- `/vesc/rpm` (std_msgs/Float64) - Motor RPM feedback
- `/odom` (nav_msgs/Odometry) - Odometry data

## Safety Features

1. **Timeout Protection**: The VESC driver automatically stops the motor if no commands are received for 0.5 seconds
2. **Explicit Stop**: The test sends explicit 0 m/s commands after the drive phase
3. **Limited Duration**: Test only runs for a few seconds total
4. **Status Logging**: Progress is logged to help monitor the test

## Expected Behavior

```
[INFO] Test Drive Node initialized
[INFO] Starting 3-second drive test at 1.5 m/s
[INFO] Driving at 1.5 m/s... 0.5/3.0 seconds
[INFO] Driving at 1.5 m/s... 1.0/3.0 seconds
[INFO] Driving at 1.5 m/s... 1.5/3.0 seconds
[INFO] Driving at 1.5 m/s... 2.0/3.0 seconds
[INFO] Driving at 1.5 m/s... 2.5/3.0 seconds
[INFO] 3 seconds complete! Sending stop command...
[INFO] Test complete! Motor should be stopped.
[INFO] Note: VESC driver has 0.5s timeout safety - motor will stop automatically if no commands sent
```

## Troubleshooting

**Robot doesn't move:**
- Check if the VESC driver nodes are running: `ros2 node list`
- Check if VESCs are connected: Look for connection messages in driver logs
- Verify topics: `ros2 topic echo /cmd_vel` should show the commands being sent

**Robot doesn't stop:**
- This should not happen due to multiple safety layers
- If it occurs, there may be an issue with the VESC driver timeout mechanism

**Build errors:**
- Make sure you have geometry_msgs and std_msgs packages installed
- Rebuild with: `colcon build --packages-select test_vesc`