# ROS2 Joystick Package

This package provides direct joystick control for VESC motor controllers in ROS2, bypassing the need for a separate skid-steer controller to reduce latency.

## Hardware Requirements

- Joystick/gamepad controller compatible with Linux (tested with Logitech F710)
- Access to `/dev/input/js*` device (user must be in `input` group)

## Button/Axis Mapping

- **Axis 0**: Angular velocity control (left stick horizontal)
  - Negative values = left turn (counter-clockwise)
  - Positive values = right turn (clockwise)
- **Axis 3**: Linear velocity control (right stick vertical, INVERTED)
  - Negative values = forward motion
  - Positive values = backward motion
- **Button 7**: Safety enable button (must be held for any movement)
- **Other axes/buttons**: Not used

## Usage

### Quick Start
```bash
# Build the package
cd ~/yantr
colcon build --packages-select joystick
source install/setup.bash

# Run the joystick node
ros2 run joystick joystick_node
```

### Using Launch File
```bash
ros2 launch joystick joystick.launch.py
```

### With Custom Parameters
```bash
ros2 launch joystick joystick.launch.py \
  joystick_device:=/dev/input/js1 \
  max_linear_velocity:=1.5 \
  publish_debug:=true
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `joystick_device` | string | `/dev/input/js0` | Path to joystick device |
| `wheel_track` | double | `0.3175` | Distance between left and right wheels (m) |
| `max_linear_velocity` | double | `1.0` | Maximum forward/backward speed (m/s) |
| `max_angular_velocity` | double | `2.0` | Maximum turning rate (rad/s) |
| `deadband_threshold` | double | `0.05` | Deadband threshold for axis values [0.0-1.0] |
| `publish_debug` | bool | `false` | Whether to publish debug Joy messages |
| `safety_timeout_ms` | int | `500` | Safety timeout in milliseconds |

## Published Topics

- `/left_vesc/target_velocity` (std_msgs/Float64): Left motor velocity command
- `/right_vesc/target_velocity` (std_msgs/Float64): Right motor velocity command  
- `/joystick/status` (sensor_msgs/Joy): Debug joystick state (if `publish_debug` is true)

## Safety Features

- **Safety Button**: Button 7 must be pressed for any movement
- **Timeout Protection**: Motors stop if no joystick input for >500ms
- **Deadband Filtering**: Small joystick movements are ignored to prevent drift
- **Graceful Disconnection**: Sends zero velocities if joystick disconnects

## Troubleshooting

### Joystick Not Detected
```bash
# Check if joystick device exists
ls -la /dev/input/js*

# Test joystick access
jstest /dev/input/js0

# Add user to input group (logout/login required)
sudo usermod -a -G input $USER
```

### No Movement
1. Ensure button 7 (safety enable) is pressed
2. Check joystick axes are working with `jstest`
3. Verify VESC driver nodes are running and accepting commands
4. Check parameter values are reasonable

### Integration with VESC Drivers
This package publishes directly to the VESC driver topics. Ensure your VESC drivers are configured to accept commands on:
- `left_vesc/target_velocity`
- `right_vesc/target_velocity`

The differential drive kinematics match the standard skid-steer controller implementation for seamless integration.