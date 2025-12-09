# ROS2 Joystick Package Implementation Plan

## Overview
This plan outlines the complete rewrite of the joystick package to be compatible with ROS2 and directly control the left and right VESC motors without going through the skid-steer controller, reducing latency.

## Current System Architecture Analysis
- The current ROS1 joystick package publishes `sensor_msgs/Joy` messages
- The system has two VESC driver nodes (`left_vesc` and `right_vesc`) that accept `target_velocity` commands on topics:
  - `left_vesc/target_velocity` (std_msgs/Float64)
  - `right_vesc/target_velocity` (std_msgs/Float64)
- A skid-steer controller converts `cmd_vel` (geometry_msgs/Twist) to individual wheel velocities using differential drive kinematics
- The controller logic: 
  - `left_velocity = linear_vel - (angular_vel * wheel_track/2)`
  - `right_velocity = linear_vel + (angular_vel * wheel_track/2)`
  - `wheel_track = 0.3175m` (distance between wheels on same axle)

## Joystick Control Mapping
- **Axis 0**: Controls angular velocity (negative = left, positive = right)
- **Axis 3**: Controls linear velocity (flipped: negative = forward, positive = backward)
- **Button 7**: Safety enable button (must be pressed for any movement)
- **Axes 1 and 2**: Not used (left stick vertical and triggers)

## Implementation Steps

### Step 2: Create ROS2 Package Structure
1. Create `package.xml` with ROS2 format and appropriate dependencies
2. Create `CMakeLists.txt` for ROS2 ament_cmake build system
3. Dependencies needed:
   - `rclcpp` (ROS2 C++ client)
   - `std_msgs` (for Float64 messages)
   - `sensor_msgs` (for Joy messages)
   - System dependencies for joystick access

### Step 3: Implement Low-Level Joystick Interface
1. Create `include/joystick/joystick_interface.hpp`:
   - Class to handle Linux joystick device (`/dev/input/js0`)
   - Methods to read axes and button states
   - Use Linux joystick API (`linux/joystick.h`)
   - Handle joystick events and maintain current state

2. Create `src/joystick_interface.cpp`:
   - Implement joystick device opening and reading
   - Process joystick events using `poll()` and `read()`
   - Maintain arrays of current axis and button states
   - Handle device disconnection/reconnection gracefully

### Step 4: Implement ROS2 Node
1. Create `src/joystick_node.cpp`:
   - ROS2 node class inheriting from `rclcpp::Node`
   - Publishers:
     - `left_vesc/target_velocity` (std_msgs::msg::Float64)
     - `right_vesc/target_velocity` (std_msgs::msg::Float64)
     - Optional: `joystick/status` for debugging (sensor_msgs::msg::Joy)
   - Timer callback to process joystick at ~50Hz
   - Parameters:
     - `joystick_device` (default: "/dev/input/js0")
     - `wheel_track` (default: 0.3175)
     - `max_linear_velocity` (default: 1.0 m/s)
     - `max_angular_velocity` (default: 1.0 rad/s)
     - `deadband_threshold` (default: 0.05) - applied to normalized values [-1.0, +1.0]
     - `publish_debug` (default: false)
     - `safety_timeout_ms` (default: 500) - stop if no joystick input

### Step 5: Implement Control Logic
In the timer callback:
1. Read joystick state using the interface
2. Check if button 7 (safety enable) is pressed:
   - If not pressed: send zero velocities to both motors
   - If pressed: proceed with velocity calculation
3. Process axis values:
   - **Raw Value Conversion**: Linux joystick API returns values typically from -32767 to +32767
   - **Normalization**: Convert to floating-point range [-1.0, +1.0] by dividing by 32767.0
   - Read normalized axis 3 (linear): flip sign (`linear_vel = -normalized_axis3 * max_linear_velocity`)
   - Read normalized axis 0 (angular): `angular_vel = -normalized_axis0 * max_angular_velocity`
   - Apply deadband filtering (recommended ~0.05) to normalized values to prevent drift from stick noise
4. Apply differential drive kinematics (copied from skid-steer controller):
   ```cpp
   double half_track = wheel_track_ / 2.0;
   double left_velocity = linear_vel - (angular_vel * half_track);
   double right_velocity = linear_vel + (angular_vel * half_track);
   ```
5. Apply velocity limiting if needed (ensure commands don't exceed motor capabilities)
6. Publish velocities to both VESC topics as std_msgs::msg::Float64
7. Optionally publish debug Joy message if enabled

### Step 6: Error Handling and Safety
1. Implement joystick disconnection detection
2. Automatic zero velocity on joystick loss
3. Watchdog timer for safety (stop if no joystick input for >500ms)
4. Parameter validation (ensure wheel_track > 0, velocity limits > 0)
5. Logging for connection status and errors

### Step 7: Create Launch File (Optional)
1. Create `launch/joystick.launch.py`:
   - Launch the joystick node with configurable parameters
   - Can be included in the main system launch file

### Step 8: Documentation
1. Update package README with:
   - Hardware requirements (joystick controller)
   - Button/axis mapping
   - Usage instructions
   - Parameter descriptions
2. Add inline code documentation

## File Structure After Implementation
```
src/joystick/
├── package.xml
├── CMakeLists.txt
├── include/joystick/
│   └── joystick_interface.hpp
├── src/
│   ├── joystick_interface.cpp
│   ├── joystick_node.cpp
│   └── main.cpp
├── launch/
│   └── joystick.launch.py
└── README.md
```

## Detailed Kinematic Analysis

### Skid-Steer Robot Configuration
- **Robot Type**: Four-wheeled skid-steer robot with left and right side drive
- **Wheel Layout**: Two wheels per side, each side driven by one VESC motor controller
- **Wheel Track**: 0.3175m (distance between left and right wheel centers)

### Differential Drive Kinematics

The robot uses differential drive kinematics where motion is controlled by varying the velocities of the left and right wheel sets:

#### Forward Kinematics: Joystick Input → Wheel Commands
Given joystick inputs converted to desired robot motion:
- `linear_vel` = desired forward velocity (m/s)
- `angular_vel` = desired turning rate (rad/s, positive = counter-clockwise)
- `wheel_track` = distance between left and right wheels (0.3175m)

The individual wheel velocities are calculated as:
```cpp
double half_track = wheel_track_ / 2.0;  // 0.15875m
double left_velocity = linear_vel - (angular_vel * half_track);
double right_velocity = linear_vel + (angular_vel * half_track);
```

**Physical Interpretation:**
- For pure forward motion (angular_vel = 0): both sides move at same speed
- For pure rotation (linear_vel = 0): left and right sides move at opposite speeds
- For combined motion: the outer wheel moves faster during turns

#### Motion Examples:
1. **Forward motion**: `linear_vel = 1.0, angular_vel = 0.0`
   - Result: `left_velocity = 1.0, right_velocity = 1.0`

2. **Counter-clockwise turn**: `linear_vel = 0.0, angular_vel = 2.0`
   - Result: `left_velocity = -0.3175, right_velocity = 0.3175`

3. **Forward left turn**: `linear_vel = 1.0, angular_vel = 1.0`
   - Result: `left_velocity = 0.84125, right_velocity = 1.15875`

#### Inverse Kinematics: Wheel Feedback → Robot State
For odometry calculation (used in skid-steer controller):
```cpp
double linear_velocity = (left_velocity + right_velocity) / 2.0;
double angular_velocity = (right_velocity - left_velocity) / wheel_track_;
```

### Joystick Axis Mapping Details
- **Axis 0 Range**: [-1.0, +1.0] → Angular velocity control
  - Negative values → Left turn (counter-clockwise, positive angular velocity)  
  - Positive values → Right turn (clockwise, negative angular velocity)
  - **Note**: This creates intuitive steering where left stick = left turn
  - **Implementation**: Use `angular_vel = -axis0_value * max_angular_velocity` for correct direction

- **Axis 3 Range**: [-1.0, +1.0] → Linear velocity control (INVERTED)
  - Negative values → Forward motion (positive linear velocity)
  - Positive values → Backward motion (negative linear velocity)
  - **Note**: Axis 3 requires sign flip due to joystick convention

#### Raw Value Conversion and Axis Processing:

**Raw Joystick Values:** Linux joystick API provides integer values typically ranging from -32767 to +32767 (16-bit signed integers). These must be converted to normalized floating-point values [-1.0, +1.0] for processing.

```cpp
// Raw joystick conversion (handled by joystick interface)
// Raw values: -32767 to +32767 → Normalized: -1.0 to +1.0
float normalized_axis = static_cast<float>(raw_joystick_value) / 32767.0;

// Process normalized joystick inputs
double raw_linear = -joystick_axes[3];   // Flip axis 3
double raw_angular = -joystick_axes[0];  // Flip axis 0 for intuitive steering

// Apply deadband filtering to normalized values
if (abs(raw_linear) < deadband_threshold) raw_linear = 0.0;
if (abs(raw_angular) < deadband_threshold) raw_angular = 0.0;

double linear_vel = raw_linear * max_linear_velocity;
double angular_vel = raw_angular * max_angular_velocity;
```

### Velocity Units and Scaling
- **Raw Input**: Linux joystick API provides integer values typically from -32767 to +32767
- **Normalization**: Values are converted to floating-point range [-1.0, +1.0] for processing
- **Output**: Wheel velocities in m/s (meters per second)
- **VESC Interface**: The VESC drivers expect target_velocity in m/s on std_msgs/Float64 topics
- **Scaling Parameters**:
  - `max_linear_velocity`: Maximum forward/backward speed (default: 1.0 m/s)
  - `max_angular_velocity`: Maximum turning rate (default: 2.0 rad/s)
- **Deadband**: Applied to normalized values (default: 0.05 means values between -0.05 and +0.05 are treated as zero)

### Safety Considerations for Kinematic Implementation
- **Deadband Filtering**: Apply small deadband (~0.05) to prevent stick drift
- **Velocity Limiting**: Consider maximum safe velocities for your robot size/weight
- **Acceleration Limiting**: Optional - add rate limiting to prevent abrupt velocity changes
- **Emergency Stop**: Button 7 must be held; releasing immediately stops all motion

### Joystick Device Access
- Use Linux joystick API (`/dev/input/js*`)
- Handle device permissions (user must be in `input` group)
- Implement non-blocking reads using `poll()` or `select()`

### Coordinate Frame Conventions
- Follow ROS conventions: +X forward, +Z up
- Angular velocity: positive = counter-clockwise (left turn)
- Linear velocity: positive = forward motion

### Real-time Considerations
- Use timer-driven processing rather than event-driven
- Maintain consistent loop rate (~50Hz recommended)
- Minimize memory allocations in the control loop

## Build and Test Instructions

### Step 1: Build the Package
```bash
# From workspace root
cd ~/yantr
colcon build --packages-select joystick
source install/setup.bash
```

### Step 2: Test Joystick Detection
```bash
# Check if joystick device exists
ls -la /dev/input/js*

# Test joystick access (user needs to be in input group)
jstest /dev/input/js0
```

### Step 3: Run the Node
```bash
# Run joystick node
ros2 run joystick joystick_node

# Check published topics
ros2 topic list | grep -E "(left_vesc|right_vesc)"

# Monitor velocity commands
ros2 topic echo /left_vesc/target_velocity
ros2 topic echo /right_vesc/target_velocity
```

### Step 4: Integration Test
```bash
# Launch full system (assuming VESC drivers are running)
ros2 launch vesc_driver skid_steer_vesc.launch.py

# In separate terminal, run joystick
ros2 run joystick joystick_node

# Test: Press button 7 and move joysticks to verify motor commands
```

### Step 5: Verify Build Success
Ensure the package compiles without errors:
- No missing dependencies
- Proper CMakeLists.txt configuration
- Correct include paths
- All source files compile cleanly

The implementation should result in a responsive joystick control system with minimal latency, directly commanding the VESC motor controllers while maintaining safety through the enable button requirement.
