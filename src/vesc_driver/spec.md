# VESC Driver Package Specification

## Overview

The VESC Driver is a ROS2 package that provides motor control interfaces for Vedder Electronic Speed Controllers (VESCs) commonly used in electric skateboard and robotics applications. This implementation supports both single motor setups and dual-motor differential drive (skid-steer) robots.

**Key Features:**
- Direct serial communication with VESC hardware
- Support for single and dual motor configurations
- ROS2 native implementation with hardcoded parameters
- Real-time motor feedback and odometry
- Built-in safety features including command timeouts
- Differential drive kinematics for skid-steer robots

## Architecture

### Launch File Configuration

The **`launch/skid_steer_vesc.launch.py`** file implements a distributed three-node architecture:

1. **Left VESC Driver** - `single_vesc_driver_node` instance
   - Parameter: `serial_port: '/dev/ttyACM0'`
   - Parameter: `motor_side: 'left'`
   - Publishes: `left_vesc/rpm`
   - Subscribes: `left_vesc/target_velocity`

2. **Right VESC Driver** - `single_vesc_driver_node` instance  
   - Parameter: `serial_port: '/dev/ttyACM1'`
   - Parameter: `motor_side: 'right'`
   - Publishes: `right_vesc/rpm`
   - Subscribes: `right_vesc/target_velocity`

3. **Skid-Steer Controller** - `skid_steer_controller_node`
   - Parameter: `wheel_track: 0.3175` (distance between wheels)
   - Subscribes: `/cmd_vel` (robot velocity commands)
   - Publishes: `left_vesc/target_velocity`, `right_vesc/target_velocity`
   - Publishes: `/odom` (combined robot odometry)

**Architecture Rationale:** The distributed approach solves the "already connected" problem where VescInterface uses global static variables (`src/vesc_interface.cpp:21-26`) that prevent multiple instances within the same process. Each VESC gets its own process and VescInterface instance.

### Entry Points

The package provides two main executable nodes used by the launch system:

1. **`single_vesc_driver_node`** (`src/single_vesc_driver_node.cpp:122-128`) - Individual VESC motor controller  
2. **`skid_steer_controller_node`** (`src/skid_steer_controller_node.cpp:124-130`) - Coordination layer for dual motor setup

### Core Implementation

#### 1. VESC Communication Layer

**VescInterface** (`src/vesc_interface.cpp:109-128`)
- Manages serial connection to VESC hardware at 115200 baud
- Creates dedicated pthread for asynchronous packet reception (`src/vesc_interface.cpp:23-91`)
- Implements packet framing, CRC validation, and error recovery
- Provides high-level motor control methods: `setSpeed()`, `setCurrent()`, `setBrake()`

**VescPacket System** (`src/vesc_packet.cpp:18-47`)
- Factory pattern implementation for VESC protocol packet creation (`src/vesc_packet_factory.cpp:17-24`)
- Handles variable-length frame encoding (2-byte or 3-byte headers) (`src/vesc_packet.cpp:18-35`)
- Supports multiple packet types: Values, SetRPM, SetCurrent, RequestFWVersion

**Serial Communication** (`src/serial.cc`)
- Low-level POSIX serial port interface
- Blocking and non-blocking I/O operations
- Platform-specific error handling

#### 2. Motor Control Implementation

**Speed Control** (motor calibration constants)
```cpp
// Key conversion constants (robot-specific calibration)
static constexpr double SPEED_TO_ERPM_GAIN = 13160.0;    // m/s to ERPM conversion
static constexpr double ERPM_SPEED_LIMIT = 26320.0;      // ±2 m/s safety limit
```

**Differential Drive Kinematics**
```cpp
void calculateWheelVelocities(double linear_vel, double angular_vel, double& left_vel, double& right_vel) {
  double half_track = wheel_track_ / 2.0;
  left_vel = linear_vel - (angular_vel * half_track);    // Left wheel velocity
  right_vel = linear_vel + (angular_vel * half_track);   // Right wheel velocity
}
```

#### 3. Node Architectures

**Single VESC Driver** (`src/single_vesc_driver_node.cpp:15-58`)
- Simplified single-motor interface with parameter-based configuration
- Publishes RPM feedback on configurable topics
- Receives target velocity commands via ROS2 parameters

**Skid-Steer Controller** (`src/skid_steer_controller_node.cpp:14-45`)
- High-level coordination node for distributed dual-motor setup
- Converts geometry_msgs::Twist to individual wheel velocities
- Publishes to `left_vesc/target_velocity` and `right_vesc/target_velocity`
- Subscribes to individual motor RPM feedback for combined odometry

### Data Flow

#### Standard Data Path (Launch File Architecture)
1. **SkidSteerController** receives `cmd_vel` (`src/skid_steer_controller_node.cpp:30-45`)
2. **Individual Commands** published to `left_vesc/target_velocity`, `right_vesc/target_velocity`
3. **SingleVescDriver** nodes receive target velocities (`src/single_vesc_driver_node.cpp:61-65`)
4. **Direct VESC Control** via independent VescInterface instances
5. **RPM Feedback** published on `left_vesc/rpm`, `right_vesc/rpm`
6. **Centralized Odometry** calculated by SkidSteerController (`src/skid_steer_controller_node.cpp:74-106`)

### Key Patterns

**Factory Pattern**: VescPacketFactory creates protocol packets based on command ID (`src/vesc_packet_factory.cpp:36-47`)

**Observer Pattern**: Boost function callbacks for asynchronous VESC packet reception (`src/vesc_interface.h:27`)

**Singleton Pattern**: Global static variables in VescInterface for pthread and serial management (`src/vesc_interface.cpp:21-26`)

**Template Pattern**: Common packet structure with specialized implementations for different VESC commands (`src/vesc_packet.h:46-69`)

### Configuration

All configuration is hardcoded as compile-time constants:

**Motor Calibration** (`src/single_vesc_driver_node.cpp:49-52` and `src/skid_steer_controller_node.cpp:25-26`)
- `SPEED_TO_ERPM_GAIN = 13160.0` - Conversion factor for 14-pole motor, 25:1 gearbox, 10" wheels
- `ERPM_SPEED_LIMIT = 26320.0` - Maximum safe motor speed (±2 m/s)
- `WHEEL_TRACK = 0.3175` - Distance between left/right wheels (meters) - configured via launch parameter

**Communication Settings** (Launch file parameters)
- `serial_port: '/dev/ttyACM0'` - Left motor VESC device
- `serial_port: '/dev/ttyACM1'` - Right motor VESC device  
- Timer frequency: 20 Hz (50ms intervals)

**Safety Parameters** (`src/single_vesc_driver_node.cpp:53`)
- `COMMAND_TIMEOUT = 0.5` - Maximum time between commands before emergency stop

### Error Handling

**Connection Monitoring**
- Individual VESC connection status checking
- Automatic emergency stop on connection loss
- Retry mechanisms with exponential backoff

**Packet Validation** (`src/vesc_interface.cpp:31-68`)
- CRC checksum verification for all received packets
- Frame synchronization recovery for corrupted data
- Buffer overflow protection with automatic discard

**Safety Timeouts**
- Command timeout triggers automatic motor stop
- Individual motor monitoring prevents single-point failures
- Exception handling for serial communication errors

## ROS2 Interface

### Published Topics

- `/odom` (nav_msgs/Odometry) - Robot pose and velocity estimation
- `vesc/left/rpm` (std_msgs/Float64) - Left motor RPM feedback
- `vesc/right/rpm` (std_msgs/Float64) - Right motor RPM feedback

### Subscribed Topics

- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands (linear.x and angular.z)

### Distributed Setup Topics

- `left_vesc/target_velocity` (std_msgs/Float64) - Left motor target velocity
- `right_vesc/target_velocity` (std_msgs/Float64) - Right motor target velocity
- `left_vesc/rpm` (std_msgs/Float64) - Left motor RPM feedback
- `right_vesc/rpm` (std_msgs/Float64) - Right motor RPM feedback

## Build and Test Instructions

### Prerequisites

**System Dependencies:**
```bash
sudo apt install ros-${ROS_DISTRO}-rclcpp ros-${ROS_DISTRO}-std-msgs ros-${ROS_DISTRO}-geometry-msgs ros-${ROS_DISTRO}-nav-msgs
sudo apt install libboost-system-dev libboost-dev
```

**Hardware Setup:**
- Connect VESC controllers to `/dev/ttyACM0` and `/dev/ttyACM1`
- Verify connections: `ls /dev/ttyACM*`

### Build Process

```bash
cd /home/arnav/yantr
colcon build --packages-select vesc_driver test_vesc
source install/setup.bash
```

### Testing and Verification

#### 1. Individual Motor Testing
```bash
# Test left motor only
ros2 run vesc_driver single_vesc_driver_node --ros-args -p serial_port:=/dev/ttyACM0 -p motor_side:=left

# In another terminal
ros2 topic pub left_vesc/target_velocity std_msgs/msg/Float64 "data: 0.5" --once
```

#### 2. Complete System Testing (Recommended)
```bash
# Launch complete dual-motor system (standard configuration)
ros2 launch vesc_driver skid_steer_vesc.launch.py

# Run comprehensive test suite
ros2 run test_vesc test_drive_node
```

#### 3. Manual Control Examples
```bash
# Forward at 1 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 1.0}, angular: {z: 0.0}" --once

# Turn in place at 1 rad/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.0}, angular: {z: 1.0}" --once

# Emergency stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.0}, angular: {z: 0.0}" --once
```

### Troubleshooting

#### Common Issues

**Permission Denied on Serial Ports:**
```bash
# Grant access to serial devices (may need to run after each boot)
sudo chmod 666 /dev/ttyACM0
sudo chmod 666 /dev/ttyACM1

# Alternative: Add user to dialout group (permanent, requires logout/login)
sudo usermod -a -G dialout $USER
```

**Reason:** Linux restricts direct hardware access for security. VESC controllers appear as USB CDC devices requiring read/write permissions for serial communication.

**Connection Failures:**
- Verify VESC hardware is powered and USB cables are connected
- Check for conflicting processes: `lsof /dev/ttyACM0`
- Monitor connection status in node logs
- Try different USB ports or cables

**Motor Direction Issues:**
- **Hardware fix (recommended):** Swap any two of the three motor phase wires
- **Software fix:** Modify ERPM calculation to multiply by -1 for reversed motor

**"Already connected" Errors:**
- This issue is resolved by the distributed node architecture
- Each VESC gets its own process and VescInterface instance
- Kill any existing VESC driver processes: `pkill -f vesc_driver`

#### Performance Issues

**Slow Response:**
- Verify command rate is set to 20 Hz (`COMMAND_RATE = 20.0`)
- Check for USB hub delays - use direct USB 2.0+ ports
- Monitor system load during operation

**Odometry Drift:**
- Verify `WHEEL_TRACK` parameter matches physical robot measurement
- Check motor calibration constants (`SPEED_TO_ERPM_GAIN`)
- Ensure both motors have consistent direction

#### Safety Warnings

**Critical Safety Features:**
- **0.5 second command timeout** - motors automatically stop if no commands received
- **Speed limits** - hardware protection at ±2 m/s maximum
- **Connection monitoring** - individual motor failure detection
- **Emergency stop** - immediate response to stop commands

**Pre-Test Safety Checklist:**
1. Ensure robot has 5+ meters of clear space
2. Test emergency stop functionality
3. Verify both VESC connections are stable
4. Keep hand near emergency stop during initial testing
5. Start with low speeds (0.1-0.5 m/s) before full speed testing

## File Reference

### Core Source Files

- **`src/single_vesc_driver_node.cpp`** - Individual motor controller (used by launch file)
- **`src/skid_steer_controller_node.cpp`** - High-level coordination node (used by launch file)
- **`src/vesc_interface.{h,cpp}`** - VESC serial communication layer
- **`src/vesc_packet.{h,cpp}`** - VESC protocol packet implementation
- **`src/vesc_packet_factory.{h,cpp}`** - Factory pattern for packet creation
- **`src/serial.{h,cc}`** - Low-level serial port interface
- **`include/vesc_driver/datatypes.h`** - VESC firmware protocol definitions

### Configuration Files

- **`CMakeLists.txt`** - Build configuration and target definitions
- **`package.xml`** - ROS2 package metadata and dependencies
- **`launch/skid_steer_vesc.launch.py`** - Complete system launch configuration

### Documentation

- **`DUAL_VESC_SETUP.md`** - Detailed setup guide for dual-motor configuration
- **`src/ros2_migration.md`** - Historical migration documentation (reference only)