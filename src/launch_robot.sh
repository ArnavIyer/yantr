#!/bin/bash
# Launch script for yantr robot
# Launches: LiDAR, joystick, T265, VESC motors, Astra camera

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== Yantr Robot Launch Script ===${NC}"

# Store PIDs for cleanup
PIDS=()

cleanup() {
    echo -e "\n${YELLOW}Shutting down all nodes...${NC}"
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null
        fi
    done
    wait
    echo -e "${GREEN}All nodes stopped.${NC}"
    exit 0
}

trap cleanup SIGINT SIGTERM

# --- Device Permission Setup ---
echo -e "${YELLOW}Checking devices...${NC}"

MISSING_DEVICE=0

# LiDAR on /dev/ttyUSB0
if [ -e /dev/ttyUSB0 ]; then
    sudo chmod 777 /dev/ttyUSB0
    echo -e "${GREEN}  ✓ LiDAR (/dev/ttyUSB0)${NC}"
else
    echo -e "${RED}  ✗ LiDAR device /dev/ttyUSB0 not found${NC}"
    MISSING_DEVICE=1
fi

# Joystick on /dev/input/js0
if [ -e /dev/input/js0 ]; then
    echo -e "${GREEN}  ✓ Joystick (/dev/input/js0)${NC}"
else
    echo -e "${RED}  ✗ Joystick device /dev/input/js0 not found - is controller connected?${NC}"
    MISSING_DEVICE=1
fi

# VESC motors on /dev/ttyACM0 and /dev/ttyACM1
if [ -e /dev/ttyACM0 ]; then
    sudo chmod 666 /dev/ttyACM0
    echo -e "${GREEN}  ✓ Left VESC (/dev/ttyACM0)${NC}"
else
    echo -e "${RED}  ✗ Left VESC device /dev/ttyACM0 not found${NC}"
    MISSING_DEVICE=1
fi

if [ -e /dev/ttyACM1 ]; then
    sudo chmod 666 /dev/ttyACM1
    echo -e "${GREEN}  ✓ Right VESC (/dev/ttyACM1)${NC}"
else
    echo -e "${RED}  ✗ Right VESC device /dev/ttyACM1 not found${NC}"
    MISSING_DEVICE=1
fi

# Arducam on /dev/video0 (optional)
if [ -e /dev/video0 ]; then
    echo -e "${GREEN}  ✓ Arducam (/dev/video0)${NC}"
else
    echo -e "${YELLOW}  - Arducam (/dev/video0) not found (optional)${NC}"
fi

if [ $MISSING_DEVICE -eq 1 ]; then
    echo -e "${RED}Exiting: One or more required devices not found.${NC}"
    exit 1
fi

# --- Source ROS2 and Workspace ---
echo -e "${YELLOW}Sourcing ROS2 and workspace...${NC}"
source /opt/ros/humble/setup.bash
source /home/arnav/yantr/install/setup.bash
echo -e "${GREEN}  ✓ Environment sourced${NC}"

# --- Launch Nodes ---
echo -e "${YELLOW}Launching nodes...${NC}"

# LiDAR (LD19)
echo -e "  Starting LiDAR..."
ros2 launch ldlidar_stl_ros2 ld19.launch.py &
PIDS+=($!)
sleep 1

# Joystick
echo -e "  Starting Joystick..."
ros2 launch joystick joystick.launch.py &
PIDS+=($!)
sleep 1

# RealSense T265
echo -e "  Starting T265..."
ros2 launch realsense2_camera rs_launch.py &
PIDS+=($!)
sleep 1

# VESC Skid Steer
echo -e "  Starting VESC motors..."
ros2 launch vesc_driver skid_steer_vesc.launch.py &
PIDS+=($!)
sleep 1

# Astra Camera (Orbbec depth camera)
echo -e "  Starting Astra camera..."
ros2 launch astra_camera astra.launch.xml enable_color:=true enable_depth:=true &
PIDS+=($!)
sleep 1

# Arducam USB Camera (V4L2)
if [ -e /dev/video0 ]; then
    echo -e "  Starting Arducam USB camera..."
    ros2 run usb_cam usb_cam_node_exe --ros-args \
        -p video_device:=/dev/video0 \
        -p framerate:=30.0 \
        -p image_width:=640 \
        -p image_height:=480 \
        -p pixel_format:=mjpeg \
        --remap __ns:=/arducam &
    PIDS+=($!)
else
    echo -e "${YELLOW}  ! Arducam (/dev/video0) not found - skipping${NC}"
fi

echo -e "${GREEN}=== All nodes launched ===${NC}"
echo -e "Press Ctrl+C to stop all nodes"

# Wait for all background processes
wait
