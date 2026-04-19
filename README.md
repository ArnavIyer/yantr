ros 2 workspace for yantr, a 4 wheel, skid steer robot like the clearpath husky

to run orbbec:
```
ros2 launch astra_camera astra_pro.launch.xml color_fps:=30 depth_fps:=30
```

to run t265 (odometry-only mode, no loop closure):
```
ros2 launch realsense2_camera rs_launch.py enable_pose_jumping:=false
```

to run lidar:
```
ros2 launch ldlidar_stl_ros2 ld19.launch.py
```

to run foxglove bridge:
```
ros2 run foxglove_bridge foxglove_bridge
```

to run slam bridge (necessary for slam toolbox):
```
ros2 run slam slam_bridge_node
```

to run slam toolbox:
```
ros2 run slam_toolbox sync_slam_toolbox_node --ros-args --params-file /home/arnav/yantr/install/slam/share/slam/config/slam_params.yaml
```

To run joystick:
```
```

To run skid steer vesc node:
```
ros2 launch vesc_driver skid_steer...launch
```

to run arducam (USB cam; device path varies — check `/dev/v4l/by-id/` for the Arducam entry):
```
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video2 -p pixel_format:=mjpeg2rgb -p image_width:=640 -p image_height:=480 -p framerate:=30.0 --remap __ns:=/arducam
```

to run piper state publisher (read-only joint states from the Piper arm for rosbag recording; safe to run alongside gello):
```
ros2 run piper_state_publisher piper_state_publisher
```
