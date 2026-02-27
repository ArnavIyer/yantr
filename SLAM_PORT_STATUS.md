# SLAM C++ Port Status

## What Was Implemented

- Added a new ROS 2 C++ package at `src/slam`:
  - `src/slam/src/slam_bridge_node.cpp`
  - `src/slam/src/utils.cc`
  - `src/slam/include/slam/utils.hpp`
  - `src/slam/config/slam_params.yaml`
  - `src/slam/CMakeLists.txt`
  - `src/slam/package.xml`

- Ported behavior from `experimental/slam.py` into `slam_bridge_node`:
  - Subscribes to `/pose/sample` (`nav_msgs/msg/Odometry`)
  - Publishes dynamic TF `odom -> base_link`
  - Subscribes to `/scan` and publishes filtered scan to `/scan_filtered`
  - Publishes static TF `base_link -> base_laser` with:
    - translation: `(0.30, -0.28, 0.18)`
    - yaw: `-pi/2`
  - First odom velocity check is warn-and-continue by default
    (`strict_first_velocity` parameter defaults to `false`)

- Patched LiDAR launch to avoid duplicate TF publishers:
  - Updated `src/ldlidar_stl_ros2/launch/ld19.launch.py`
  - Removed the `static_transform_publisher` for `base_link -> base_laser`
  - Added comment explaining TF ownership moved to `slam_bridge_node`

- Updated robot launch script:
  - `src/launch_robot.sh` now starts:
    - `ros2 run slam slam_bridge_node`
    - `ros2 run slam_toolbox sync_slam_toolbox_node --ros-args --params-file /home/arnav/yantr/src/slam/config/slam_params.yaml`


## What Still Needs To Be Done On Robot

1. Ensure ROS 2 Humble environment is installed and sourceable.
2. Ensure `slam_toolbox` is installed and available in ROS environment.
3. Install package dependencies:
   ```bash
   source /opt/ros/humble/setup.bash
   cd /home/arnav/yantr
   rosdep install --from-paths src --ignore-src -r -y
   ```
4. Build workspace:
   ```bash
   cd /home/arnav/yantr
   colcon build --packages-select slam
   source install/setup.bash
   ```
5. Run robot launch:
   ```bash
   ./src/launch_robot.sh
   ```


## Verification Checklist

- Node/process checks:
  - `ros2 node list` includes `slam_bridge`
  - `slam_toolbox` node is running

- Topic checks:
  - `ros2 topic list` includes `/scan_filtered`
  - `ros2 topic hz /scan_filtered` reports live data

- TF checks:
  - `base_link -> base_laser` exists and is single-sourced
  - `odom -> base_link` updates live from T265 odometry
  - no duplicate TF warnings in logs

- SLAM checks:
  - map is published by `slam_toolbox` (e.g. `/map`)
  - map and localization in RViz look consistent with robot motion


## Notes

- This port intentionally removed rosbag orchestration and map-save debug flow from the Python script.
- Existing uncommitted changes in `experimental/slam.py` were left untouched.
