FROM ros:humble-ros-base

# Set the working directory
WORKDIR /app

# Install system dependencies and scipy
# Using apt-get is faster and more reliable for scipy on ROS images
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-scipy \
    && rm -rf /var/lib/apt/lists/*

# Copy your files
COPY rosbag2_2026_01_23-21_02_43 /app/bags/rosbag2_2026_01_23-21_02_43
COPY rosbag2_2026_01_23-21_01_26 /app/bags/rosbag2_2026_01_23-21_01_26
COPY ros.py /app/ros.py
COPY utils.py /app/utils.py
COPY icp.py /app/icp.py

# Ensure your workspace is sourced and run your script
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && python3 /app/ros.py"]