import pickle
import numpy as np
import matplotlib.pyplot as plt
from utils import pose_to_matrix

with open('scan_data.pkl', 'rb') as f:
    data = pickle.load(f)

raw_clouds = data['raw_clouds']
poses = data['t265_pose']
icp_transforms = data['icp_transforms']

# Build global poses by composing ICP transforms, anchored at first odometry pose
global_poses = []
if len(poses) > 0:
    if poses[0] is not None:
        global_poses.append(pose_to_matrix(poses[0][0], poses[0][1], poses[0][2]))
    else:
        global_poses.append(np.eye(3))
for i in range(len(icp_transforms)):
    global_poses.append(global_poses[-1] @ icp_transforms[i])

step = 40

for start in range(0, len(raw_clouds), step):
    end = min(start + step, len(raw_clouds))
    fig, (ax_odom, ax_icp) = plt.subplots(1, 2, figsize=(14, 6))

    ax_odom.set_aspect('equal')
    ax_odom.set_title(f'Scans {start}–{end - 1} (T265 Odometry)')
    ax_icp.set_aspect('equal')
    ax_icp.set_title(f'Scans {start}–{end - 1} (ICP)')

    for i in range(start, end):
        cloud = np.array(raw_clouds[i])
        if len(cloud) == 0:
            continue

        # T265 odometry
        if poses[i] is not None:
            T_odom = pose_to_matrix(poses[i][0], poses[i][1], poses[i][2])
            world_odom = (T_odom @ cloud.T).T
            ax_odom.scatter(world_odom[:, 0], world_odom[:, 1], s=0.3)

        # ICP
        if i < len(global_poses):
            T_icp = global_poses[i]
            world_icp = (T_icp @ cloud.T).T
            ax_icp.scatter(world_icp[:, 0], world_icp[:, 1], s=0.3)

    # mark robot positions
    for i in range(start, end):
        if poses[i] is not None:
            ax_odom.plot(poses[i][0], poses[i][1], 'rx', markersize=6)
        if i < len(global_poses):
            ax_icp.plot(global_poses[i][0, 2], global_poses[i][1, 2], 'rx', markersize=6)

    plt.tight_layout()
    plt.show()
