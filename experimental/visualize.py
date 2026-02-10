import pickle
import numpy as np
import matplotlib.pyplot as plt
from utils import pose_to_matrix

with open('scan_data.pkl', 'rb') as f:
    data = pickle.load(f)

raw_clouds = data['raw_clouds']
poses = data['t265_pose']

step = 20

for start in range(0, len(raw_clouds), step):
    end = min(start + step, len(raw_clouds))
    fig, ax = plt.subplots()
    ax.set_aspect('equal')
    ax.set_title(f'Scans {start}–{end - 1}')

    for i in range(start, end):
        if poses[i] is None or len(raw_clouds[i]) == 0:
            continue
        x, y, yaw = poses[i][0], poses[i][1], poses[i][2]
        T = pose_to_matrix(x, y, yaw)
        cloud = np.array(raw_clouds[i])  # (N, 2)
        ones = np.ones((cloud.shape[0], 1))
        pts_h = np.hstack([cloud, ones])  # (N, 3)
        world = (T @ pts_h.T).T  # (N, 3)
        ax.scatter(world[:, 0], world[:, 1], s=0.3)

    # mark robot positions
    for i in range(start, end):
        if poses[i] is None:
            continue
        ax.plot(poses[i][0], poses[i][1], 'rx', markersize=6)

    plt.show()
