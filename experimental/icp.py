import math
from scipy.spatial import KDTree
import numpy as np

def icp(old_cloud, cloud):
    oc_arr = np.array(old_cloud)
    c_arr = np.array(cloud)

    old_centroid = np.mean(oc_arr, axis=0)
    centroid = np.mean(c_arr, axis=0)

    oc_arr = oc_arr - old_centroid
    c_arr = c_arr - centroid

    R = np.identity(2)
    prev_error = math.inf
    max_iters = 50
    tol = 1e-4
    tree = KDTree(oc_arr)
    for _ in range(max_iters):
        # for each point in cloud, find the nearest point in oc
        # to construct the cross covariance matrix
        dists, indices = tree.query(c_arr)

        error = np.mean(dists)
        cross_covariance = oc_arr[indices].T @ c_arr

        U, _, Vt = np.linalg.svd(cross_covariance)

        R_iter = U @ Vt
        if np.linalg.det(R_iter) < 0:
            Vt[1, :] *= -1
            R_iter = U @ Vt
        R = R_iter @ R

        if abs(prev_error - error) < tol:
            break

        # rotate every point in the new cloud and redo
        c_arr = (R_iter @ c_arr.T).T
        prev_error = error

    translation = old_centroid - R @ centroid

    return np.array([[R[0][0], R[0][1], translation[0]], [R[1][0] ,R[1][1], translation[1]], [0,0,1]])
