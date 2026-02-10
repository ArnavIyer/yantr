from utils import matrix_to_pose, pose_to_matrix
import numpy as np

tests = [
    lambda : np.linalg.norm(matrix_to_pose(pose_to_matrix(1,1,np.pi/4)) - np.array([1, 1, np.pi/4])) < 1e-2,
    lambda : np.linalg.norm(matrix_to_pose(pose_to_matrix(-2,3,-np.pi/2)) - np.array([-2, 3, -np.pi/2])) < 1e-2,
    lambda : np.linalg.norm(matrix_to_pose(pose_to_matrix(0,0,np.pi)) - np.array([0, 0, np.pi])) < 1e-2,
    lambda : np.linalg.norm(matrix_to_pose(pose_to_matrix(5,-5,0)) - np.array([5, -5, 0])) < 1e-2,
]

for i, test in enumerate(tests):
    assert test(), f"Test {i} failed."
print("All tests passed.")