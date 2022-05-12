import arc_utilities.transformation_helper as tf_helper
import numpy as np


# This file meant to have helpers for writing unit tests.


def poses_equal(pose_a: np.ndarray, pose_b: np.ndarray, rtol: float = 1e-05, atol: float = 1e-08) -> bool:
    """
    Determine if the two given pose arrays
    are equivalent. This function accounts for
    non-uniqueness of quaternions by comparing
    the corresponding transformation matrices.

    Pose arrays should be:
    [x, y, z, orn_x, orn_y, orn_z, orn_w]

    Args:
        pose_a (np.ndarray): array of pose a
        pose_b (np.ndarray): array of pose b
        rtol (float): relative tolerance as used by np.allclose for comparing the matrices
        atol (float): absolute tolerance as used by np.allclose for comparing the matrices
    Returns:
        poses_equal (bool): whether the given poses are equal up to tolerances
    """
    transform_matrix_a = tf_helper.BuildMatrix(pose_a[:3], pose_a[3:])
    transform_matrix_b = tf_helper.BuildMatrix(pose_b[:3], pose_b[3:])

    return np.allclose(transform_matrix_a, transform_matrix_b, rtol=rtol, atol=atol)
