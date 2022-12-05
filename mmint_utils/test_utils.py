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


def poses_equal_batch(poses_a: np.ndarray, poses_b: np.ndarray, rtol: float = 1e-05, atol: float = 1e-08) -> bool:
    """
    Determine if the two given pose arrays
    are equivalent. This function accounts for
    non-uniqueness of quaternions by comparing
    the corresponding transformation matrices.

    Last dim of arrays should be:
    [x, y, z, orn_x, orn_y, orn_z, orn_w]

    Args:
        poses_a (np.ndarray): array of poses a
        poses_b (np.ndarray): array of poses b
        rtol (float): relative tolerance as used by np.allclose for comparing the matrices
        atol (float): absolute tolerance as used by np.allclose for comparing the matrices
    Returns:
        poses_equal (bool): whether the given poses are equal up to tolerances
    """
    num_poses = poses_a.shape[0]
    assert poses_b.shape[0] == num_poses

    poses_equal_ = True

    for pose_idx in range(num_poses):
        poses_equal_ = poses_equal_ and poses_equal(poses_a[pose_idx], poses_b[pose_idx], rtol, atol)

    return poses_equal_


def dict_equal(dict_a: dict, dict_b: dict):
    """
    Determine equality of two dictionaries. Handles numpy array equality and recurses to handle dicts.

    Args:
        dict_a (dict)
        dict_b (dict)
    Returns:
        equal (bool): whether the two provided dictionaries contain equal data.
    """
    for k in dict_a:
        if k not in dict_b or type(dict_a[k]) != type(dict_b[k]):
            return False

        if type(dict_a[k]) is np.ndarray:
            if not (dict_a[k] == dict_b[k]).all():
                return False
        elif type(dict_a[k]) is dict:
            if not dict_equal(dict_a[k], dict_b[k]):
                return False
        else:
            if not (dict_a[k] == dict_b[k]):
                return False

    return True
