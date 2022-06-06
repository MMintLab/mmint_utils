import unittest
from mmint_utils.test_utils import poses_equal, poses_equal_batch
import tf.transformations as tf
import numpy as np


class TestTestUtils(unittest.TestCase):

    def test_poses_equal(self):
        np.random.seed(42)

        pose_a = np.zeros(7, dtype=float)
        pose_a[:3] = np.random.rand(3)
        pose_a[3:] = tf.random_quaternion()

        self.assertTrue(poses_equal(pose_a, pose_a))

        pose_b = np.zeros(7, dtype=float)
        pose_b[:3] = np.random.rand(3)
        pose_b[3:] = tf.random_quaternion()
        self.assertFalse(poses_equal(pose_a, pose_b))

    def test_poses_equal_batch(self):
        np.random.seed(42)

        pose_a = np.zeros([10, 7], dtype=float)
        for pose_idx in range(10):
            pose_a[pose_idx, :3] = np.random.rand(3)
            pose_a[pose_idx, 3:] = tf.random_quaternion()

        self.assertTrue(poses_equal_batch(pose_a, pose_a))

        pose_b = np.zeros([10, 7], dtype=float)
        for pose_idx in range(10):
            pose_b[pose_idx, :3] = np.random.rand(3)
            pose_b[pose_idx, 3:] = tf.random_quaternion()

        self.assertFalse(poses_equal_batch(pose_a, pose_b))


if __name__ == '__main__':
    unittest.main()
