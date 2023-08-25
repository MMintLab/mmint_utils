## This script is to set the gamma sensor to different posese to acquire the gravity parameters
## now only for med

import rospy
import numpy as np
import tf.transformations as tr
import time

from geometry_msgs.msg import WrenchStamped, PoseStamped
from bubble_utils.bubble_med.bubble_med import BubbleMed
from mmint_utils.gamma_helpers import zero_ati_gamma

from mmint_tools import tr


fixed_orientation_quat_down = tr.quaternion_from_euler(0, np.pi, np.pi*0.5, axes='sxyz')     # this is the down orientation
fixed_orientation_quat_up = tr.quaternion_from_euler(0, np.pi, np.pi*1.5, axes='sxyz')  # this is the up orientation
fixed_orientation_quat_horizontal_left = tr.quaternion_from_euler(0, np.pi, np.pi, axes='sxyz')  # this is the horizontal left orientation
fixed_orientation_quat_horizontal_right = tr.quaternion_from_euler(0, np.pi, 0, axes='sxyz')  # this is the horizontal right orientation

fixed_position_down = np.array([0.5, 0, 0.3])
fixed_position_up = np.array([0.5, 0, 0.75])
fixed_position_horizontal = np.array([0.7, 0, 0.5])


def wrench_record_callback(data):
    pass

if __name__ == '__main__':
    '''
    This script is to set the gamma sensor to different posese to acquire the gravity parameters
    1. set the gamma sensor to vertical, down pose, and zero the gamma sensor
    2. set the gamma sensor to vertical, up pose, and get the measurement, as 2*mg
    3. set the gamma sensor to horizontal, left pose, both to validate mg, and also using the torque to get the center of mass in one direction
    4. set the gamma sensor to horizontal, right pose, both to validate mg, and also using the torque to get the center of mass in another direction
    Then save the parameters, for the gravity compensation
    '''
    # pass
    rospy.init_node('gamma_on_hand_gravity_parameters_acquisition')
    med = BubbleMed()
    netft_sub = rospy.Subscriber('/netft/netft_data', WrenchStamped, wrench_record_callback)
    # set the gamma sensor to vertical, down pose, and zero the gamma sensor
    