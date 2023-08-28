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

gravity_acc = 9.81

def wrench_record_callback(data):
    pass

def wrench_record_filter(filter_length=10):
    # get the wrench data from the subscriber, and filter the data, for now just use the average
    wrench_data = []
    for i in range(filter_length):
        current_wrench_stamped = rospy.wait_for_message('/netft/netft_data', WrenchStamped)
        current_wrench_data = [current_wrench_stamped.wrench.force.x, current_wrench_stamped.wrench.force.y, current_wrench_stamped.wrench.force.z, current_wrench_stamped.wrench.torque.x, current_wrench_stamped.wrench.torque.y, current_wrench_stamped.wrench.torque.z]
        wrench_data.append(current_wrench_data)
    wrench_data = np.array(wrench_data)
    wrench_data = np.mean(wrench_data, axis=0)
    return wrench_data
    
def process_gravity_parameters(wrench_up, wrench_left, wrench_right):
    '''
    This function is to process the gravity parameters, and return the gravity parameters 
    which are mass, center of mass in gamma frame, now ignore the inertia  
    'left', 'right' are just labels for different wrench
    '''
    
    equivalent_wrench = - 0.5 * wrench_up # this is the equivalent wrench applied to gamma wehn down pose caused by gravity, only to de-zero gamma
    
    mass, center_x, center_y = wrench_to_mass_center(equivalent_wrench)
    center = [center_x, center_y]
    return equivalent_wrench, mass, center
                            
def wrench_to_mass_center(wrench):
    # Assuming vertical, recover mass and dist for compensation
    mass = wrench[2] / gravity_acc
    # also assuming force is only along Z axis, then center_x and center_y are:
    center_x = - wrench[4] / wrench[2]
    center_y = wrench[3] / wrench[2]        # TODO: check signs
    center = np.cross()
    return mass, center
    
    
    

if __name__ == '__main__':
    '''
    This script is to set the gamma sensor to different posese to acquire the gravity parameters
    1. set the gamma sensor to vertical, down pose, and zero the gamma sensor
    2. set the gamma sensor to vertical, up pose, and get the measurement, as 2*mg
    #3. set the gamma sensor to horizontal, left pose, both to validate mg, and also using the torque to get the center of mass in one direction
    #4. set the gamma sensor to horizontal, right pose, both to validate mg, and also using the torque to get the center of mass in another direction
    Then save the parameters, for the gravity compensation
    '''
    # pass
    rospy.init_node('gamma_on_hand_gravity_parameters_acquisition')
    med = BubbleMed()
    # netft_sub = rospy.Subscriber('/netft/netft_data', WrenchStamped, wrench_record_callback)
    # set the gamma sensor to vertical, down pose, and zero the gamma sensor
    med.set_raw_pose(np.concatenate([fixed_position_down, fixed_orientation_quat_down]), blocking=True)
    rospy.sleep(2)
    zero_ati_gamma()
    rospy.sleep(2)
    # set the gamma sensor to vertical, up pose, and get the measurement, as 2*mg
    med.set_raw_pose(np.concatenate([fixed_position_up, fixed_orientation_quat_up]), blocking=True)
    rospy.sleep(2)
    up_wrench_data = wrench_record_filter(filter_length=10)
    # set the gamma sensor to horizontal, left pose, both to validate mg, and also using the torque to get the center of mass in one direction
    med.set_raw_pose(np.concatenate([fixed_position_horizontal, fixed_orientation_quat_horizontal_left]), blocking=True)
    rospy.sleep(2)
    left_wrench_data = wrench_record_filter(filter_length=10)
    # # set the gamma sensor to horizontal, right pose, both to validate mg, and also using the torque to get the center of mass in another direction
    med.set_raw_pose(np.concatenate([fixed_position_horizontal, fixed_orientation_quat_horizontal_right]), blocking=True)
    rospy.sleep(2)
    right_wrench_data = wrench_record_filter(filter_length=10)
    
    gravity_params = process_gravity_parameters(up_wrench_data, left_wrench_data, right_wrench_data)
    
    ## store the gravity parameters
    
    
    
    
    