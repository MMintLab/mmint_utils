#! /usr/bin/env python
# This scripts is to compensate the netft measurement on hand with the gravity compensation, read parameters from file

import rospy
import numpy as np
import argparse
import os

from geometry_msgs.msg import WrenchStamped, PoseStamped
from mmint_utils.config import dump_cfg, load_cfg
from tf import TransformListener
import tf.transformations as tr
import copy

if __name__ == '__main__':
    parser = argparse.ArgumentParser('Gamma_on_hand_compensation')
    parser.add_argument('--params_file_name', default='gravity_params', help='name of the configuration file for gravity compensation')
    args = parser.parse_args()
    # if args.gamma_on_hand:

    project_path = os.path.join(os.path.dirname(os.path.abspath(__file__)).split('/mmint_utils')[0], 'mmint_utils')
    rospy.init_node("netft_compensated")
    gravity_cfg = load_cfg(os.path.join(project_path, 'tests', 'gravity_compensation', f'config/{args.params_file_name}.yaml'))
    equivalent_wrench = gravity_cfg['equivanlent_wrench']
    mass = gravity_cfg['mass']
    center = gravity_cfg['center']
    gravity_acc = 9.81
    gravity_vector  = np.array([0, 0, - mass * gravity_acc])
    center_in_gamma_frame = np.array(center)
    
    # compensated_wrench pub:
    compensated_wrench_pub = rospy.Publisher('/netft/compensated_netft_data', WrenchStamped)
    # tf for C.o.M, with fixed orientation in world frame
    tf_listener = TransformListener()
    world_frame_name = 'world'
    gamma_frame_name = 'gamma_on_hand_link_ft'
    
    def wrench_callback(data):
        '''
            feed in wrench from netft, and then publish the compensated wrench
        '''
        
        tf_listener.waitForTransform(world_frame_name, gamma_frame_name, rospy.Time(), rospy.Duration(1.0))
        t = tf_listener.getLatestCommonTime(world_frame_name, gamma_frame_name)
        world_to_gamma_frame_transform = tf_listener.lookupTransform(gamma_frame_name, world_frame_name, t)
        world_to_gamma_quaternion = np.array(world_to_gamma_frame_transform[1])
        world_to_gamma_rotation_matrix = tr.quaternion_matrix(world_to_gamma_quaternion)
        force_in_gamma_frame = world_to_gamma_rotation_matrix[0:3,0:3] @ gravity_vector
        torque_in_gamma_frame = np.cross(center_in_gamma_frame , force_in_gamma_frame)
        
        new_wrenchstamped = copy.deepcopy(data)
        new_wrenchstamped.wrench.force.x = data.wrench.force.x + equivalent_wrench[0] - force_in_gamma_frame[0]
        new_wrenchstamped.wrench.force.y = data.wrench.force.y + equivalent_wrench[1] - force_in_gamma_frame[1]
        new_wrenchstamped.wrench.force.z = data.wrench.force.z + equivalent_wrench[2] - force_in_gamma_frame[2]
        new_wrenchstamped.wrench.torque.x = data.wrench.torque.x + equivalent_wrench[3] - torque_in_gamma_frame[0]
        new_wrenchstamped.wrench.torque.y = data.wrench.torque.y + equivalent_wrench[4] - torque_in_gamma_frame[1]
        new_wrenchstamped.wrench.torque.z = data.wrench.torque.z + equivalent_wrench[5] - torque_in_gamma_frame[2]
                
        compensated_wrench_pub.publish(new_wrenchstamped)
        return 
    
    # netft_sub:
    netft_sub = rospy.Subscriber('/netft/netft_data', WrenchStamped, wrench_callback)
    # get the current com frame in gamma's frame
    rospy.spin()


    
        
        
