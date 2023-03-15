import rospy
import numpy as np
# import scipy
from geometry_msgs.msg import Wrench, TransformStamped, WrenchStamped
from tf import TransformListener
import tf
import tf.transformations as tr
import time
import copy
import argparse

class WrenchTransform(object):

    def __init__(self, wrench_frame, base_frame, sub_wrench_topic='/netft/netft_data', pub_wrench_topic='/netft/netft_data_transformed', flip=False):
        self.wrench_topic = sub_wrench_topic
        self.pub_wrench_topic = pub_wrench_topic
        self.wrench_frame = wrench_frame # the frame that the wrench is measured in, or sensor
        self.base_frame = base_frame    # the frame that the wrench is transformed to

        self.wrench_sub = rospy.Subscriber(self.wrench_topic, WrenchStamped, callback=self.wrench_callback)
        self.transformed_wrench = None
        self.wrench = None
        self.wrench_time = None
        self.wrench_transformed = None
        self.wrench_transformed_time = None
        self.tf_listener = TransformListener()
        self.wrench_lock = True         # lock publisher if true until wrench is received, and we have both frames
        self.flip = flip              # flip the wrench if true

        self.wrench_pub = rospy.Publisher(self.pub_wrench_topic, WrenchStamped, queue_size=1)

    def wrench_callback(self, msg):
        self.wrench = msg.wrench
        self.wrench_time = msg.header.stamp
        wrench_frame = msg.header.frame_id
        # we assume the frame_id of this wrench is the same as the wrench_frame, TODO: check this
        self.wrench_transformed = self.transform_wrench(self.wrench, wrench_frame, self.base_frame)
        if self.wrench_transformed is not None:
            self.wrench_transformed_time = rospy.Time.now()
            self.wrench_transformed.header.stamp = self.wrench_transformed_time
            self.wrench_pub.publish(self.wrench_transformed)
        return

    def transform_wrench(self, wrench, wrench_frame, base_frame):
        # get the transform from wrench_frame to base_frame
        if self.wrench_lock:
            # if self.tf_listener.frameExists(wrench_frame) and self.tf_listener.frameExists(base_frame):
            #     self.wrench_lock = False
            # else:
            #     return
            self.wrench_lock = False
        t = self.tf_listener.getLatestCommonTime(wrench_frame, base_frame)
        frame_transform = self.tf_listener.lookupTransform(wrench_frame, base_frame, t)
        wrench_to_transform = copy.deepcopy(wrench)
        transformed_wrench = self.wrench_to_frame(wrench_to_transform, frame_transform) # data type is Wrench
        # make a WrenchStamped
        wrench_stamped = WrenchStamped()
        wrench_stamped.header.stamp = t
        wrench_stamped.header.frame_id = base_frame
        wrench_stamped.wrench = transformed_wrench
        return wrench_stamped

            
    def wrench_to_frame(self, wrench, frame_transform):
        wrench_in_frame = Wrench()
        transform_translation = -np.array(frame_transform[0])
        transform_rotation = np.array(frame_transform[1])
        ## TODO wrench transform 
        norm = np.linalg.norm(transform_rotation)
        if np.abs(norm - 1.0) > 1e-3:
            raise ValueError(
                "Received un-normalized quaternion (transform_rotation = {0:s} ||transform_rotation|| = {1:3.6f})".format(
                    str(transform_rotation), np.linalg.norm(transform_rotation)))
        elif np.abs(norm - 1.0) > 1e-6:
            transform_rotation = transform_rotation / norm
        transform_matrix = tr.quaternion_matrix(transform_rotation)
        transform_rotation_matrix = transform_matrix[0:3,0:3]
        transform_matrix[0:3, -1] = transform_translation

        transform_matrix_adj = np.cross(transform_translation, transform_rotation_matrix)

        transform_adjoint_matrix = np.concatenate((np.concatenate((transform_rotation_matrix,np.zeros([3,3])),axis = 1),np.concatenate((transform_matrix_adj,transform_rotation_matrix),axis = 1)),axis = 0)

        wrench_vector = np.array([wrench.torque.x,wrench.torque.y,wrench.torque.z,wrench.force.x,wrench.force.y,wrench.force.z])

        wrench_vector_frame = np.matmul(np.transpose(transform_adjoint_matrix), wrench_vector)
        if self.flip:
            wrench_vector_frame = -wrench_vector_frame

        wrench_in_frame.force.x = wrench_vector_frame[3]
        wrench_in_frame.force.y = wrench_vector_frame[4]
        wrench_in_frame.force.z = wrench_vector_frame[5]
        wrench_in_frame.torque.x = wrench_vector_frame[0]
        wrench_in_frame.torque.y = wrench_vector_frame[1]
        wrench_in_frame.torque.z = wrench_vector_frame[2]

        return wrench_in_frame
    
    def get_transformed_wrench(self):
        # return data type is Wrench
        if self.wrench_transformed is not None:
            return self.wrench_transformed.wrench
        
    def get_raw_wrench(self):
        # return data type is Wrench
        if self.wrench is not None:
            return self.wrench
    
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('wrench_frame', type=str, help='Name of the frame that the wrench is in', default='med_kuka_link_ee')
    parser.add_argument('base_frame', type=str, help='Name of the frame that the wrench is transformed to', default='med_kuka_link_ee_parallel')
    parser.add_argument('sub_wrench_topic', type=str, help='Name of the wrench topic', default='/med/wrench_calibrated')
    parser.add_argument('pub_wrench_topic', type=str, help='Name of the transformed wrench topic', default='/med/wrench_calibrated_transformed')
    parser.add_argument('node_name', type=str, help='Name of the node', default='wrench_transform') 
    parser.add_argument('--flip', type=str, help='Flip the wrench', default='False')
    args = parser.parse_args()

    node_name = rospy.get_param('~node_name', args.node_name)
    flip = rospy.get_param('~flip', args.flip)=='True'
    rospy.init_node(node_name)
    #args to get the wrench frame and base frame
    
    wrench_frame = rospy.get_param('~wrench_frame', args.wrench_frame)
    base_frame = rospy.get_param('~base_frame', args.base_frame)
    sub_wrench_topic = rospy.get_param('~sub_wrench_topic', args.sub_wrench_topic)
    pub_wrench_topic = rospy.get_param('~pub_wrench_topic', args.pub_wrench_topic)
    wrench_transform = WrenchTransform(wrench_frame, base_frame, sub_wrench_topic, pub_wrench_topic, flip)
    rospy.spin()