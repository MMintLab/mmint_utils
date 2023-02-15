import rospy
import numpy as np
# import scipy
from geometry_msgs.msg import Wrench, TransformStamped, WrenchStamped
from tf import TransformListener
import tf
import tf.transformations as tr
import time
import copy

class WrenchTransform(object):

    def __init__(self, wrench_frame, base_frame, wrench_topic='/netft/netft_data'):
        self.wrench_topic = wrench_topic
        self.wrench_frame = wrench_frame # the frame that the wrench is measured in, or sensor
        self.base_frame = base_frame    # the frame that the wrench is transformed to

        self.wrench_sub = rospy.Subscriber(self.wrench_topic, WrenchStamped, callback=self.wrench_callback)
        self.transformed_wrench = None
        self.wrench_time = None
        self.wrench_transformed = None
        self.wrench_transformed_time = None
        self.tf_listener = TransformListener()
        self.wrench_lock = True         # lock publisher if true until wrench is received, and we have both frames

        self.wrench_pub = rospy.Publisher('/wrench_transformed', WrenchStamped, queue_size=1)

    def wrench_callback(self, msg):
        self.wrench = msg.wrench
        self.wrench_time = msg.header.stamp
        wrench_frame = msg.header.frame_id
        # we assume the frame_id of this wrench is the same as the wrench_frame, TODO: check this
        self.wrench_transformed = self.transform_wrench(self.wrench, self.wrench_frame, self.base_frame)
        if self.wrench_transformed is not None:
            self.wrench_transformed_time = rospy.Time.now()
            self.wrench_pub.publish(self.wrench_transformed)
        return

    def transform_wrench(self, wrench, wrench_frame, base_frame):
        # get the transform from wrench_frame to base_frame
        if self.wrench_lock:
            if self.tf_listener.frameExists(wrench_frame) and self.tf_listener.frameExists(base_frame):
                self.wrench_lock = False
            else:
                return
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

        wrench_in_frame.force.x = wrench_vector_frame[3]
        wrench_in_frame.force.y = wrench_vector_frame[4]
        wrench_in_frame.force.z = wrench_vector_frame[5]
        wrench_in_frame.torque.x = wrench_vector_frame[0]
        wrench_in_frame.torque.y = wrench_vector_frame[1]
        wrench_in_frame.torque.z = wrench_vector_frame[2]

        return wrench_in_frame
    
    def get_wrench(self):
        if self.wrench_transformed is not None:
            return self.wrench_transformed
    
if __name__ == '__main__':
    rospy.init_node('wrench_transform')
    #args to get the wrench frame and base frame
    wrench_frame = rospy.get_param('~wrench_frame', 'netft')
    base_frame = rospy.get_param('~base_frame', 'base_link')
    wrench_transform = WrenchTransform(wrench_frame, base_frame)
    rospy.spin()