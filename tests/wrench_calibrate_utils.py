import rospy
import numpy as np

from geometry_msgs.msg import Wrench, TransformStamped, WrenchStamped
from std_srvs.srv import Empty
from tf import TransformListener
import tf
import tf2_ros as tf2
import copy
import argparse

'''
read the wrench from a topic, and publish the calibrated wrench, also publish a frame parallel to /world
'''

class WrenchCalibrator():

    def __init__(self, wrench_topic='/med/wrench'):
        self.wrench_topic = wrench_topic
        self.reference_wrench = Wrench()
        self.current_wrench = Wrench()
        self.parallel_frame_pub = tf.TransformBroadcaster()
        self.tf_listener = TransformListener()
        self.wrench_sub = rospy.Subscriber(self.wrench_topic, WrenchStamped, callback=self.wrench_callback)
        self.wrench_pub = rospy.Publisher(self.wrench_topic+'_calibrated', WrenchStamped, queue_size=0)
        # self.parallel_wrench_pub = rospy.Publisher(self.wrench_topic+'_parallel', WrenchStamped, queue_size=0)
        ## TODO: add a service to calibrate the wrench
        self.calibrate_srv = rospy.Service('calibrate_wrench', Empty, self.calibrate_reference_wrench)
        rospy.Rate(100)


    def wrench_callback(self, msg):
        self.current_wrench = msg.wrench
        wrench_calibrated = self.calibrate_wrench(self.current_wrench)
        if wrench_calibrated is not None:
            wrench_calibrated.header.stamp = rospy.Time.now()
            # print(wrench_calibrated.header.stamp.to_sec())
            wrench_calibrated.header.frame_id = msg.header.frame_id
            self.wrench_pub.publish(wrench_calibrated)
            self.parallelize_frame(msg.header.frame_id)
        return

    def parallelize_frame(self, frame_id):
        '''make a new frame parallel to the current frame, this frame has the same position, but different orientation, then publish it'''
        # get the transform from wrench_frame to base_frame
        wrench_frame_pose = self.get_transform(frame_id, 'world')
        # get the transform from base_frame to world
        self.parallel_frame_pub.sendTransform((wrench_frame_pose[0][0], wrench_frame_pose[0][1], wrench_frame_pose[0][2]),
                                              (0, 0, 0, 1),
                                              rospy.Time.now(),
                                              frame_id+'_parallel',
                                              'world')

    def get_transform(self, frame_id, target_frame='world'):
        '''get the transform from frame_id to target_frame'''
        t = self.tf_listener.getLatestCommonTime(target_frame, frame_id)
        try:
            transform = self.tf_listener.lookupTransform(target_frame, frame_id, t)
            return transform
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr('Failed to get transform from %s to %s' % (frame_id, target_frame))
            return None

    def calibrate_wrench(self, wrench):
        wrench_calibrated = WrenchStamped()
        wrench_calibrated.wrench.force.x = wrench.force.x - self.reference_wrench.force.x
        wrench_calibrated.wrench.force.y = wrench.force.y - self.reference_wrench.force.y
        wrench_calibrated.wrench.force.z = wrench.force.z - self.reference_wrench.force.z
        wrench_calibrated.wrench.torque.x = wrench.torque.x - self.reference_wrench.torque.x
        wrench_calibrated.wrench.torque.y = wrench.torque.y - self.reference_wrench.torque.y
        wrench_calibrated.wrench.torque.z = wrench.torque.z - self.reference_wrench.torque.z
        return wrench_calibrated

    def calibrate_reference_wrench(self, _):
        self.reference_wrench = self.current_wrench
        return []

    

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('wrench_topic', type=str, help='Name of the wrench topic', default='/med/wrench')
    parser.add_argument('node_name', type=str, help='Name of the node', default='wrench_calibrated')
    args = parser.parse_args()

    rospy.init_node(args.node_name)
    wrench_calibrator = WrenchCalibrator(args.wrench_topic)
    rospy.spin()

