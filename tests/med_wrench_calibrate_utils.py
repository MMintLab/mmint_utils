import rospy
import numpy as np

from geometry_msgs.msg import Wrench, TransformStamped, WrenchStamped
from std_srvs.srv import Empty
from tf import TransformListener
import tf

class WrenchCalibrator():

    def __init__(self, wrench_topic='/med/wrench'):
        self.wrench_topic = wrench_topic
        self.reference_wrench = Wrench()
        self.current_wrench = Wrench()
        self.wrench_sub = rospy.Subscriber(self.wrench_topic, WrenchStamped, callback=self.wrench_callback)
        self.wrench_pub = rospy.Publisher(self.wrench_topic+'_calibrated', WrenchStamped, queue_size=0)
        ## TODO: add a service to calibrate the wrench
        self.calibrate_srv = rospy.Service('calibrate_wrench', Empty, self.calibrate_reference_wrench)

    def wrench_callback(self, msg):
        self.current_wrench = msg.wrench
        wrench_calibrated = self.calibrate_wrench(self.current_wrench)
        if wrench_calibrated is not None:
            wrench_calibrated.header.stamp = rospy.Time.now()
            wrench_calibrated.header.frame_id = msg.header.frame_id
            self.wrench_pub.publish(wrench_calibrated)
        return

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
    rospy.init_node('wrench_calibrator')
    wrench_calibrator = WrenchCalibrator('med/wrench')
    rospy.spin()

