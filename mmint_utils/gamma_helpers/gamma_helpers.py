import rospy
from netft_rdt_driver.srv import Zero


def zero_ati_gamma():
    rospy.wait_for_service("/netft/zero")
    try:
        zero_srv = rospy.ServiceProxy("/netft/zero", Zero)
        zero_srv()
    except rospy.ServiceException as e:
        print("Zeroing gamma failed...")
        return False
    return True
