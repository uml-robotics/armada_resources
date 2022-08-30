#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import JointState

def talker():
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rospy.init_node('gripper_state_publisher', anonymous=True)
    rate = rospy.Rate(50) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
