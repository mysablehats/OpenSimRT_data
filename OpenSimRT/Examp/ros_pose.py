#!/usr/bin/env python3

# from https://answers.ros.org/question/316829/how-to-publish-a-pose-in-quaternion/?answer=316844#post-id-316844
import rospy
from geometry_msgs.msg import Pose

def publisher():
    pub = rospy.Publisher('pose', Pose, queue_size=1)
    rospy.init_node('pose_publisher', anonymous=True)
    rate = rospy.Rate(2) # Hz
    while not rospy.is_shutdown():
        p = Pose()
        p.position.x = 0.5
        p.position.y = -0.1
        p.position.z = 1.0
        # Make sure the quaternion is valid and normalized
        p.orientation.x = 0.0
        p.orientation.y = 0.0
        p.orientation.z = 0.0
        p.orientation.w = 1.0
        pub.publish(p)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy:
        pass
