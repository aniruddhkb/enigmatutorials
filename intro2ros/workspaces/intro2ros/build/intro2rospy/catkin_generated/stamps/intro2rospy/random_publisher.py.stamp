#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import random
def random_publisher():
    rospy.init_node('random_publisher', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1000)
    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        to_publish = Twist()
        to_publish.linear.x =random.uniform(0, 1)
        to_publish.angular.z = random.uniform(-1, 1)
        rospy.loginfo(str(to_publish))
        pub.publish(to_publish)
        rate.sleep()

if __name__ == "__main__":
    random_publisher()