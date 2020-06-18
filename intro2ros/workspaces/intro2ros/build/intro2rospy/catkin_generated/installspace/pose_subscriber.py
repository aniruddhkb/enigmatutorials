#!/usr/bin/env python2
import rospy 
from turtlesim.msg import Pose

def callback(msg):
    rospy.loginfo(str(rospy.get_name()) + " heard " + str(msg)) 

def pose_subscriber():

    rospy.init_node("pose_subscriber", anonymous=True)
    rospy.Subscriber("/turtle1/pose", Pose, callback, queue_size=1000)
    rospy.spin()

if __name__ == "__main__":
    pose_subscriber()