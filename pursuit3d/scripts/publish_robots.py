#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import numpy as np
import tf

def odometry_cb(msg):

    pose = msg.pose.pose
    br.sendTransform((pose.position.x, pose.position.y, pose.position.z),
                     (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                     msg.header.stamp,
                     "base_link",
                     "map")


if __name__ == "__main__":

    rospy.init_node('simple_nav', anonymous=True)
    br = tf.TransformBroadcaster()


    rospy.Subscriber('/ground_truth/state', Odometry, odometry_cb)
    rospy.spin()

