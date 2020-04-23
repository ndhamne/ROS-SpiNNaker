#!/usr/bin/env python

import rospy
from std_msgs.msg import *
from random import randint

# ROS node publishing message
rospy.init_node("msg_generator")
pub = rospy.Publisher('generator', Int32 , queue_size = 0)
rate = rospy.Rate(200)

# Publishing a number for a certain number of times
# so that the networks stabilizes on a particular neuron
# and publish -1 to indicate no input from ROS side
# Alternating between a number and -1 (no input) to
# simulate the inputs from the ROS node/ Robot sensors
rospy.loginfo(-1)
for i in range(100):
    pub.publish(-1)
    rate.sleep()

rospy.loginfo(5)
for j in range(15):
    pub.publish(5)
    rate.sleep()

rospy.loginfo(-1)
for i in range(300):
    pub.publish(-1)
    rate.sleep()

rospy.loginfo(10)
for j in range(15):
    pub.publish(10)
    rate.sleep()

rospy.loginfo(-1)
for i in range(300):
    pub.publish(-1)
    rate.sleep()

rospy.loginfo(15)
for j in range(15):
    pub.publish(15)
    rate.sleep()
