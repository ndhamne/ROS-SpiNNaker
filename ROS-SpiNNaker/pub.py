import rospy
from random import randint
from std_msgs.msg import Int32

class RanNumGen:

# Initialising the class
    def __init__(self):
        print "class defined"

# Callback function for ROS node to know which neuron is spiked
    def spike_callback(msg):
        print "Neuron {} spiked !".format(msg.data)

# Initialise node and start publishing
    rospy.init_node('spinnaker_side')
    pub = rospy.Publisher('generator', Int32 , queue_size = 0)

    rate = rospy.Rate(4) # Hz (per sec)

# Initialise the Subscriber that gets the neuron that spiked
    sub = rospy.Subscriber('snn_spiker', Int32, spike_callback)

# keep generating a random number from 0 - 99
    while not rospy.is_shutdown():
        num = randint(0,99)
        rospy.loginfo(num)
        pub.publish(num)
        rate.sleep()
