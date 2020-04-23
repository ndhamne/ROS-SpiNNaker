#!/usr/bin/env python

from spynnaker8.external_devices import SpynnakerLiveSpikesConnection
from thread import allocate_lock
import rospy
from std_msgs.msg import *
import sys
import random
import numpy as np
import pdb

num = 0

# Injecting spikes in neurons when
# a input message is received from the ROS node
class SpikeInjector(object):

    def __init__(self):
        print("INJECTOR INIT")
        self.lock = allocate_lock()
        self.inject_spikes = False
        self.event_sub = rospy.Subscriber('generator', Int32, self.event_callback) #, callback_args='/dvs_right/events')
        self.live_spikes_connection = SpynnakerLiveSpikesConnection(receive_labels = None, local_port = 19996,
                                                                    send_labels = ["spike_sender"])

        self.live_spikes_connection.add_start_resume_callback("spike_sender", self.start)

# Making sure LiveConnections is up and running
    def start(self, a, b):
        print("Spikeinjection ready\n\n\n")
# Callback when an input is received (other than -1 as -1 denotes no input)
    def event_callback(self, msg):
      	global num
        num = msg.data
# Sending spikes if an input is received from the ROS side
        if num != -1:
    	       self.live_spikes_connection.send_spike("spike_sender", num)

spike_injector = SpikeInjector()

def main(argv=None):
    rospy.init_node("spike_injector")
    rospy.spin()

if __name__ == "__main__":
    sys.exit(main())
