# Imports of both spynnaker and external device plugin.

import spynnaker8 as Frontend
from pyNN.utility.plotting import Figure, Panel

import matplotlib.pyplot as plt
import random
from threading import Condition
import time

import rospy
from std_msgs.msg import Int32

# Initial call to set up the front end (pynn requirement)
Frontend.setup(timestep=1.0, min_delay=1.0, max_delay=144.0)


# Synaptic weight and other global variables
weight_to_spike = 2.0
pub_num = 0
last_num = 0

# Neural parameters
cell_params_lif = {'cm': 0.25,
                   'i_offset': 0.0,
                   'tau_m': 20.0,
                   'tau_refrac': 2.0,
                   'tau_syn_E': 5.0,
                   'tau_syn_I': 5.0,
                   'v_reset': -70.0,
                   'v_rest': -65.0,
                   'v_thresh': -50.0
                   }

##################################
# Parameters for the injector population.  Note that each injector needs to
# be given a different port.  The virtual key is assigned here, rather than
# being allocated later.  As with the above, spikes injected need to match
# this key, and this will be done automatically with 16-bit keys.
##################################
cell_params_spike_injector_with_key = {

    # The port on which the spiNNaker machine should listen for packets.
    # Packets to be injected should be sent to this port on the spiNNaker
    # machine
    'port': 12346,

    # This is the base key to be used for the injection, which is used to
    # allow the keys to be routed around the spiNNaker machine.  This
    # assignment means that 32-bit keys must have the high-order 16-bit
    # set to 0x7; This will automatically be prepended to 16-bit keys.
    'virtual_key': 0x70000,
}

# Callback method for receiving input from ROS publisher
def callback(msg):
    global pub_num
    pub_num=msg.data
    print pub_num

# Initialising ROS node and subscriber
rospy.init_node('ros_side')
print "spinnaker_listener node initiated"
pub_num = rospy.Subscriber('generator', Int32, callback)

# Create neuron populations (if cur exp)
pop_forward = Frontend.Population(
    10, Frontend.IF_curr_exp(**cell_params_lif), label='pop_forward')

# Create injection populations
injector_forward = Frontend.Population(
    100, Frontend.external_devices.SpikeInjector(),
    label='spike_injector_forward',
    additional_parameters=cell_params_spike_injector_with_key)

# Create a connection from the injector into the populations
# The connection follows the pattern of 10 sequential neurons from injector
# population connects to one neuron of pop_forward sequentially

list0 = list()
for i in range (0,10):
    list0.append((i, 0, weight_to_spike, 3))
Frontend.Projection(injector_forward, pop_forward, Frontend.FromListConnector(list0))

list1 = list()
for i in range (10,20):
    list1.append((i, 1, weight_to_spike, 3))
Frontend.Projection(injector_forward, pop_forward, Frontend.FromListConnector(list1))

list2 = list()
for i in range (20,30):
    list2.append((i, 2, weight_to_spike, 3))
Frontend.Projection(injector_forward, pop_forward, Frontend.FromListConnector(list2))

list3 = list()
for i in range (30,40):
    list3.append((i, 3, weight_to_spike, 3))
Frontend.Projection(injector_forward, pop_forward, Frontend.FromListConnector(list3))

list4 = list()
for i in range (40,50):
    list4.append((i, 4, weight_to_spike, 3))
Frontend.Projection(injector_forward, pop_forward, Frontend.FromListConnector(list4))

list5 = list()
for i in range (50,60):
    list5.append((i, 5, weight_to_spike, 3))
Frontend.Projection(injector_forward, pop_forward, Frontend.FromListConnector(list5))

list6 = list()
for i in range (60,70):
    list6.append((i, 6, weight_to_spike, 3))
Frontend.Projection(injector_forward, pop_forward, Frontend.FromListConnector(list6))

list7 = list()
for i in range (70,80):
    list7.append((i, 7, weight_to_spike, 3))
Frontend.Projection(injector_forward, pop_forward, Frontend.FromListConnector(list7))

list8 = list()
for i in range (80,90):
    list8.append((i, 8, weight_to_spike, 3))
Frontend.Projection(injector_forward, pop_forward, Frontend.FromListConnector(list8))

list9 = list()
for i in range (90,100):
    list9.append((i, 9, weight_to_spike, 3))
Frontend.Projection(injector_forward, pop_forward, Frontend.FromListConnector(list9))

# Record spikes from the pop_forward population to be used after simulation is complete
pop_forward.record('spikes')

# Activate the sending of live spikes
Frontend.external_devices.activate_live_output_for(
    pop_forward, database_notify_host="localhost",
    database_notify_port_num=19996)

# Create a condition to avoid overlapping prints
print_condition = Condition()

# Create an initialisation method
def init_pop(label, n_neurons, run_time_ms, machine_timestep_ms):
    print("{} has {} neurons".format(label, n_neurons))
    print("Simulation will run for {}ms at {}ms timesteps".format(
        run_time_ms, machine_timestep_ms))

# Create a sender of packets for the forward population
def send_input_forward(label, sender):
    global last_num, pub_num
    while True:
        if last_num != pub_num:
            print "number caught in send_input_forward"
            print pub_num
            print "sending spike"
            sender.send_spike(label, pub_num, send_full_keys=True)
            last_num = pub_num

pub_spike = rospy.Publisher('snn_spiker', Int32, queue_size = 0)

# Create a receiver of live spikes
def receive_spikes(label, time, neuron_ids):
    for neuron_id in neuron_ids:
        print_condition.acquire()
        print("Received spike at time {} from {} - {}".format(
            time, label, neuron_id))
        print_condition.release()
        pub_spike.publish(neuron_id)
        print "spike published"

# Set up the live connection for sending spikes
live_spikes_connection_send = \
    Frontend.external_devices.SpynnakerLiveSpikesConnection(
        receive_labels=None, local_port=19999,
        send_labels=["spike_injector_forward"])

# Set up callbacks to occur at initialisation
live_spikes_connection_send.add_init_callback(
     "spike_injector_forward", init_pop)

# Set up callbacks to occur at the start of simulation
live_spikes_connection_send.add_start_callback(
    "spike_injector_forward", send_input_forward)

# Set up the live connection for receiving spikes
live_spikes_connection_receive = \
    Frontend.external_devices.SpynnakerLiveSpikesConnection(
        receive_labels=["pop_forward"],
        local_port=19996, send_labels=None)

# Set up callbacks to occur when spikes are received
live_spikes_connection_receive.add_receive_callback(
        "pop_forward", receive_spikes)

# Open ended simulation (running forever)
Frontend.external_devices.run_forever()
