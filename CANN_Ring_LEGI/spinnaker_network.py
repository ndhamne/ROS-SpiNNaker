#!/usr/bin/env python

import spynnaker8 as Frontend
import spynnaker8.external_devices as ExternalDevices

import rospy
import rospkg
from std_msgs.msg import *

import numpy as np
import os
import traceback

from pyNN.utility.plotting import Figure, Panel
import matplotlib.pyplot as plt

class SpinnakerNetwork(object):
    def __init__(self, n_neurons):
# Initializing values
        self.n_neurons = n_neurons

        self.output_spikes_pub = rospy.Publisher('/output_spikes', Int32, queue_size=10)
# Lists for populations and projections
        self.projections = {}
        self.populations = {}
# Matrices and Connector list
        self.dist = np.zeros((n_neurons,n_neurons))
        self.weights = np.zeros((n_neurons,n_neurons))
        self.cann_connector = list()
# Constants (working for the THIS setup )
        self.timestep = 1.
# Synaptic weights between neurons of different populations
        self.weight_to_spike = 6 #6 #1.5 #1.1
        #2.#1.7#2.15#2.2#2.1#2.25#2.5#3.#4.#2.25#1.57#1.565#1.6#1.6#1.55#=1.62#1.75#1.5#1.#2.#1.5#2.25#2.5#3.#
        self.weight_inhibitory = 0.55 #0.375
        self.weight_input2cann = 3.
        self.weight_cann2inh = 1 #1 #0.9 #1
# Delays
        self.delay_input2cann = 1
        self.delay_cann2inh = 1
        self.delay_inh2cann = 0
        self.delay_cann2cann = 0
# Normal distribution for LEGI implementation
# LEGI : Local Excitation Global Inhibition
        self.mu = 0.
        self.sig = 1 #0.75#1.

# Setting up the hardware
    def setup(self):

        Frontend.setup(timestep=self.timestep, min_delay=1.0, max_delay=144.0)

# Defining network (populations and projections) with connectivity
    def make_inference_network(self, weights=None):


        self.populations['spike_sender'] = Frontend.Population(self.n_neurons,
                                            ExternalDevices.SpikeInjector(),
                                            additional_parameters={'port': 12345},
                                            label="spike_sender")

        self.populations['cann_pop'] = Frontend.Population(self.n_neurons, Frontend.IF_curr_exp(),#**neuron_params),
            label="cann_pop")

        self.populations['inh_pop'] = Frontend.Population(1, Frontend.IF_curr_exp(),#**neuron_params),
            label="inh_pop")

        Frontend.Projection(self.populations['spike_sender'], self.populations['cann_pop'],
                            Frontend.OneToOneConnector(),
                            Frontend.StaticSynapse(weight=self.weight_input2cann,delay= self.delay_input2cann))

        self.projections['cann2inh'] = Frontend.Projection(
            self.populations['cann_pop'],
            self.populations['inh_pop'],
            Frontend.AllToAllConnector(),
            synapse_type=Frontend.StaticSynapse(weight=self.weight_cann2inh, delay = self.delay_cann2inh),
            receptor_type="excitatory")

        self.projections['inh2cann'] = Frontend.Projection(
            self.populations['inh_pop'],
            self.populations['cann_pop'],
            Frontend.AllToAllConnector(),
            synapse_type=Frontend.StaticSynapse(weight=self.weight_inhibitory, delay = self.delay_inh2cann),
            receptor_type="inhibitory")

# LEGI logic for a ring network
        for i in range(self.n_neurons):
            for j in range(self.n_neurons):
                self.dist[i][j] = abs(i - j)%(self.n_neurons)
                if self.dist[i][j] > self.n_neurons/2:
                    self.dist[i][j] = self.n_neurons - self.dist[i][j]
                self.weights[i][j] = round(self.weight_to_spike * \
                    (1 / (self.sig * np.sqrt(2 * np.pi)) * \
                    (np.exp(-np.power(self.dist[i][j] - self.mu, 2.) \
                    / (2 * np.power(self.sig, 2.))))), 2)
                self.cann_connector.append((i, j, self.weights[i][j], self.delay_cann2cann))
        # print "Distance matrix:\n", self.dist
        print "Weight matrix:\n", self.weights
        # print "CANN connector:\n",self.cann_connector

        self.projections['cann2cann'] = Frontend.Projection(
            self.populations['cann_pop'],
            self.populations['cann_pop'],
            Frontend.FromListConnector(self.cann_connector),
            receptor_type="excitatory")

        ExternalDevices.activate_live_output_for(self.populations['cann_pop'], database_notify_host = "localhost",
                                                 database_notify_port_num = 19996)

        ExternalDevices.activate_live_output_for(self.populations['inh_pop'], database_notify_host = "localhost",
                                                 database_notify_port_num = 19998)

# Recording the values from populations during simulation
        self.populations['cann_pop'].record('spikes')
        self.populations['inh_pop'].record('spikes')
        self.populations['cann_pop'].record('v')
        self.populations['inh_pop'].record('v')
        self.populations['spike_sender'].record('spikes')

def main(argv=None):
    run_time = 10000
    rospy.init_node("offline_spinnaker")
    np.random.seed(12345)

    spinn_network = SpinnakerNetwork(n_neurons=20)

    spinn_network.setup()
    spinn_network.make_inference_network()

    rospy.loginfo("SpiNNaker network ready: starting to run")
    print "running"

    Frontend.run(run_time)
# Getting specific data from the recordings
    input_spikes = spinn_network.populations['spike_sender'].get_data('spikes')

    cann_spikes = spinn_network.populations['cann_pop'].get_data('spikes')
    cann_spikes_count = spinn_network.populations['cann_pop'].get_spike_counts(gather=True)
    cann_v = spinn_network.populations['cann_pop'].get_data('v')

    inhib_spikes = spinn_network.populations['inh_pop'].get_data('spikes')
    inhib_spikes_count = spinn_network.populations['inh_pop'].get_spike_counts(gather=True)
    inhib_v = spinn_network.populations['inh_pop'].get_data('v')

    print "Spikes per neuron in cann_net: ", cann_spikes_count
    winner = max(cann_spikes_count, key = cann_spikes_count.get)
    print "The most active neuron is NeuronID:", winner
    print "spikes in inhib_pop\n\n", inhib_spikes_count

    Frontend.end()
# Plotting the data
    Figure(
        Panel(input_spikes.segments[0].spiketrains, xticks = True,
              yticks=True, markersize=.2, xlim=(0, run_time)),
        Panel(cann_spikes.segments[0].spiketrains, xticks = True,
              yticks=True, markersize=.2, xlim=(0, run_time)),
        Panel(inhib_spikes.segments[0].spiketrains, xticks = True,
              yticks=True, markersize=.2, xlim=(0, run_time)),
        Panel(cann_v.segments[0].filter(name='v')[0],
            ylabel="Memb pot (mV) for Cann", xticks = True,
            yticks=True, xlim=(0, run_time)),
        annotations="Simulated with {}".format(Frontend.name())
    )
    plt.show()

if __name__ == "__main__":
    main()
