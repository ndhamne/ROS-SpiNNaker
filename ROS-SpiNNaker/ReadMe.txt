This project is a proof of concept of ROS and Spiking Neural Network working in tandem with PyNN as frontend and SpiNN-5 board as back-end.

The pub.py file is the ROS node publisher which publishes random numbers between 0-99 and the spinn_sript.py is the SpiNNaker script to be run on the PyNN-SpiNNkar setup. Both files need to be run simultaneously. (Although running spinn_script before publishing number is highly recommended because initialisation process of the board takes some time and in the meanwhile the data coming form ROS side should not be lost).

The connectivity between injecting neurons and the spiking neurons is 10 injecting neurons are connected to one spiking neuron, both sequentially based on their neuron_ids.

This project is merely sending a random number for ROS to SpiNNaker and replying the id of the neuron which spiked on the SpiNNaker board. (SpiNN-5 board is used here, but would work on other boards as well)
