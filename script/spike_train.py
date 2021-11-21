#!/usr/bin/env python3

import h5py
import numpy as np
import matplotlib.pyplot as plt
import sys

filename = "../build/log.h5"


def plot_spike_train(h5f):
    sim_steps = np.int32(h5f["SIM_STEPS"])
    print(f"Sim steps: {sim_steps}")
    group_name = str(h5f["GROUP_NAME"])
    print(f"Group name: {group_name}")

    group_neurons_spike_train = h5f["SPIKE_TRAIN"]
    neurons_count = len(group_neurons_spike_train)
    red_neurons_spike_train = sorted(group_neurons_spike_train.items(), key=lambda x: int(x[0].split('_')[-1]))
    # print(red_neurons_spike_train.keys())
    # for item in red_neurons_spike_train:
    #    print(f"Dataset name: {item[0]}")
    #    print(np.asarray(item[1]))

    # a = np.array([0, 1])
    # b = np.array([5, 6])
    # c = np.multiply(a, b)
    # print(c)

    fig = plt.figure(figsize=(8, 8), dpi=100)

    ax = fig.add_subplot(111)

    plt.suptitle("RED VISION GROUP", fontsize=14)

    major_xticks = np.arange(0, sim_steps, 5)
    minor_xticks = np.arange(0, sim_steps, 1)

    major_yticks = np.arange(0, neurons_count, 2)
    minor_yticks = np.arange(0, neurons_count, 1)

    # ax.tick_params(axis='both', which='major', labelsize=6)
    ax.set_xticks(major_xticks)
    ax.set_xticks(minor_xticks, minor=True)

    ax.set_yticks(major_yticks)
    ax.set_yticks(minor_yticks, minor=True)

    # plt.tick_params(axis='both', which='major', pad=8)
    # plt.rcParams['axes.formatter.use_locale'] = True
    # plt.xticks(np.arange(0, sim_steps, 5.0), fontsize=9, rotation=60)
    # ax.set_yticks(np.arange(0, neurons_count, 1.0))
    ax.set_xlabel('Simulation steps (ms)', fontsize=16)
    ax.set_ylabel('Neuron number', fontsize=16)
    # plt.plot(1, 1, '-ro')

    for nrn_num in range(0, len(red_neurons_spike_train)):
        spiked_neuron_steps = np.nonzero(np.asarray(red_neurons_spike_train[nrn_num][1]))

        # print(spiked_neuron_steps)
        # if len(spiked_neuron_steps) == 0:
        #    print(f"{nrn_num} not spiked!")
        for sim_time in spiked_neuron_steps:
            plt.errorbar([sim_time], [nrn_num], yerr=[0.4], capsize=0, ls='none', color='black',
                         elinewidth=2)  # fmt='o'
            # plt.plot([sim_time], [nrn_num], 'go--')
            # plt.errorbar([sim_time], [nrn_num], [1.2], linestyle='None', marker='^')  #, linestyle='None', marker='^'

    # print(np.asarray(red_neurons_spike_train[0][1]))
    # spiked_neuron_steps = np.nonzero(np.asarray((red_neurons_spike_train[0][1])))
    # print(spiked_neuron_steps)

    # plt.plot(x, y)
    # plt.plot(z, t)
    plt.grid(b=True, which='major', color='k', alpha=0.5)
    plt.grid(b=True, which='minor', color='k', alpha=0.2)
    plt.show()

    # red_neurons_spike_train = h5f["SPIKE_TRAIN"]["RED"]["NEURONS_SPIKE_TRAIN"]
    # print(f"Neurons count: {len(red_neurons_spike_train)}")
    # print(np.asarray(red_neurons_spike_train[1]))

    # test1 = h5f["TEST1"]
    # print(len(test1))
    # print(test1)
    # for item in test1:
    #    print(item)

    # for item in red_neurons_spike_train[0]:
    #    print(item)
    # cont1_bitset = np.asarray(red_neurons_spike_train[0]).view(dtype=np.uint8)
    # print(type(cont1_bitset))
    # print(cont1_bitset.shape)
    # print(cont1)
    # cont1_bitset = np.binary_repr(cont1, width=sim_steps)
    # for pos in range(0, cont1_bitset.size):
    #    print(f"Bit pos {pos} = {cont1_bitset[pos]}")


def main(h5_filename):
    with h5py.File(h5_filename, "r") as h5f:
        # Plot SPIKE TRAIN of group
        plot_spike_train(h5f)


if __name__ == "__main__":  # for exit input q then ctrl + c
    h5_filename = sys.argv[1]
    main(h5_filename)
