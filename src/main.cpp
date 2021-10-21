#include <iostream>
#include <chrono>
#include <vector>
#include <random>
#include "spdlog/spdlog.h"

#include "Network.h"

// Boost::Graph
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>
#include <boost/graph/graph_utility.hpp>

// Neuron
struct Vertex
{
    int threshold;
};

// Synapse
struct Edge
{
    int speed;
    std::string channel;
};

int main() {
    //std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(
    //        std::chrono::system_clock::now().time_since_epoch()).count() << std::endl;

    // Create Spiking Neural Network
	/*
    Network snn_network;

    // Create Input Group #1
    std::string group1_name("Hearing");
    int64_t group1_neuron_count(10);
    GROUP_TYPE group1_type = GROUP_TYPE::Excitatory;
    snn_network.createGroup(group1_name, group1_type, group1_neuron_count);

    // Create Input Group #2
    std::string group2_name("Vision");
    int64_t group2_neuron_count(10);
    GROUP_TYPE group2_type = GROUP_TYPE::Inhibitory;
    snn_network.createGroup(group2_name, group2_type, group2_neuron_count);

    // Create Output Group #3
    std::string group3_name("Output");
    int64_t group3_neuron_count(10);
    GROUP_TYPE group3_type = GROUP_TYPE::Excitatory;
    snn_network.createGroup(group3_name, group3_type, group3_neuron_count);

    // Connect 1 -> 3
    snn_network.connectGroup(group1_name, group3_name);

    // Connect 2 -> 3
    snn_network.connectGroup(group2_name, group3_name);

    // Print info
    snn_network.info();
    */

	spdlog::info("SNN in action!");

    Network snn_net;

    std::uint32_t sim_steps = 100;

    // Vision generator
    std::uint32_t vision_hw_frequency = 50;
    snn_net.create_generator("vision", vision_hw_frequency, sim_steps);

	// TODO: return group created
    snn_net.create_group("dopamine", 50);
    //snn_net.create_group("serotonine", 10);

	// TODO: return value
    snn_net.connect_generator("vision", "dopamine");
    //snn_net.connect_generator("vision", "serotonine");

    //snn_net.show_debug_info();

    snn_net.run(sim_steps);

    return 0;
}