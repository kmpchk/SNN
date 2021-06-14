#include <iostream>
#include <chrono>
#include <vector>

#include "Network.h"

int main() {
    //std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(
    //        std::chrono::system_clock::now().time_since_epoch()).count() << std::endl;

    // Create Spiking Neural Network
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

    return 0;
}