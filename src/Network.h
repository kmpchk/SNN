#ifndef SNN_NETWORK_H
#define SNN_NETWORK_H

#include "Neuron.h"
#include <string>
#include <tuple>
#include <random>
#include <cstdio>

typedef NEURON_TYPE GROUP_TYPE;

class Network {

    typedef vector<Neuron> *NeuronGroup;
    typedef vector<size_t> *Connections;

private:
    vector<tuple<string, NeuronGroup>> neuron_groups;
    vector<tuple<string, string, Connections>> group_connections;

public:
    Network() = default;

    ~Network() {
        // Clear neuron_groups
        for (auto neuron_group : neuron_groups) {
            delete get<1>(neuron_group);
        }
        neuron_groups.clear();

        // Clear group_connections
        for (auto group_connection : group_connections) {
            delete get<2>(group_connection);
        }
        group_connections.clear();
    }

    // Print info
    void info() {
        // Common info
        for (auto neuron_group : neuron_groups) {
            cout << "\n-----------------\n";
            cout << "Group Name: " << get<0>(neuron_group) << endl;
            cout << "Neuron Count: " << get<1>(neuron_group)->size() << endl;
            cout << "-----------------\n";
        }

        // Connections
        for (auto group_connection : group_connections) {
            cout << "\n-----------------\n";
            printf("%s -> %s\n", get<0>(group_connection).c_str(), get<1>(group_connection).c_str());
            auto connections = get<2>(group_connection);
            for (size_t idx = 0; idx < (*connections).size(); idx++) {
                cout << "\t[in] = " << idx << " " << "[out] = " << (*connections)[idx] << endl;
            }
            cout << "-----------------\n";
        }
    }

    // Create neuron group
    void createGroup(std::string &group_name, GROUP_TYPE group_type, int64_t neuron_count) {
        NeuronGroup neuron_group = new vector<Neuron>(neuron_count, group_type);
        neuron_groups.emplace_back(group_name, neuron_group);
    }

    // Connect group
    void connectGroup(std::string &in_group_name, std::string &out_group_name) {
        //cout << "\n-----------------\n";
        // Get input group
        auto in_group = find_if(neuron_groups.begin(), neuron_groups.end(),
                                [&in_group_name](const tuple<string, NeuronGroup> &e) {
                                    return get<0>(e) == in_group_name;
                                });
        if (in_group == neuron_groups.end()) {
            cout << in_group_name << " not found!" << endl;
            return;
        }

        // Get output group
        auto out_group = find_if(neuron_groups.begin(), neuron_groups.end(),
                                 [&out_group_name](const tuple<string, NeuronGroup> &e) {
                                     return get<0>(e) == out_group_name;
                                 });
        if (out_group == neuron_groups.end()) {
            cout << out_group_name << " not found!" << endl;
            return;
        }

        // Extract vectors
        NeuronGroup input_neurons = std::get<1>(*in_group);
        NeuronGroup output_neurons = std::get<1>(*out_group);
        // Vectors must be the same length!
        if (input_neurons->size() != output_neurons->size()) {
            cout << "Vectors must be the same length!\n";
            return;
        }

        // Connecting neurons
        // Connection (DEBUG)
        Connections connections = new vector<size_t>();
        size_t neuron_count = input_neurons->size();
        // 1. Init random generator
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> distrib(0, neuron_count - 1);
        // 2. Connect groups
        for (size_t idx = 0; idx < neuron_count; idx++) {
            auto random_neuron_idx = distrib(gen);
            (*input_neurons)[idx].connect(&(*output_neurons)[random_neuron_idx]);
            //cout << "[Input group name]: " << in_group_name << endl;
            //cout << "[Output group name]: " << out_group_name << endl;
            //cout << "[Connection]: Input = " << idx << " " << "Output = " << random_neuron_idx << endl;
            connections->push_back(random_neuron_idx);
        }
        group_connections.emplace_back(in_group_name, out_group_name, connections);
        //cout << "-----------------\n";
    }

};

#endif //SNN_NETWORK_H
