#ifndef SNN_NETWORK_H
#define SNN_NETWORK_H

// C++
#include <unordered_map>
#include <set>
#include <sstream>

// ThirdParty
#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"

#include "Connection.h"
#include "Generator.h"
#include "NeuronGroup.h"

typedef std::normal_distribution<float> NormDist;

class Network {
private:
    std::vector<NeuronGroup> neuron_groups;
    std::unordered_map<std::string, NeuronGroup*> activators;

public:
    Network() = default;

    void run(std::uint32_t sim_steps = 100)
    {

        spdlog::trace("Activators count = {0}", activators.size());
        for (const auto& activator : activators) {
            spdlog::trace("Activator: {0}, Address = {1}", activator.first, fmt::ptr(activator.second));
        }

        // Vision generator
        int vis_spikes_num = 0;

        // Coin acceptor generator
        /*std::uint32_t ca_hw_frequency = 30;
        Generator ca_generator(ca_hw_frequency, sim_steps);
        int ca_spikes_num = 0;*/

        /*for (const auto& activator : activators) {
            //spdlog::info("Activator: {0}, size = {1}", activator.first, activator.second);
            std::cout << activator.first << " : " << activator.second << "\n";
        }*/

        for (size_t step = 0; step < sim_steps; step++)
        {
            //simulate_generator_connections();
            // Process vision part (camera)
            /*if (vision_generator.fired()) {
                vis_spikes_num++;
                spdlog::info("[Vision] Fired at Step: {0}", step);
            }*/

            // Process coin acceptor part
            /*if (ca_generator.fired()) {
                ca_spikes_num++;
                spdlog::info("[Coin Acceptor] Fired at Step: {0}", step);
            }*/
        }
        //spdlog::info("[Vision] Spikes: {0}", vis_spikes_num);
        //spdlog::info("[Coin Acceptor] Spikes: {0}", ca_spikes_num);
    }

    // Create neuron group
    NeuronGroup create_group(GroupOptions &group_opts)
    {
        // TODO: check group_opts fields
        if (group_opts.neurons_count <= 0)
            group_opts.neurons_count = 50; // default neurons count
        return NeuronGroup(group_opts);
    }

    void connect_groups(NeuronGroup *group1, NeuronGroup *group2)
    {
        // Find activator
        if (group1->group_type == GROUP_TYPE::ACTIVATOR) {
            if (!activators.count(group1->name))
                activators[group1->name] = group1;
        }

        //spdlog::info("[G1] Size = {0}", group1->neurons.size());
        //spdlog::info("[G2] Size = {0}", group2->neurons.size());
        std::size_t group1_neurons_count = group1->count;
        std::size_t group2_neurons_count = group2->count;
        std::size_t connection_num_mean = group2_neurons_count / 2;
        std::size_t connection_num_stddev = connection_num_mean / 2;

        std::mt19937 rnd(std::random_device{}());
        auto conn_num_dist = NormDist(connection_num_mean, connection_num_stddev);

        std::vector<std::uint32_t> ids(group2_neurons_count);
        std::iota(std::begin(ids), std::end(ids), 0);
        auto rng = std::default_random_engine {};

        for (std::size_t n_idx = 0; n_idx < group1_neurons_count; n_idx++) {
            int conn_num = static_cast<int>(std::round(conn_num_dist(rnd)));
            while (conn_num <= 0) {
                conn_num = static_cast<int>(std::round(conn_num_dist(rnd)));
            }
            // Make connections
            std::vector<int> sample_group2_neuron_ids;
            std::shuffle(std::begin(ids), std::end(ids), rng);
            std::sample(ids.begin(), ids.end(),
                        std::back_inserter(sample_group2_neuron_ids),
                        conn_num,
                        std::mt19937{std::random_device{}()});
            Neuron &current_neuron = group1->neurons[n_idx];
            spdlog::trace("===========================");
            spdlog::trace("[Post] Neuron = {0}", n_idx);
            for (const int &group2_neuron_idx : sample_group2_neuron_ids) {
                spdlog::trace("[Conn] Neuron = {0}", group2_neuron_idx);
                // Group1 post connection
                current_neuron.post_neurons.emplace_back(&group2->neurons[group2_neuron_idx]);
                current_neuron.post_conns.emplace_back(Connection());
                // Group2 pre connection
                group2->neurons[group2_neuron_idx].pre_neurons.emplace_back(&group1->neurons[n_idx]);
            }
            spdlog::trace("[Pre] Neuron = {0}", group1->neurons[n_idx].post_neurons[0]->pre_neurons.size());
            spdlog::trace("===========================");
        }
    }

    void debug_info()
    {
        // TODO: implement func
    }
};

#endif //SNN_NETWORK_H
