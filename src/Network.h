#ifndef SNN_NETWORK_H
#define SNN_NETWORK_H

// C++
#include <unordered_map>
#include <set>
#include <sstream>

// ThirdParty
#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "csv.hpp"
#include "h5pp/h5pp.h"

// SNN
#include "Connection.h"
#include "Generator.h"
#include "NeuronGroup.h"

typedef std::normal_distribution<float> NormDist;
typedef std::uniform_int_distribution<> WeightDist;

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

        NeuronGroup *green_group = activators["vision_green"];
        NeuronGroup *red_group = activators["vision_red"];
        NeuronGroup *real_coin = activators["real_coin"];
        NeuronGroup *fake_coin = activators["fake_coin"];

        std::mt19937 rnd(std::random_device{}());
        std::uniform_int_distribution<> distrib(0, 1);

        for (size_t step = 0; step < sim_steps; step++)
        {
            //simulate_generator_connections();
            // Process vision part (camera)
            /*if (vision_generator.fired()) {
                vis_spikes_num++;
                spdlog::info("[Vision] Fired at Step: {0}", step);
            }*/
            // read vision ROS topic once
            // return value can be -1, 0, 1
            // group testing
            // 0 - red not detected, 1 - red detected

            int red_vision_ret = distrib(rnd);
            if (red_vision_ret == 1)
            {
                // red group spikes
                //simulate_activator(red_group);
            }
            else if (red_vision_ret == 0)
            {
                // red group does not spike
            }

            /*else if (vision_ret == -1)
            {
                // red group spikes
                simulate_activator(red_group);
            }
            else
            {
                // nothing
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

    void simulate_activator(NeuronGroup *activator)
    {
        std::size_t mean = activator->count / 4;
        std::size_t stddev = mean / 3;
        std::mt19937 rnd(std::random_device{}());
        auto norm_dist = NormDist(mean, stddev);

        int activated_neurons_count = static_cast<int>(std::round(norm_dist(rnd)));
        while (activated_neurons_count <= 0) {
            activated_neurons_count = static_cast<int>(std::round(norm_dist(rnd)));
        }

        std::vector<Neuron> act_nrns;
        std::sample(activator->neurons.begin(), activator->neurons.end(),
                    std::back_inserter(act_nrns),
                    activated_neurons_count,
                    std::mt19937{std::random_device{}()});

        for (Neuron &act_nrn : act_nrns) {
            act_nrn.set_fired(true);
            for (Connection &conn : act_nrn.post_conns) {
                conn.timers.push_back(2); // 2 ticks for transmitting signal
            }
        }
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
        auto weight_dist = WeightDist(WEIGHT_BOUNDARIES::LOW, WEIGHT_BOUNDARIES::HIGH);

        std::vector<std::uint32_t> ids(group2_neurons_count);
        std::iota(std::begin(ids), std::end(ids), 0);
        auto rng = std::default_random_engine {};

        for (std::size_t n_idx = 0; n_idx < group1_neurons_count; n_idx++) {
            int conn_num = static_cast<int>(std::round(conn_num_dist(rnd)));
            while (conn_num <= 0) {
                conn_num = static_cast<int>(std::round(conn_num_dist(rnd)));
            }
            // Get neuron-neighbors of current neuron
            std::vector<int> sample_group2_neuron_ids;
            std::shuffle(std::begin(ids), std::end(ids), rng);
            std::sample(ids.begin(), ids.end(),
                        std::back_inserter(sample_group2_neuron_ids),
                        conn_num,
                        std::mt19937{std::random_device{}()});
            spdlog::trace("===========================");
            spdlog::trace("[Current] Neuron = {0}", n_idx);
            for (const int &group2_neuron_idx : sample_group2_neuron_ids) {
                spdlog::trace("[Connect] Neuron = {0}", group2_neuron_idx);
                // Group1 post neurons append
                group1->neurons[n_idx].post_neurons.emplace_back(&group2->neurons[group2_neuron_idx]);
                // Group2 pre neurons append
                group2->neurons[group2_neuron_idx].pre_neurons.emplace_back(&group1->neurons[n_idx]);
                // Make connection b/w pre and post neurons
                Connection conn;
                conn.pre_neuron = &group1->neurons[n_idx];
                conn.post_neuron = &group2->neurons[group2_neuron_idx];
                conn.weight = weight_dist(rnd);
                // Group1 post neuron connection
                group1->neurons[n_idx].post_conns.emplace_back(conn);
                // Group2 pre neuron connection
                group2->neurons[group2_neuron_idx].pre_conns.emplace_back(&group1->neurons[n_idx].post_conns.back());
                spdlog::trace("[POST Conn] Weight = {0}", group1->neurons[n_idx].post_conns.back().weight);
                spdlog::trace("[PRE Conn] Weight = {0}", group2->neurons[group2_neuron_idx].pre_conns.back()->weight);
            }
            spdlog::trace("[Pre] Neuron size = {0}", group1->neurons[n_idx].post_neurons[0]->pre_neurons.size());
            spdlog::trace("===========================");
        }
    }

    void debug_info()
    {
        // TODO: implement func
    }
};

#endif //SNN_NETWORK_H
