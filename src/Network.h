#ifndef SNN_NETWORK_H
#define SNN_NETWORK_H

// C++
#include <unordered_map>
#include <set>
#include <sstream>
#include <bitset>
#include <functional>
#include <string>
#include <mutex>
#include <thread>
#include <random>

// ThirdParty
#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "h5pp/h5pp.h"

// ROS
#include "ros/ros.h"
#include "std_msgs/Float32.h"

// Boost
#include <boost/circular_buffer.hpp>

// SNN
//#include "Connection.h"
//#include "Generator.h"
#include "NeuronGroup.h"
#include "Drawer.h"

typedef std::normal_distribution<float> NormDist;
typedef std::uniform_int_distribution<> WeightDist, UniDist;

class Network {
private:
    std::unordered_map<std::string, NeuronGroup*> input_neuron_groups;
    std::unordered_map<std::string, NeuronGroup*> hidden_neuron_groups;
    std::unordered_map<std::string, NeuronGroup*> output_neuron_groups;

    ros::NodeHandle node_handle;
    ros::Subscriber red_vis_sub, green_vis_sub;
    boost::circular_buffer<double> red_vis_data{10};
    boost::circular_buffer<double> green_vis_data{10};
    std::mutex red_vis_mutex, green_vis_mutex;
    std::thread ros_spin;

    const std::string red_vision_group_topic = "/card_detect/red";
    const std::string green_vision_group_topic = "/card_detect/green";

    std::vector<std::vector<std::uint8_t>> red_neurons_spike_train;

    std::size_t cur_sim_step;

    NeuronGroup *red_group_ = nullptr, *serotonin_red_group_ = nullptr, *bad_mt_serotonin_group_ = nullptr,
                *green_group_ = nullptr, *dopamine_green_group_ = nullptr, *good_mt_dopamine_group_ = nullptr;
    std::unordered_map<int, int> output_neurons_spiked;

public:
    Network() = default;

    ~Network()
    {
        spdlog::debug("Simulation finished!");
        spdlog::debug("Press Ctrl + C to Exit");
        ros_spin.join();
    }

    Network(ros::NodeHandle& node_handle): node_handle(node_handle)
    {
        // Red vision subscriber
        red_vis_sub = node_handle.subscribe<std_msgs::Float32>(red_vision_group_topic, 10,
                                            [&](const std_msgs::Float32ConstPtr &message){
            const std::lock_guard<std::mutex> lock(red_vis_mutex);
            red_vis_data.push_back(message->data);
        });

        // Green vision subscriber
        green_vis_sub = node_handle.subscribe<std_msgs::Float32>(green_vision_group_topic, 10,
                                                               [&](const std_msgs::Float32ConstPtr &message){
           const std::lock_guard<std::mutex> lock(green_vis_mutex);
           green_vis_data.push_back(message->data);
       });

        ros_spin = std::thread([](){
            auto wait_time = ros::Duration(0.001);
            while (ros::ok() && !ros::isShuttingDown()) {
                ros::spinOnce();
                wait_time.sleep();
            }
        });
    }

    void run(std::uint32_t sim_steps = 100)
    {

        // Wait for incoming messages
        spdlog::debug("Initialization...");
        duration<double, std::milli> code_exec_time;
        auto t1 = high_resolution_clock::now();
        // Before we continue, there must be at least one message in buffers
        while (red_vis_data.empty()) {
            //spdlog::debug("No incoming message. Waiting...");
            ros::Duration(0.001).sleep();
        }
        while (green_vis_data.empty()) {
            //spdlog::debug("No incoming message. Waiting...");
            ros::Duration(0.001).sleep();
        }
        code_exec_time = (high_resolution_clock::now() - t1);
        spdlog::debug("[Initialization] Init exec time {0} (ms)", code_exec_time.count());

        //ros::Duration(0.5).sleep();

        spdlog::trace("Input groups count = {0}", input_neuron_groups.size());
        for (const auto& input_neuron_group : input_neuron_groups) {
            spdlog::trace("Input group: {0}, Address = {1}", input_neuron_group.first, fmt::ptr(input_neuron_group.second));
        }

        spdlog::trace("Hidden groups count = {0}", hidden_neuron_groups.size());
        for (const auto& hidden_neuron_group : hidden_neuron_groups) {
            spdlog::trace("Hidden group: {0}, Address = {1}", hidden_neuron_group.first, fmt::ptr(hidden_neuron_group.second));
        }

        spdlog::trace("Output groups count = {0}", output_neuron_groups.size());
        for (const auto& output_neuron_group : output_neuron_groups) {
            spdlog::trace("Output group: {0}, Address = {1}", output_neuron_group.first, fmt::ptr(output_neuron_group.second));
        }

        green_group_ = input_neuron_groups["vision_green"];
        dopamine_green_group_ = hidden_neuron_groups["dopamine_green_vision"];
        good_mt_dopamine_group_ = output_neuron_groups["good_mt_dopamine"];

        red_group_ = input_neuron_groups["vision_red"];
        serotonin_red_group_ = hidden_neuron_groups["serotonin_red_vision"];
        bad_mt_serotonin_group_ = output_neuron_groups["bad_mt_serotonin"];
        //red_neurons_spike_train.resize(red_group->neurons.size(), std::vector<std::uint8_t>(sim_steps, false));

        //NeuronGroup *real_coin = input_neuron_groups["real_coin"];

        //NeuronGroup *fake_coin = input_neuron_groups["fake_coin"];

        //NeuronGroup *dopamine = hidden_neuron_groups["dopamine"];

        /*std::mt19937 rnd(std::random_device{}());
        std::uniform_int_distribution<> distrib(0, 1);*/

        // The amount of time to wait before returning if no message is received
        auto wait_time = ros::Duration(0.01);

        for (size_t step = 0; step < 1000; step++)
        {
            cur_sim_step = step;

            spdlog::debug("CURRENT SIM STEP: {0}", cur_sim_step);

            const std::lock_guard<std::mutex> lock_red(red_vis_mutex);
            const std::lock_guard<std::mutex> lock_green(green_vis_mutex);
            //spdlog::debug("RED VIS VAL (ROS): {0}", red_vis_data.back());

            // RED VISION
            int red_neurons_count_spiked = normalize_number(red_vis_data.back(), 0.0, 1.0, 0.0, red_group_->count);
            int green_neurons_count_spiked = normalize_number(green_vis_data.back(), 0.0, 1.0, 0.0, green_group_->count);

            // PROCESS INPUT LAYER
            simulate_input_groups(red_neurons_count_spiked);

            // PROCESS HIDDEN LAYER
            simulate_hidden_groups();

            // PROCESS OUTPUT LAYER
            simulate_out_groups();


            spdlog::debug("Progress {:.1f}%", step * (100.0 / sim_steps));

            // Workaround
            wait_time.sleep();
        }

        // MOTOR CORTEX
        // 1. Calculate OUT groups
        /*for (auto item : output_neurons_spiked)
        {
            spdlog::debug("SIM STEP = {0}, SPIKED OUT = {1}", item.first, item.second);
        }*/
    }

    void simulate_input_groups(int red_cnt, int green_cnt=0, int real_cnt=0, int fake_cnt=0)
    {
        // RED
        std::vector<std::reference_wrapper<Neuron>> red_random_neurons;
        std::sample(red_group_->neurons.begin(), red_group_->neurons.end(),
                    std::back_inserter(red_random_neurons),
                    red_cnt,
                    std::mt19937{std::random_device{}()});
        // RED layer spikes
        int spiked = 0;
        for (Neuron &neuron : red_random_neurons) {
            neuron.spike();
        }
        for (Neuron& neuron : red_group_->neurons) {
            auto state = neuron.check(false);
            if (state == State::ActionStart)
                spiked++;
        }
        spdlog::debug("[INPUT SIMULATION] TIME = {0}, SPIKED = {1}", cur_sim_step, spiked);
    }

    void simulate_hidden_groups()
    {
        // RED -> SER layer checks
        int spiked = 0;
        for (Neuron& neuron : serotonin_red_group_->neurons) {
            auto state = neuron.check(true);
            //spdlog::debug("[HIDDEN SIMULATION] STATE = {0}", state);
            if (state == State::ActionStart)
                spiked++;
        }
        spdlog::debug("[HIDDEN SIMULATION] TIME = {0}, SPIKED = {1}", cur_sim_step, spiked);
    }

    void simulate_out_groups()
    {
        int spiked = 0;
        for (Neuron& neuron : bad_mt_serotonin_group_->neurons) {
            auto state = neuron.check(true);
            //spdlog::debug("[OUT] NEURON = {0}, STATE = {1}", neuron.getId(), state);
            if (state == State::ActionStart)
                spiked++;
        }
        spdlog::debug("[OUT SIMULATION] TIME = {0}, SPIKED = {1}", cur_sim_step, spiked);
    }

    // Create neuron group
    NeuronGroup create_group(GroupOptions &group_opts, Drawer &darwer)
    {
        // TODO: check group_opts fields
        if (group_opts.neurons_count <= 0)
            group_opts.neurons_count = 50; // default neurons count
        return NeuronGroup(group_opts, darwer);
    }

    void connect_groups(NeuronGroup *group1, NeuronGroup *group2, Drawer *darwer)
    {
        
        // Check groups
        assert(group1->group_type != GroupType::OUTPUT);
        assert(group2->group_type != GroupType::INPUT);

        // Add input neuron group #1
        if (group1->group_type == GroupType::INPUT) {
            if (!input_neuron_groups.count(group1->name))
                input_neuron_groups[group1->name] = group1;
        }
        // Add hidden neuron group #1
        if (group1->group_type == GroupType::HIDDEN) {
            if (!hidden_neuron_groups.count(group1->name))
                hidden_neuron_groups[group1->name] = group1;
        }
        // Add input neuron group #2
        if (group2->group_type == GroupType::INPUT) {
            if (!input_neuron_groups.count(group2->name))
                input_neuron_groups[group2->name] = group2;
        }
        // Add hidden neuron group #2
        if (group2->group_type == GroupType::HIDDEN) {
            if (!hidden_neuron_groups.count(group2->name))
                hidden_neuron_groups[group2->name] = group2;
        }
        // Add output neuron group #2
        if (group2->group_type == GroupType::OUTPUT) {
            if (!output_neuron_groups.count(group2->name))
                output_neuron_groups[group2->name] = group2;
        }

        std::mt19937 rnd(std::random_device{}());
        std::size_t connection_num_mean = group1->count / 3;
        std::size_t connection_num_stddev = 0;
        auto conn_num_dist = NormDist(connection_num_mean, connection_num_stddev);
        std::vector<int> ids(group1->count);
        std::iota(std::begin(ids), std::end(ids), 0);

        for (int g2_n_idx = 0; g2_n_idx < group2->count; g2_n_idx++) {
            int conn_num = static_cast<int>(std::round(conn_num_dist(rnd)));
            while (conn_num <= 0) {
                conn_num = static_cast<int>(std::round(conn_num_dist(rnd)));
            }

            std::vector<int> sample_group1_neuron_ids;
            std::shuffle(std::begin(ids), std::end(ids), rnd);
            std::sample(ids.begin(), ids.end(),
                        std::back_inserter(sample_group1_neuron_ids),
                        conn_num,
                        rnd);

            for (const int &group1_neuron_idx : sample_group1_neuron_ids) {
                group2->neurons[g2_n_idx].connect(group1->neurons[group1_neuron_idx]);
                darwer->connect_nodes(group1->name + std::to_string(group1_neuron_idx), group2->name + std::to_string(g2_n_idx));
            }
        }
    }
};

#endif //SNN_NETWORK_H
