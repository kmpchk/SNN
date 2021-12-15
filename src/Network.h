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

typedef std::normal_distribution<float> NormDist;
typedef std::uniform_int_distribution<> WeightDist, UniDist;

class Network {
private:
    std::unordered_map<std::string, NeuronGroup*> input_neuron_groups;
    std::unordered_map<std::string, NeuronGroup*> hidden_neuron_groups;
    std::unordered_map<std::string, NeuronGroup*> output_neuron_groups;

    ros::NodeHandle node_handle;
    ros::Subscriber red_vis_sub;
    boost::circular_buffer<double> red_vis_data{10};
    std::mutex red_vis_mutex;
    std::thread ros_spin;

    const std::string red_vision_group_topic = "/card_detect/red";
    const std::string green_vision_group_topic = "/card_detect/green";

    std::vector<std::vector<std::uint8_t>> red_neurons_spike_train;

    std::size_t cur_sim_step;

    NeuronGroup *red_group_ = nullptr, *serotonin_red_group_ = nullptr, *bad_mt_serotonin_group_ = nullptr;
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
        red_vis_sub = node_handle.subscribe(red_vision_group_topic, 10, &Network::red_vis_callback, this);

        ros_spin = std::thread([](){
            auto wait_time = ros::Duration(0.001);
            while (ros::ok() && !ros::isShuttingDown())
                ros::spinOnce();
            wait_time.sleep();
        });
    }

    void red_vis_callback(const std_msgs::Float32ConstPtr &message)
    {
        const std::lock_guard<std::mutex> lock(red_vis_mutex);
        red_vis_data.push_back(message->data);
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
        code_exec_time = (high_resolution_clock::now() - t1);
        spdlog::debug("[Initialization] Init exec time {0} (ms)", code_exec_time.count());

        //ros::Duration(0.5).sleep();

        spdlog::debug("Input groups count = {0}", input_neuron_groups.size());
        for (const auto& input_neuron_group : input_neuron_groups) {
            spdlog::debug("Input group: {0}, Address = {1}", input_neuron_group.first, fmt::ptr(input_neuron_group.second));
        }

        spdlog::debug("Hidden groups count = {0}", hidden_neuron_groups.size());
        for (const auto& hidden_neuron_group : hidden_neuron_groups) {
            spdlog::debug("Hidden group: {0}, Address = {1}", hidden_neuron_group.first, fmt::ptr(hidden_neuron_group.second));
        }

        spdlog::debug("Output groups count = {0}", output_neuron_groups.size());
        for (const auto& output_neuron_group : output_neuron_groups) {
            spdlog::debug("Output group: {0}, Address = {1}", output_neuron_group.first, fmt::ptr(output_neuron_group.second));
        }

        NeuronGroup *green_group = input_neuron_groups["vision_green"];

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

            const std::lock_guard<std::mutex> lock(red_vis_mutex);
            //spdlog::debug("RED VIS VAL (ROS): {0}", red_vis_data.back());

            // RED VISION
            int red_neurons_count_spiked = normalize_number(red_vis_data.back(), 0.0, 1.0, 0.0, red_group_->count);
            //simulate_input_group(red_group, red_neurons_count_spiked);
            //simulate_hidden_group(dopamine);
            //spdlog::debug("[Vision RED] spiked = {0}", red_neurons_count_spiked);

            // PROCESS INPUT LAYERS
            simulate_input_groups(red_neurons_count_spiked);

            // PROCESS HIDDEN LAYERS
            simulate_hidden_groups();

            // PROCESS OUTPUT LAYERS
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

        /*for (std::size_t idx = 0; idx < red_neurons_spike_train.size(); idx++) {
            int spiked_neurons_count = std::count_if(red_neurons_spike_train[idx].begin(),
                                                     red_neurons_spike_train[idx].end(),
                                                     [](bool spiked){return spiked == true;});
            spdlog::debug("Neuron {0} - Spiked {1} times", idx, spiked_neurons_count);
        }*/

        /*
        // Measuring execution time of code block
        duration<double, std::milli> code_exec_time;

        // Start measuring time
        auto t1 = high_resolution_clock::now();
        // Log RED vision group spike train
        auto h5_red_spike_time_logger = std::make_shared<h5pp::File>("h5_red_spike_time.h5");
        // Logging data
        h5_red_spike_time_logger->writeDataset("RED_VISION", "GROUP_NAME", H5D_COMPACT);
        h5_red_spike_time_logger->writeDataset(sim_steps, "SIM_STEPS", H5D_COMPACT);
        for (std::size_t idx = 0; idx < red_neurons_spike_train.size(); idx++) {
            std::string dataset_name = "SPIKE_TRAIN/NEURON_SPIKE_TRAIN_" + std::to_string(idx);
            h5_red_spike_time_logger->writeDataset(red_neurons_spike_train[idx], dataset_name, H5D_COMPACT);
        }
        auto t2 = high_resolution_clock::now();
        code_exec_time = (t2 - t1);
        spdlog::trace("[log] [SPIKE TRAIN] Group: {0}, Count: {1}, Func exec time (ms): {2}",
                      red_group->name, red_group->count, code_exec_time.count());
        // Stop measuring time
        */

        //spdlog::info("[Vision] Spikes: {0}", vis_spikes_num);
        //spdlog::info("[Coin Acceptor] Spikes: {0}", ca_spikes_num);
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
        for (const auto& neuron : red_random_neurons) {
            neuron.get().spike();
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

    /*void simulate_hidden_group(NeuronGroup *hidden_group)
    {
        // Process pre connections
        for (Neuron &neuron : hidden_group->neurons) {
            //spdlog::debug("Neuron {0}, Connections count: {1}", neuron.id, neuron.pre_conns.size());
            int weight_sum = 0;
            for (Connection *conn : neuron.pre_conns) {
                for(std::vector<int>::iterator timer_it = conn->timers.begin(); timer_it != conn->timers.end();) {
                    if (*timer_it == 0) {
                        timer_it = conn->timers.erase(timer_it);
                        spdlog::trace("[Spike] Neuron Pre {0} -> Neuron Post {1} via Conn {2} : Signal transmitted!!!",
                                      conn->pre_neuron->id, conn->post_neuron->id, conn->ID);
                        weight_sum += conn->weight;
                        //spdlog::trace("[Spike] spike on Neuron {1} !", conn->post_neuron->id);
                    }
                    else
                    {
                        ++timer_it;
                    }
                }
            }
            neuron.V = weight_sum;
            spdlog::trace("[Spike] Neuron {0} : MP = {1} : {2}", neuron.id, neuron.V, neuron.V > 60.0);
        }
    }*/

    void simulate_input_group(NeuronGroup *input_group, int neurons_count)
    {
        /*std::size_t mean = neurons_count;
        std::size_t stddev = mean / 4;
        static std::mt19937 rnd(std::random_device{}());
        auto norm_dist = NormDist(mean, stddev);

        int activated_neurons_count = static_cast<int>(std::round(norm_dist(rnd)));
        while (activated_neurons_count <= 0) {
            activated_neurons_count = static_cast<int>(std::round(norm_dist(rnd)));
        }*/

        // Update post connections
        /*for (Neuron &neuron : input_group->neurons) {
            for (Connection &conn : neuron.post_conns) {
                for(int &time : conn.timers) {
                    if (time > 0)
                        time--;
                    //spdlog::debug("CHECK = {0}", neuron.id == conn.pre_neuron->id);
                    spdlog::trace("[Updated] Neuron Pre {0} -> Neuron Post {1} : timer counts {2}",
                                  neuron.id, conn.post_neuron->id, conn.timers.size());
                }
            }
        }

        std::vector<std::reference_wrapper<Neuron>> random_neurons_number;
        std::sample(input_group->neurons.begin(), input_group->neurons.end(),
                    std::back_inserter(random_neurons_number),
                    neurons_count,
                    std::mt19937{std::random_device{}()});

        for (Neuron &act_nrn : random_neurons_number) {
            //act_nrn.set_fired(true);
            //red_neurons_spike_train[act_nrn.id][cur_sim_step] = true;
            for (Connection &conn : act_nrn.post_conns) {
                conn.timers.push_back(2); // 2 ticks for transmitting signal
                spdlog::trace("Neuron {0} - timer counts {1}", act_nrn.id, conn.timers.size());
            }
        }*/

        /*for (Neuron &act_nrn : random_neurons_number) {
            //spdlog::debug("Neuron {0} : Post conns count {1}", act_nrn.id, act_nrn.post_conns.size());
            for (Connection &conn : act_nrn.post_conns) {
                spdlog::debug("Neuron {0} : Post conns timers count {1}", act_nrn.id, conn.timers.size());
                for (int &timer : conn.timers) {
                    spdlog::debug("Neuron {0} : Timer values {1}", act_nrn.id, timer);
                }
            }
        }*/

        //std::exit(-1);


        /*spdlog::debug("Sim step: {0}, Total Neurons count: {1}, Neurons spiked count: {2}",
                      cur_sim_step, input_group->neurons.size(), act_nrns.size());*/


        /*std::vector<int> neuron_ids(input_group->neurons.size());
        std::iota(std::begin(neuron_ids), std::end(neuron_ids), 0);
        h5_logger->writeDataset(neuron_ids, "SPIKE_TRAIN/RED/NEURON_IDS");
        std::vector<bool> neurons_spike_train(input_group->neurons.size());*/

        /*for (Neuron &act_nrn : act_nrns) {
            act_nrn.set_fired(true);
            red_neurons_spike_train[act_nrn.id][cur_sim_step] = true;
            for (Connection &conn : act_nrn.post_conns) {
                conn.timers.push_back(2); // 2 ticks for transmitting signal
            }
        }*/
    }

    // Create neuron group
    NeuronGroup create_group(GroupOptions &group_opts)
    {
        // TODO: check group_opts fields
        if (group_opts.neurons_count <= 0)
            group_opts.neurons_count = 50; // default neurons count
        return NeuronGroup(group_opts);
    }

    void connect_out_groups(NeuronGroup *group1, NeuronGroup *group2)
    {
        // Add output neuron group #2
        if (group2->group_type == GroupType::OUTPUT) {
            if (!output_neuron_groups.count(group2->name))
                output_neuron_groups[group2->name] = group2;
        }

        for (int g2_n_idx = 0; g2_n_idx < 2; g2_n_idx++) {

            for (Neuron& neuron : group1->neurons) {
                group2->neurons[g2_n_idx].connect(neuron);
            }

            //spdlog::debug("[OUT INFO] IN = {0}", group2->neurons[g2_n_idx].getIn().size());
        }

    }

    void connect_groups(NeuronGroup *group1, NeuronGroup *group2)
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
        std::size_t connection_num_mean = group1->count / 4;
        std::size_t connection_num_stddev = connection_num_mean / 2;
        auto conn_num_dist = NormDist(connection_num_mean, connection_num_stddev);
        std::vector<int> ids(group1->count);
        std::iota(std::begin(ids), std::end(ids), 0);

        for (int g2_n_idx = 0; g2_n_idx < group2->count; g2_n_idx++) {
            int conn_num = static_cast<int>(std::round(conn_num_dist(rnd)));
            while (conn_num <= 0) {
                conn_num = static_cast<int>(std::round(conn_num_dist(rnd)));
            }

            spdlog::deb

            std::vector<int> sample_group1_neuron_ids;
            std::shuffle(std::begin(ids), std::end(ids), rnd);
            std::sample(ids.begin(), ids.end(),
                        std::back_inserter(sample_group1_neuron_ids),
                        conn_num,
                        rnd);

            for (const int &group1_neuron_idx : sample_group1_neuron_ids) {
                group2->neurons[g2_n_idx].connect(group1->neurons[group1_neuron_idx]);
            }

            /*for (int g2_n_idx = 0; g2_n_idx < group2->count; g2_n_idx++) {

                for (Neuron& neuron : group1->neurons) {
                    group2->neurons[g2_n_idx].connect(neuron);
                }

                //spdlog::debug("[OUT INFO] IN = {0}", group2->neurons[g2_n_idx].getIn().size());
            }*/

            //std::cout << group2->neurons[g2_n_idx] << std::endl;
        }

        //spdlog::debug("[split] Size = {0}", group1->neurons.size());

        //spdlog::info("[G1] Size = {0}", group1->neurons.size());
        //spdlog::info("[G2] Size = {0}", group2->neurons.size());
        /*std::size_t group1_neurons_count = group1->count;
        std::size_t group2_neurons_count = group2->count;
        std::size_t connection_num_mean = group2_neurons_count / 4;
        std::size_t connection_num_stddev = connection_num_mean / 2;

        std::mt19937 rnd(std::random_device{}());
        auto conn_num_dist = NormDist(connection_num_mean, connection_num_stddev);
        auto weight_dist = WeightDist(WEIGHT_BOUNDARIES::LOW, WEIGHT_BOUNDARIES::HIGH);

        std::vector<int> ids(group2_neurons_count);
        std::iota(std::begin(ids), std::end(ids), 0);
        auto rng = std::default_random_engine {};

        for (std::size_t n_idx = 0; n_idx < group1_neurons_count; n_idx++) {
            int conn_num = static_cast<int>(std::round(conn_num_dist(rnd)));
            while (conn_num <= 0) {
                conn_num = static_cast<int>(std::round(conn_num_dist(rnd)));
            }

            // Reserve memory space
            group1->neurons[n_idx].post_conns.reserve(conn_num * 10);
            group1->neurons[n_idx].post_neurons.reserve(conn_num * 10);

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
                group1->neurons[n_idx].post_neurons.push_back(&group2->neurons[group2_neuron_idx]);
                // Group2 pre neurons append
                group2->neurons[group2_neuron_idx].pre_neurons.push_back(&group1->neurons[n_idx]);
                // Make connection b/w pre and post neurons
                Connection conn;
                conn.pre_neuron = &group1->neurons[n_idx];
                conn.post_neuron = &group2->neurons[group2_neuron_idx];
                conn.weight = weight_dist(rnd);
                conn.ID = std::to_string(group1->neurons[n_idx].id) + "_" + std::to_string(group2->neurons[group2_neuron_idx].id);
                // Group1 post neuron connection
                group1->neurons[n_idx].post_conns.push_back(conn);
                group2->neurons[group2_neuron_idx].pre_conns.push_back(&group1->neurons[n_idx].post_conns[group1->neurons[n_idx].post_conns.size()-1]);
                //spdlog::trace("ADDRESS1: {:p}", fmt::ptr(&group1->neurons[n_idx].post_conns[group1->neurons[n_idx].post_conns.size()-1]));
                //spdlog::trace("ADDRESS2: {:p}", fmt::ptr(group2->neurons[group2_neuron_idx].pre_conns[group2->neurons[group2_neuron_idx].pre_conns.size()-1]));
                // Trace
                spdlog::trace("[POST Conn] Weight = {0}", group1->neurons[n_idx].post_conns[group1->neurons[n_idx].post_conns.size()-1].weight);
                spdlog::trace("[POST Conn] ADR = {:p}", fmt::ptr(&group1->neurons[n_idx].post_conns[group1->neurons[n_idx].post_conns.size()-1]));
                spdlog::trace("[PRE Conn] Weight = {0}", group2->neurons[group2_neuron_idx].pre_conns[group2->neurons[group2_neuron_idx].pre_conns.size()-1]->weight);
                spdlog::trace("[PRE Conn] ADR = {:p}", fmt::ptr(group2->neurons[group2_neuron_idx].pre_conns[group2->neurons[group2_neuron_idx].pre_conns.size()-1]));
            }
            //spdlog::trace("[Pre] Neuron size = {0}", group1->neurons[n_idx].post_neurons[0].get().pre_neurons.size());
            spdlog::trace("===========================");
        }

        auto uni_dist = UniDist(0, group2_neurons_count-1);
        for (std::size_t n_idx = 0; n_idx < group2_neurons_count; n_idx++) {
            if (!group2->neurons[n_idx].pre_conns.size()) {
                spdlog::trace("Neuron {0} has no connections!", n_idx);
                std::size_t random_idx = uni_dist(rnd);
                // Make connection b/w pre and post neurons
                Connection conn;
                conn.pre_neuron = &group1->neurons[random_idx];
                conn.post_neuron = &group2->neurons[n_idx];
                conn.weight = weight_dist(rnd);
                conn.ID = std::to_string(group1->neurons[random_idx].id) + "_" + std::to_string(group2->neurons[n_idx].id);
                // Group1 post neuron connection
                group1->neurons[random_idx].post_conns.push_back(conn);
                group2->neurons[n_idx].pre_conns.push_back(&group1->neurons[random_idx].post_conns[group1->neurons[random_idx].post_conns.size()-1]);
            }
        }*/
    }

    void debug_info()
    {
        // TODO: implement func
    }
};

#endif //SNN_NETWORK_H
