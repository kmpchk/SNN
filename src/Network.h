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
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"

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
    ros::Subscriber red_vis_sub, green_vis_sub, coin_acceptor_sub;
    ros::Publisher mt_pub;
    boost::circular_buffer<double> red_vis_data{10};
    boost::circular_buffer<double> green_vis_data{10};
    boost::circular_buffer<int> coin_acceptor_data{10};
    std::mutex red_vis_mutex, green_vis_mutex, coin_acceptor_mutex;
    std::thread ros_spin;

    const std::string red_vision_group_topic = "/card_detect/red";
    const std::string green_vision_group_topic = "/card_detect/green";
    const std::string coin_acceptor_topic = "/coin_acceptor";
    const std::string mt_pattern_topic = "/mt_pattern";
    enum COIN_TYPE {REAL=1, FAKE=0};
    enum BEHAVIOR {HAPPY=1, ANGRY=0};

    std::vector<std::vector<std::uint8_t>> red_neurons_spike_train;

    std::size_t cur_sim_step;

    NeuronGroup *red_group_ = nullptr, *serotonin_red_group_ = nullptr, *bad_mt_serotonin_group_ = nullptr,
                *green_group_ = nullptr, *dopamine_green_group_ = nullptr, *good_mt_dopamine_group_ = nullptr,
                *fake_coin_group_ = nullptr, *real_coin_group_ = nullptr, *serotonin_fake_coin_group_ = nullptr,
                *dopamine_real_coin_group_ = nullptr,
                *dopamine_group_ = nullptr, *serotonin_group_ = nullptr;
    std::unordered_map<int, int> output_neurons_spiked;
    std::unordered_map<int, int> bad_states, good_states;

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
            //const std::lock_guard<std::mutex> lock(red_vis_mutex);
            red_vis_data.push_back(message->data);
        });

        // Green vision subscriber
        green_vis_sub = node_handle.subscribe<std_msgs::Float32>(green_vision_group_topic, 10,
                                                               [&](const std_msgs::Float32ConstPtr &message){
           //const std::lock_guard<std::mutex> lock(green_vis_mutex);
           green_vis_data.push_back(message->data);
        });

        // Coin acceptor subscriber
        coin_acceptor_sub = node_handle.subscribe<std_msgs::Int8>(coin_acceptor_topic, 10,
                                                               [&](const std_msgs::Int8ConstPtr &message){
          //const std::lock_guard<std::mutex> lock(coin_acceptor_mutex);
          coin_acceptor_data.push_back(message->data);
        });

        // Motor Cortex
        mt_pub = node_handle.advertise<std_msgs::String>(mt_pattern_topic, 10);

        ros_spin = std::thread([](){
            auto wait_time = ros::Duration(0.0001);
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
        while (coin_acceptor_data.empty()) {
            //spdlog::debug("No incoming message. Waiting...");
            ros::Duration(0.001).sleep();
        }
        code_exec_time = (high_resolution_clock::now() - t1);
        spdlog::debug("[Initialization] Init took {0} (ms)", code_exec_time.count());

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

        // Input groups
        green_group_ = input_neuron_groups["vision_green"];
        assert (green_group_ != nullptr);
        red_group_ = input_neuron_groups["vision_red"];
        assert (red_group_ != nullptr);
        fake_coin_group_ = input_neuron_groups["fake_coin"];
        assert (fake_coin_group_ != nullptr);
        real_coin_group_ = input_neuron_groups["real_coin"];
        assert (real_coin_group_ != nullptr);

        // Hidden groups
        dopamine_group_ = hidden_neuron_groups["dopamine"];
        assert (dopamine_group_ != nullptr);
        //dopamine_real_coin_group_ = hidden_neuron_groups["dopamine_real_coin"];
        //assert (dopamine_real_coin_group_ != nullptr);
        serotonin_group_ = hidden_neuron_groups["serotonin"];
        assert (serotonin_group_ != nullptr);
        //serotonin_fake_coin_group_ = hidden_neuron_groups["serotonin_fake_coin"];
        //assert (serotonin_fake_coin_group_ != nullptr);

        // Output groups
        good_mt_dopamine_group_ = output_neuron_groups["good_mt_dopamine"];
        assert (good_mt_dopamine_group_ != nullptr);
        bad_mt_serotonin_group_ = output_neuron_groups["bad_mt_serotonin"];
        assert (bad_mt_serotonin_group_ != nullptr);

        // The amount of time to wait before returning if no message is received
        auto wait_time = ros::Duration(0.001);

        for (size_t step = 0; step < sim_steps; step++)
        {
            cur_sim_step = step;

            spdlog::debug("CURRENT SIM STEP: {0}", cur_sim_step);

            //const std::lock_guard<std::mutex> lock_red(red_vis_mutex);
            //const std::lock_guard<std::mutex> lock_green(green_vis_mutex);
            //const std::lock_guard<std::mutex> lock_coin_acpt(coin_acceptor_mutex);
            //spdlog::debug("RED VIS VAL (ROS): {0}", red_vis_data.back());

            int red_neurons_count_spiked = normalize_number(red_vis_data.back(), 0.0, 1.0, 0.0, red_group_->count);
            // Workaround
            if (red_neurons_count_spiked)
                red_neurons_count_spiked += 50;

            int green_neurons_count_spiked = normalize_number(green_vis_data.back(), 0.0, 1.0, 0.0, green_group_->count);
            // Workaround
            if (green_neurons_count_spiked)
                green_neurons_count_spiked += 50;

            int coin = coin_acceptor_data.back();
            COIN_TYPE ct;
            ct = coin ? COIN_TYPE::REAL : COIN_TYPE::FAKE;

            // PROCESS INPUT LAYER
            simulate_input_groups(red_neurons_count_spiked, green_neurons_count_spiked, ct);

            // PROCESS HIDDEN LAYER
            simulate_hidden_groups();

            // PROCESS OUTPUT LAYER
            simulate_out_groups();


            spdlog::debug("Progress {:.1f}%", step * (100.0 / sim_steps));

            // Workaround
            wait_time.sleep();
        }

        // MOTOR CORTEX
        int good_spikes = 0;
        for (auto item : good_states)
        {
            if (item.second)
                good_spikes++;
        }
        //spdlog::debug("GOOD STATES = {0}", good_spikes);

        int bad_spikes = 0;
        for (auto item : bad_states)
        {
            if (item.second)
                bad_spikes++;
        }
        //spdlog::debug("BAD STATES = {0}", bad_spikes);

        BEHAVIOR bh = good_spikes > bad_spikes ? BEHAVIOR::HAPPY : BEHAVIOR::ANGRY;
        std::string bh_str = bh ? "HAPPY DOG" : "ANGRY DOG";
        spdlog::debug("BEHAVIOR = {0}", bh_str);
        mt_pub.publish(bh);

        // 1. Calculate OUT groups
        /*for (auto item : output_neurons_spiked)
        {
            spdlog::debug("SIM STEP = {0}, SPIKED OUT = {1}", item.first, item.second);
        }*/
    }

    void simulate_input_groups(int red_cnt, int green_cnt, COIN_TYPE ct)
    {
        // Common params
        int spiked = 0;
        std::vector<std::reference_wrapper<Neuron>> random_neurons;
        static auto rg = std::mt19937{std::random_device{}()};

        /// RED LAYER
        // 1. Create container
        std::sample(red_group_->neurons.begin(), red_group_->neurons.end(),
                    std::back_inserter(random_neurons),
                    red_cnt,
                    rg);
        // 2. Spike
        spiked = 0;
        for (Neuron &neuron : random_neurons) {
            neuron.spike();
        }
        // 3. Check
        for (Neuron& neuron : red_group_->neurons) {
            auto state = neuron.check(false);
            if (state == State::ActionStart)
                spiked++;
        }
        spdlog::debug("[INPUT SIMULATION] [RED] TIME = {0}, SPIKED = {1}", cur_sim_step, spiked);

        /// GREEN LAYER
        // 1. Create container
        random_neurons.clear();
        std::sample(green_group_->neurons.begin(), green_group_->neurons.end(),
                    std::back_inserter(random_neurons),
                    green_cnt,
                    rg);
        // 2. Spike
        spiked = 0;
        for (Neuron &neuron : random_neurons) {
            neuron.spike();
        }
        // 3. Check
        for (Neuron& neuron : green_group_->neurons) {
            auto state = neuron.check(false);
            if (state == State::ActionStart)
                spiked++;
        }
        spdlog::debug("[INPUT SIMULATION] [GREEN] TIME = {0}, SPIKED = {1}", cur_sim_step, spiked);

        /// COIN LAYER
        if (ct == COIN_TYPE::REAL) {
            // 1. Spike all
            spiked = 0;
            for (Neuron &neuron: real_coin_group_->neurons) {
                neuron.spike();
            }
            // 2. Check
            for (Neuron &neuron: real_coin_group_->neurons) {
                auto state = neuron.check(false);
                if (state == State::ActionStart)
                    spiked++;
            }
            spdlog::debug("[INPUT SIMULATION] [REAL COIN] TIME = {0}, SPIKED = {1}", cur_sim_step, spiked);
        } else {
            // 1. Spike all
            spiked = 0;
            for (Neuron &neuron: fake_coin_group_->neurons) {
                neuron.spike();
            }
            // 2. Check
            for (Neuron &neuron: fake_coin_group_->neurons) {
                auto state = neuron.check(false);
                if (state == State::ActionStart)
                    spiked++;
            }
            spdlog::debug("[INPUT SIMULATION] [FAKE COIN] TIME = {0}, SPIKED = {1}", cur_sim_step, spiked);
        }
    }

    void simulate_hidden_groups()
    {
        // Common params
        int spiked = 0;

        // RED -> SER layer checks
        spiked = 0;
        for (Neuron& neuron : serotonin_group_->neurons) {
            auto state = neuron.check(true);
            //spdlog::debug("[HIDDEN SIMULATION] STATE = {0}", state);
            if (state == State::ActionStart)
                spiked++;
        }
        spdlog::debug("[HIDDEN SIMULATION] [SEROTONIN] TIME = {0}, SPIKED = {1}", cur_sim_step, spiked);
        bad_states[cur_sim_step] = spiked;


        spiked = 0;
        for (Neuron& neuron : dopamine_group_->neurons) {
            auto state = neuron.check(true);
            //spdlog::debug("[HIDDEN SIMULATION] STATE = {0}", state);
            if (state == State::ActionStart)
                spiked++;
        }
        spdlog::debug("[HIDDEN SIMULATION] [DOPAMINE] TIME = {0}, SPIKED = {1}", cur_sim_step, spiked);
        good_states[cur_sim_step] = spiked;

    }

    void simulate_out_groups()
    {
        // Common params
        int spiked = 0;

        // BAD Motor Cortex
        for (Neuron& neuron : bad_mt_serotonin_group_->neurons) {
            auto state = neuron.check(true);
            //spdlog::debug("[OUT] NEURON = {0}, STATE = {1}", neuron.getId(), state);
            if (state == State::ActionStart)
                spiked++;
        }
        spdlog::debug("[OUT SIMULATION] [BAD Motor Cortex] TIME = {0}, SPIKED = {1}", cur_sim_step, spiked);
        //output_neurons_spiked[cur_sim_step] = spiked;
        //bad_states[cur_sim_step] += spiked;

        // GOOD Motor Cortex
        spiked = 0;
        for (Neuron& neuron : good_mt_dopamine_group_->neurons) {
            auto state = neuron.check(true);
            //spdlog::debug("[OUT] NEURON = {0}, STATE = {1}", neuron.getId(), state);
            if (state == State::ActionStart)
                spiked++;
        }
        spdlog::debug("[OUT SIMULATION] [GOOD Motor Cortex] TIME = {0}, SPIKED = {1}", cur_sim_step, spiked);
        //good_states[cur_sim_step] += spiked;
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

        if (group1->group_type == GroupType::HIDDEN && group2->group_type == GroupType::OUTPUT) {
            for (int g2_n_idx = 0; g2_n_idx < group2->count; g2_n_idx++) {
                for (int g1_n_idx = 0; g1_n_idx < group1->count; g1_n_idx++) {
                    group2->neurons[g2_n_idx].connect(group1->neurons[g1_n_idx]);
                }
            }
            return;
        }

        std::mt19937 rnd(std::random_device{}());
        std::size_t connection_num_mean = group1->count / 2;
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
