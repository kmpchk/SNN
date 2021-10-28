#include <iostream>
#include <chrono>
#include <vector>
#include <random>
#include "spdlog/spdlog.h"

#include "Network.h"

int main() {
    //std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(
    //std::chrono::system_clock::now().time_since_epoch()).count() << std::endl;

	spdlog::info("SNN in action!");

    // Create Spiking Neural Network
    Network snn_net;

    std::uint32_t sim_steps = 100;

    // Vision generator
    //std::uint32_t vision_hw_frequency = 50;
    //snn_net.create_generator("vision", vision_hw_frequency, sim_steps);

	// Vision red group
    GroupOptions vision_red_group_opts;
    vision_red_group_opts.name = "vision_red";
    vision_red_group_opts.neurons_count = 50;
    vision_red_group_opts.group_type = GROUP_TYPE::ACTIVATOR;
    NeuronGroup vision_red_group = snn_net.create_group(vision_red_group_opts);
    //vision_red_group.debug_info();
    //vision_red_group->name = "f1f";

    // Vision green group
    GroupOptions vision_green_group_opts;
    vision_green_group_opts.name = "vision_green";
    vision_green_group_opts.neurons_count = 50;
    vision_green_group_opts.group_type = GROUP_TYPE::ACTIVATOR;
    NeuronGroup vision_green_group = snn_net.create_group(vision_green_group_opts);
    //vision_green_group.debug_info();

    // Fake coin group
    GroupOptions fake_coin_group_opts;
    fake_coin_group_opts.name = "fake_coin";
    fake_coin_group_opts.neurons_count = 50;
    fake_coin_group_opts.group_type = GROUP_TYPE::ACTIVATOR;
    NeuronGroup fake_coin_group = snn_net.create_group(fake_coin_group_opts);
    //spdlog::info("[main] Size = {0}", fake_coin_group.neurons.size());
    //fake_coin_group.debug_info();

    // Real coin group
    GroupOptions real_coin_group_opts;
    real_coin_group_opts.name = "real_coin";
    real_coin_group_opts.neurons_count = 50;
    real_coin_group_opts.group_type = GROUP_TYPE::ACTIVATOR;
    NeuronGroup real_coin_group = snn_net.create_group(real_coin_group_opts);
    //real_coin_group.debug_info();

    // 5-HT group
    GroupOptions serotonin_group_opts;
    serotonin_group_opts.name = "serotonin";
    serotonin_group_opts.neurons_count = 50;
    serotonin_group_opts.group_type = GROUP_TYPE::USUAL_NEURON;
    NeuronGroup serotonin_group = snn_net.create_group(serotonin_group_opts);
    //serotonin_group.debug_info();

    // Dopamine group
    GroupOptions dopamine_group_opts;
    dopamine_group_opts.name = "dopamine";
    dopamine_group_opts.neurons_count = 50;
    dopamine_group_opts.group_type = GROUP_TYPE::USUAL_NEURON;
    NeuronGroup dopamine_group = snn_net.create_group(dopamine_group_opts);
    dopamine_group.name = "f1f";
    //dopamine_group.debug_info();

    // Fake Coin -> Dopamine
    snn_net.connect_groups(&vision_red_group, &dopamine_group);



    //std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(
    //        std::chrono::system_clock::now().time_since_epoch()).count() << std::endl;

    // TODO: SPIKE TIME
    // TODO: RTF
    // TODO: 10 sec sim time to real time

	// TODO: return value
    //snn_net.connect_generator("vision", "dopamine");
    //snn_net.connect_generator("vision", "serotonine");

    //snn_net.debug_info();

    snn_net.run(sim_steps);

    return 0;
}