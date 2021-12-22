#include <chrono>

#include "Utils.h"
#include "Network.h"
#include "Type.h"
#include "Neuron.h"
#include "Drawer.h"

int main(int argc, char **argv) {

    spdlog::info("SNN in action!");

    // Init ROS node
    ros::init(argc, argv, "SNN");
    ros::NodeHandle node_handle;

    // Init logger
    init_logger(spdlog::level::debug, spdlog::level::trace);

    // Create Spiking Neural Network
    Network snn_net(node_handle);

    // Simulation steps
    std::uint32_t sim_steps = 10000;

    // Measuring execution time of func
    duration<double, std::milli> func_exec_time;

    Drawer drawer = Drawer(100.f, 40.f, 40.f, 2.f, 20);
    Drawer drawer_top = Drawer(26.f, 10.f, 2000.f, 12.f, 10);

	// Vision red group
    auto t1 = high_resolution_clock::now();
    GroupOptions vision_red_group_opts;
    vision_red_group_opts.name = "vision_red";
    vision_red_group_opts.neurons_count = 150;
    vision_red_group_opts.group_type = GroupType::INPUT;
    vision_red_group_opts.neurons_type = NeuronType::Excitatory;
    NeuronGroup vision_red_group = snn_net.create_group(vision_red_group_opts, drawer_top);
    auto t2 = high_resolution_clock::now();
    func_exec_time = (t2 - t1);
    spdlog::trace("[create_group] Group: {0}, Count: {1}, Func exec time (ms): {2}", vision_red_group.name, vision_red_group.count, func_exec_time.count());
    drawer.add_node(1, vision_red_group_opts.name);
    //vision_red_group.debug_info();

    // Vision green group
    t1 = high_resolution_clock::now();
    GroupOptions vision_green_group_opts;
    vision_green_group_opts.name = "vision_green";
    vision_green_group_opts.neurons_count = 150;
    vision_green_group_opts.group_type = GroupType::INPUT;
    vision_green_group_opts.neurons_type = NeuronType::Excitatory;
    NeuronGroup vision_green_group = snn_net.create_group(vision_green_group_opts, drawer_top);
    t2 = high_resolution_clock::now();
    func_exec_time = (t2 - t1);
    spdlog::trace("[create_group] Group: {0}, Count: {1}, Func exec time (ms): {2}", vision_green_group.name, vision_green_group.count, func_exec_time.count());
    drawer.add_node(1, vision_green_group_opts.name);
    //vision_green_group.debug_info();

    // Fake coin group
    t1 = high_resolution_clock::now();
    GroupOptions fake_coin_group_opts;
    fake_coin_group_opts.name = "fake_coin";
    fake_coin_group_opts.neurons_count = 150;
    fake_coin_group_opts.group_type = GroupType::INPUT;
    fake_coin_group_opts.neurons_type = NeuronType::Excitatory;
    NeuronGroup fake_coin_group = snn_net.create_group(fake_coin_group_opts, drawer_top);
    t2 = high_resolution_clock::now();
    func_exec_time = (t2 - t1);
    drawer.add_node(1, fake_coin_group_opts.name);
    spdlog::trace("[create_group] Group: {0}, Count: {1}, Func exec time (ms): {2}", fake_coin_group.name, fake_coin_group.count, func_exec_time.count());
    //spdlog::info("[main] Size = {0}", fake_coin_group.neurons.size());
    //fake_coin_group.debug_info();

    // Real coin group
    t1 = high_resolution_clock::now();
    GroupOptions real_coin_group_opts;
    real_coin_group_opts.name = "real_coin";
    real_coin_group_opts.neurons_count = 150;
    real_coin_group_opts.group_type = GroupType::INPUT;
    real_coin_group_opts.neurons_type = NeuronType::Excitatory;
    NeuronGroup real_coin_group = snn_net.create_group(real_coin_group_opts, drawer_top);
    t2 = high_resolution_clock::now();
    func_exec_time = (t2 - t1);
    drawer.add_node(1, real_coin_group_opts.name);
    spdlog::trace("[create_group] Group: {0}, Count: {1}, Func exec time (ms): {2}", real_coin_group.name, real_coin_group.count, func_exec_time.count());
    //real_coin_group.debug_info();

    // 5-HT group for red vision
    t1 = high_resolution_clock::now();
    GroupOptions serotonin_group_opts;
    serotonin_group_opts.name = "serotonin";
    serotonin_group_opts.neurons_count = 250;
    serotonin_group_opts.group_type = GroupType::HIDDEN;
    serotonin_group_opts.neurons_type = NeuronType::Inhibitory;
    NeuronGroup serotonin_group = snn_net.create_group(serotonin_group_opts, drawer_top);
    t2 = high_resolution_clock::now();
    func_exec_time = (t2 - t1);
    drawer.add_node(2, serotonin_group_opts.name);
    //spdlog::trace("[create_group] Group: {0}, Count: {1}, Func exec time (ms): {2}", serotonin_red_group.name, serotonin_red_group.count, func_exec_time.count());
    //serotonin_group.debug_info();

    // 5-HT group for fake coin
    /*
    t1 = high_resolution_clock::now();
    serotonin_group_opts.name = "serotonin_fake_coin";
    serotonin_group_opts.neurons_count = 150;
    serotonin_group_opts.group_type = GroupType::HIDDEN;
    serotonin_group_opts.neurons_type = NeuronType::Excitatory;
    NeuronGroup serotonin_fake_group = snn_net.create_group(serotonin_group_opts, drawer_top);
    t2 = high_resolution_clock::now();
    func_exec_time = (t2 - t1);
    drawer.add_node(2, serotonin_group_opts.name);
    spdlog::trace("[create_group] Group: {0}, Count: {1}, Func exec time (ms): {2}", serotonin_fake_group.name, serotonin_fake_group.count, func_exec_time.count());
    */
    //serotonin_group.debug_info();

    // Dopamine group for green vision
    t1 = high_resolution_clock::now();
    GroupOptions dopamine_group_opts;
    dopamine_group_opts.name = "dopamine";
    dopamine_group_opts.neurons_count = 250;
    dopamine_group_opts.group_type = GroupType::HIDDEN;
    dopamine_group_opts.neurons_type = NeuronType::Excitatory;
    NeuronGroup dopamine_group = snn_net.create_group(dopamine_group_opts, drawer_top);
    t2 = high_resolution_clock::now();
    func_exec_time = (t2 - t1);
    drawer.add_node(2, dopamine_group_opts.name);
    //spdlog::trace("[create_group] Group: {0}, Count: {1}, Func exec time (ms): {2}", dopamine_green_group.name, dopamine_green_group.count, func_exec_time.count());
    //dopamine_group.debug_info();

    // Dopamine group for real coin
    /*t1 = high_resolution_clock::now();
    dopamine_group_opts.name = "dopamine_real_coin";
    dopamine_group_opts.neurons_count = 150;
    dopamine_group_opts.group_type = GroupType::HIDDEN;
    dopamine_group_opts.neurons_type = NeuronType::Excitatory;
    NeuronGroup dopamine_real_coin_group = snn_net.create_group(dopamine_group_opts, drawer_top);
    t2 = high_resolution_clock::now();
    func_exec_time = (t2 - t1);
    drawer.add_node(2, dopamine_group_opts.name);
    spdlog::trace("[create_group] Group: {0}, Count: {1}, Func exec time (ms): {2}", dopamine_real_coin_group.name, dopamine_real_coin_group.count, func_exec_time.count());*/
    //dopamine_group.debug_info();

    // Good Motor Cortex (MT) for Dopamine (DA)
    t1 = high_resolution_clock::now();
    GroupOptions mt_dopamine_group_opts;
    mt_dopamine_group_opts.name = "good_mt_dopamine";
    mt_dopamine_group_opts.neurons_count = 50;
    mt_dopamine_group_opts.group_type = GroupType::OUTPUT;
    mt_dopamine_group_opts.neurons_type = NeuronType::Excitatory;
    NeuronGroup good_mt_dopamine_group = snn_net.create_group(mt_dopamine_group_opts, drawer_top);
    t2 = high_resolution_clock::now();
    func_exec_time = (t2 - t1);
    drawer.add_node(3, mt_dopamine_group_opts.name);
    spdlog::trace("[create_group] Group: {0}, Count: {1}, Func exec time (ms): {2}", good_mt_dopamine_group.name, good_mt_dopamine_group.count, func_exec_time.count());
    //dopamine_group.debug_info();

    // Bad Motor Cortex (MT) for Serotonin (5-HT)
    t1 = high_resolution_clock::now();
    mt_dopamine_group_opts.name = "bad_mt_serotonin";
    mt_dopamine_group_opts.neurons_count = 50;
    mt_dopamine_group_opts.group_type = GroupType::OUTPUT;
    mt_dopamine_group_opts.neurons_type = NeuronType::Excitatory;
    NeuronGroup bad_mt_serotonin_group = snn_net.create_group(mt_dopamine_group_opts, drawer_top);
    t2 = high_resolution_clock::now();
    func_exec_time = (t2 - t1);
    drawer.add_node(3, mt_dopamine_group_opts.name);
    //drawer.connect_nodes(vision_red_group.name, serotonin_group.name);
    spdlog::trace("[create_group] Group: {0}, Count: {1}, Func exec time (ms): {2}", bad_mt_serotonin_group.name, bad_mt_serotonin_group.count, func_exec_time.count());
    //dopamine_group.debug_info();

    /// VISION
    drawer.init_texture();
    drawer_top.init_texture();

    // Vision Red -> Serotonin
    t1 = high_resolution_clock::now();
    snn_net.connect_groups(&vision_red_group, &serotonin_group, &drawer_top);
    t2 = high_resolution_clock::now();
    func_exec_time = (t2 - t1);
    drawer.connect_nodes(vision_red_group.name, serotonin_group.name);
    spdlog::trace("[connect_groups] {0}({1}) -> {2}({3}), Func exec time (ms): {4}",
                  vision_red_group.name, vision_red_group.count,
                  serotonin_group.name, serotonin_group.count,
                  func_exec_time.count());

    // Vision Green -> Dopamine
    t1 = high_resolution_clock::now();
    snn_net.connect_groups(&vision_green_group, &dopamine_group, &drawer_top);
    t2 = high_resolution_clock::now();
    func_exec_time = (t2 - t1);
    drawer.connect_nodes(vision_green_group.name, dopamine_group.name);
    spdlog::trace("[connect_groups] {0}({1}) -> {2}({3}), Func exec time (ms): {4}",
                  vision_green_group.name, vision_green_group.count,
                  dopamine_group.name, dopamine_group.count,
                  func_exec_time.count());

    /// COIN ACCEPTOR

    // Fake Coin -> Serotonin
    t1 = high_resolution_clock::now();
    snn_net.connect_groups(&fake_coin_group, &serotonin_group, &drawer_top);
    t2 = high_resolution_clock::now();
    func_exec_time = (t2 - t1);
    drawer.connect_nodes(fake_coin_group.name, serotonin_group.name);
    spdlog::trace("[connect_groups] {0}({1}) -> {2}({3}), Func exec time (ms): {4}",
                  fake_coin_group.name, fake_coin_group.count,
                  serotonin_group.name, serotonin_group.count,
                  func_exec_time.count());

    // Real Coin -> Dopamine
    t1 = high_resolution_clock::now();
    snn_net.connect_groups(&real_coin_group, &dopamine_group, &drawer_top);
    t2 = high_resolution_clock::now();
    func_exec_time = (t2 - t1);
    drawer.connect_nodes(real_coin_group.name, dopamine_group.name);
    spdlog::trace("[connect_groups] {0}({1}) -> {2}({3}), Func exec time (ms): {4}",
                  real_coin_group.name, real_coin_group.count,
                  dopamine_group.name, dopamine_group.count,
                  func_exec_time.count());


    /// Motor Cortex
    // serotonin_group -> bad_mt_serotonin_group
    t1 = high_resolution_clock::now();
    snn_net.connect_groups(&serotonin_group, &bad_mt_serotonin_group, &drawer_top);
    t2 = high_resolution_clock::now();
    func_exec_time = (t2 - t1);
    drawer.connect_nodes(serotonin_group.name, bad_mt_serotonin_group.name);
    spdlog::trace("[connect_groups] {0}({1}) -> {2}({3}), Func exec time (ms): {4}",
                  serotonin_group.name, serotonin_group.count,
                  bad_mt_serotonin_group.name, bad_mt_serotonin_group.count,
                  func_exec_time.count());

    // dopamine_group -> good_mt_dopamine_group
    t1 = high_resolution_clock::now();
    snn_net.connect_groups(&dopamine_group, &good_mt_dopamine_group, &drawer_top);
    t2 = high_resolution_clock::now();
    func_exec_time = (t2 - t1);
    drawer.connect_nodes(dopamine_group.name, good_mt_dopamine_group.name);
    spdlog::trace("[connect_groups] {0}({1}) -> {2}({3}), Func exec time (ms): {4}",
                  dopamine_group.name, dopamine_group.count,
                  good_mt_dopamine_group.name, good_mt_dopamine_group.count,
                  func_exec_time.count());

    // INHIBITORY
    /*t1 = high_resolution_clock::now();
    snn_net.connect_groups(&dopamine_group, &serotonin_group, &drawer_top);
    t2 = high_resolution_clock::now();
    func_exec_time = (t2 - t1);
    drawer.connect_nodes(dopamine_group.name, serotonin_group.name);
    spdlog::trace("[connect_groups] {0}({1}) -> {2}({3}), Func exec time (ms): {4}",
                  dopamine_group.name, dopamine_group.count,
                  serotonin_group.name, serotonin_group.count,
                  func_exec_time.count());

    t1 = high_resolution_clock::now();
    snn_net.connect_groups(&serotonin_group, &dopamine_group, &drawer_top);
    t2 = high_resolution_clock::now();
    func_exec_time = (t2 - t1);
    drawer.connect_nodes(serotonin_group.name, dopamine_group.name);
    spdlog::trace("[connect_groups] {0}({1}) -> {2}({3}), Func exec time (ms): {4}",
                  dopamine_group.name, dopamine_group.count,
                  serotonin_group.name, serotonin_group.count,
                  func_exec_time.count());*/


    // TODO: SPIKE TIME
    // TODO: RTF
    // TODO: 10 sec sim time to real time

	// TODO: return value
    //snn_net.connect_generator("vision", "dopamine");
    //snn_net.connect_generator("vision", "serotonine");

    //snn_net.debug_info();

    /*for (Neuron &neuron : serotonin_red_group.neurons) {
        spdlog::debug("Neuron = {0}, In conn num = {1}", neuron.getId(), neuron.getIn().size());
    }*/
    drawer.draw();
    drawer_top.draw();
    drawer.save("architect.jpeg");
    drawer_top.save("topology.jpeg");
    drawer.save("architect.png");
    drawer_top.save("topology.png");

    snn_net.run(sim_steps);

    return 0;
}