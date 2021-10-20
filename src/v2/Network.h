#ifndef SNN_NETWORK_H
#define SNN_NETWORK_H

#include "Connection.h"
#include "Neuron.h"
#include "Generator.h"

// C++
#include <unordered_map>
#include <set>
#include <sstream>

// ThirdParty
#include "spdlog/spdlog.h"

/*
 * INDEX --- GROUP NAME
 *   0   ---  SER (RED CAMERA)
 *   1   ---  SER (GREEN CAMERA)
 *   2   ---  DA ()
 */

typedef std::vector<Neuron> NeuronGroup;
typedef std::vector<Connection> ConnectionGroup;
typedef std::vector<std::uint32_t> GeneratorNeuronIds;

typedef std::normal_distribution<float> NormDist;

class Network {
private:
    std::vector<std::string> group_names;
    std::vector<NeuronGroup> groups;
    std::unordered_map<std::string, Generator> generators;
    std::vector<ConnectionGroup> group_connections;
    std::vector<std::uint32_t> generator_group_ids;
    std::vector<GeneratorNeuronIds> generator_neurons; // Mapping b/w GroupId and NeuronIds

    // Process intra-group connections
    void connect_intra_group(std::size_t g_idx)
    {
        // TODO: check idx
        NeuronGroup &neuron_group = groups[g_idx];
        std::size_t neurons_count = neuron_group.size();
        std::size_t connection_num_mean = neurons_count / 2;
        std::size_t connection_num_stddev = connection_num_mean / 2;
        std::mt19937 rnd(std::random_device{}());
        auto conn_num_dist = NormDist(connection_num_mean, connection_num_stddev);
        // Fill with 0, 1, ..., conn_num
        std::vector<std::uint32_t> ids(neurons_count);
        std::iota(std::begin(ids), std::end(ids), 0);

        std::vector<std::vector<std::uint32_t>> connectivity_matrix;
        connectivity_matrix.resize(neurons_count);

        std::vector<std::uint32_t> processed_neurons_ids;

        for (std::size_t n_idx = 0; n_idx < neurons_count; n_idx++) {
            int conn_num = static_cast<int>(std::round(conn_num_dist(rnd)));
            //spdlog::info("[connect_intra_group] Connection count = {0}", conn_num);
            //group_connections.push_back(Connection())
            // generate a sample of target node ids

            // Remove element by index
            ids.erase(ids.begin() + n_idx);

            // Out connections
            std::vector<std::uint32_t> sample;
            std::sample(ids.begin(), ids.end(),
                        std::back_inserter(sample),
                        conn_num,
                        std::mt19937{std::random_device{}()});

            // Check connectivity

            /*for (std::size_t idx = 0; idx < processed_neurons_ids.size(); idx++) {
                if (connectivity_matrix[idx].size()) {
                    auto conns = connectivity_matrix[idx];

                }
            }*/

            /*for (std::size_t idx = 0; idx < processed_neurons_ids.size(); idx++) {

            }*/

            if (neuron_group[n_idx].generator_connected) {
                // process main neuron
                neuron_group[n_idx].post_neurons_idx = sample;
                for (std::uint32_t &idx : sample) {
                    neuron_group[idx].pre_neurons_idx.push_back(n_idx);
                }
            }
            else {
                // process not main neuron
                // check duplicates
                std::vector<std::uint32_t> sample_unique;
                std::set_difference(sample.begin(), sample.end(),
                                    neuron_group[n_idx].pre_neurons_idx.begin(),
                                    neuron_group[n_idx].pre_neurons_idx.end(),
                                    std::back_inserter(sample_unique));

                if (!sample_unique.size()) {
                    sample_unique.clear();
                    std::set_difference(ids.begin(), ids.end(),
                                        processed_neurons_ids.begin(),
                                        processed_neurons_ids.end(),
                                        std::back_inserter(sample_unique));
                }
                // connect post
                neuron_group[n_idx].post_neurons_idx = sample_unique;
                for (std::uint32_t &idx : sample_unique) {
                    neuron_group[idx].pre_neurons_idx.push_back(n_idx);
                }

                // connect pre
                if (!neuron_group[n_idx].pre_neurons_idx.size()) {
                    std::vector<std::uint32_t> out;
                    std::sample(
                            ids.begin(),
                            ids.end(),
                            std::back_inserter(out),
                            1,
                            std::mt19937{std::random_device{}()}
                    );
                    auto rand_idx = out[0];
                    neuron_group[n_idx].pre_neurons_idx.push_back(rand_idx);
                    neuron_group[rand_idx].post_neurons_idx.push_back(n_idx);
                }
            }

            // Restore element
            ids.push_back(n_idx);

            processed_neurons_ids.push_back(n_idx);

            // TODO: Implement GENERATOR_TO_GROUP connection
            //if(n_idx != 0)


            /*spdlog::info("Neuron {0}", n_idx);
            for (auto item : sample) {
                spdlog::info("Neuron {0}", n_idx);
            }*/

            /*auto neighbors_samples = NeuronGroup();
            neighbors_samples.reserve(conn_num);
            std::sample(std::cbegin(neuron_group), std::cend(neuron_group),
                        std::back_inserter(neighbors_samples), conn_num, rnd);
            for (Neuron &neighbor : neighbors_samples) {
                //group_connections.push_back(Connection(&neighbor, &neuron_group[n_idx]));
            }*/
        }

        spdlog::info("=====================");
        spdlog::info("Neuron Group ID: {0}", g_idx);
        spdlog::info("----------------------");
        for (std::size_t n_idx = 0; n_idx < neurons_count; n_idx++) {
            spdlog::info("Neuron ID: {0}", n_idx);
            spdlog::info("GenConn Neuron: {0}", neuron_group[n_idx].generator_connected);
            if (neuron_group[n_idx].generator_connected)
            {
                std::stringstream post_neurons_ss;
                for (std::size_t i = 0; i < neuron_group[n_idx].post_neurons_idx.size(); ++i) {
                    if (i != 0)
                        post_neurons_ss << ",";
                    post_neurons_ss << neuron_group[n_idx].post_neurons_idx[i];
                }
                std::string post_neurons_str = post_neurons_ss.str();
                spdlog::info("Post neurons: {0}", post_neurons_str);
            }
            else
            {
                std::stringstream pre_neurons_ss;
                std::stringstream post_neurons_ss;
                for (std::size_t i = 0; i < neuron_group[n_idx].post_neurons_idx.size(); ++i) {
                    if (i != 0)
                        post_neurons_ss << ",";
                    post_neurons_ss << neuron_group[n_idx].post_neurons_idx[i];
                }
            }
        }
        spdlog::info("=====================");

        //spdlog::info("Group connection count = {0}", group_connections.size());
        //std::set<Connection> s( group_connections.begin(), group_connections.end() );
        //group_connections.assign( s.begin(), s.end() );
        //spdlog::info("Group connection count = {0}", group_connections.size());
    }

public:
    Network() = default;

    void run(std::uint32_t sim_steps = 100)
    {

        // Vision generator
        Generator vision_generator = generators["vision"];
        int vis_spikes_num = 0;

        // Coin acceptor generator
        /*std::uint32_t ca_hw_frequency = 30;
        Generator ca_generator(ca_hw_frequency, sim_steps);
        int ca_spikes_num = 0;*/

        for (size_t step = 0; step < sim_steps; step++)
        {
            //simulate_generator_connections();
            // Process vision part (camera)
            if (vision_generator.fired()) {
                vis_spikes_num++;
                spdlog::info("[Vision] Fired at Step: {0}", step);
            }

            // Process coin acceptor part
            /*if (ca_generator.fired()) {
                ca_spikes_num++;
                spdlog::info("[Coin Acceptor] Fired at Step: {0}", step);
            }*/
        }
        spdlog::info("[Vision] Spikes: {0}", vis_spikes_num);
        //spdlog::info("[Coin Acceptor] Spikes: {0}", ca_spikes_num);
    }

    void create_generator(std::string generator_name, std::uint32_t hw_frequency,
                          std::uint32_t sim_steps)
    {
        // TODO: check for duplicate generator_name

        generators[generator_name] = Generator(hw_frequency, sim_steps);
    }

    // Create neuron group
    void create_group(std::string group_name, std::int32_t neuron_count)
    {
        // TODO: check neuron_count
        // TODO: check for duplicate group_name

        // one-to-one mapping b/w group names and their groups
        group_names.push_back(group_name);
        groups.push_back(NeuronGroup(neuron_count));
        // create random neuron connections in group
        std::size_t last_added_group_idx = groups.size() - 1;
        //connect_intra_group(last_added_group_idx);
    }

    void connect_generator(std::string generator_name, std::string group_name)
    {
        // Get Generator
        // TODO: check generator_name
        Generator &generator = generators[generator_name];

        // Get Group
        // TODO: check group_name
        auto group_idx = std::distance(group_names.begin(), find(group_names.begin(), group_names.end(), group_name));
        // TODO: check group_idx
        generator_group_ids.push_back(group_idx);
        NeuronGroup &neuron_group = groups[group_idx];
        // TODO: check neuron_group
        std::size_t neurons_count = neuron_group.size();
        std::size_t connection_num_mean = neurons_count / 3;
        std::size_t connection_num_stddev = connection_num_mean / 3;
        std::mt19937 rnd(std::random_device{}());
        auto conn_num_dist = NormDist(connection_num_mean, connection_num_stddev);
        int conn_num = static_cast<int>(std::round(conn_num_dist(rnd)));
        spdlog::info("[connect_generator] Generator {0} to Group {1} conns = {2}",
                     generator_name,
                     group_name,
                     conn_num);
        std::vector<std::uint32_t> ids(neurons_count);
        std::iota(std::begin(ids), std::end(ids), 0);
        GeneratorNeuronIds generator_neurons_ids;
        std::sample(ids.begin(), ids.end(),
                    std::back_inserter(generator_neurons_ids),
                    conn_num,
                    std::mt19937{std::random_device{}()});
        /*for (auto idx : sample)
            spdlog::info("[connect_generator] Generator to Group idx = {0}", idx);*/
        for (const auto &neuron_ids : generator_neurons_ids) {
            neuron_group[neuron_ids].generator_connected = true;
            // Connection(&neuron_group[idx], nullptr)
            Connection conn;
            conn.timers.resize(0);
            conn.spike_transmit_time = 2;
            neuron_group[neuron_ids].pre_conns.push_back(std::move(conn));
        }
        // Add neurons ids
        generator_neurons.push_back(generator_neurons_ids);
        connect_intra_group(group_idx);
        // Print info
        /*spdlog::info("=================");
        spdlog::info("Neuron Group ID = {0}", group_idx);
        for (const auto &n_idx : generator_neurons_ids)
            spdlog::info("Neuron Idx = {0}", n_idx);
        spdlog::info("=================");*/
    }

    void simulate_generator_connections()
    {
        /*
        for (std::size_t g_idx; g_idx < generator_group_ids.size(); g_idx++) {
            spdlog::info("=================");
            spdlog::info("Neuron Group ID = {0}", g_idx);
            NeuronGroup neuron_group = groups[g_idx];
            //neuron_group
            GeneratorNeuronIds generator_neurons_ids = generator_neurons[g_idx];
            for (const auto &n_idx : generator_neurons_ids)
                spdlog::info("Neuron Idx = {0}", n_idx);
            spdlog::info("=================");
        }*/

        // connectivity matrix testing
        /*std::vector<std::uint32_t> ids = {0, 1, 2, 3, 4, 5, 6};

        std::vector<std::uint32_t> ids_of_ids(ids.size());
        std::iota(std::begin(ids_of_ids), std::end(ids_of_ids), 0);

        std::vector<std::vector<std::uint32_t>> connectivity_matrix;
        for (std::size_t idx = 0; idx < ids.size(); idx++) {
            spdlog::info("=================");
            spdlog::info("Idx = {0}", ids[idx]);

            // Remove element by index
            ids.erase(ids_of_ids.begin() + idx);

            // Out connections
            std::vector<std::uint32_t> sample;
            std::sample(ids.begin(), ids.end(),
                        std::back_inserter(sample),
                        conn_num,
                        std::mt19937{std::random_device{}()});

            // Restore element
            ids_of_ids.push_back(idx);

            spdlog::info("=================");
        }*/
    }

    void show_debug_info()
    {
        // Neuron groups info
        for (std::uint32_t idx = 0; idx < groups.size(); idx++) {
            spdlog::info("[show_debug_info] Neuron Group = {0} | Count = {1}", group_names[idx], groups[idx].size());
        }
    }
};

#endif //SNN_NETWORK_H
