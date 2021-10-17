#ifndef SNN_NETWORK_H
#define SNN_NETWORK_H

#include "Connection.h"
#include "Neuron.h"
#include "Generator.h"

// C++
#include <unordered_map>

// ThirdParty
#include "spdlog/spdlog.h"

/*
 * INDEX --- GROUP NAME
 *   0   ---  SER (RED CAMERA)
 *   1   ---  SER (GREEN CAMERA)
 *   2   ---  DA ()
 */

class Network {
private:
    // one-to-one mapping b/w group names and their indices
    std::unordered_map<std::string, std::uint8_t> neuron_group_names_to_ids;
    std::uint8_t neuron_group_idx = 0;
    // one-to-one mapping b/w group indices and their groups
    std::unordered_map<std::uint8_t, std::vector<Neuron>> neuron_group_ids_to_groups;
    std::unordered_map<std::string, Generator> generators;

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

        // one-to-one mapping b/w group names and their indices
        neuron_group_names_to_ids[group_name] = neuron_group_idx;
        neuron_group_ids_to_groups[neuron_group_idx] = std::vector<Neuron>(neuron_count);
        neuron_group_idx++;
    }

    void connect_generator(std::string generator_name, std::string group_name)
    {

    }

    void show_debug_info()
    {
        for (const auto &item : neuron_group_names_to_ids) {
            spdlog::info("Group = {0} | ID = {1} | Neuron count = {2}",
                         item.first, item.second, neuron_group_ids_to_groups[item.second].size());
        }
    }
};

#endif //SNN_NETWORK_H
