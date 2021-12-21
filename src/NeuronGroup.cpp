// ThirdParty
#include "spdlog/spdlog.h"
#include <string> 

// SNN
#include "NeuronGroup.h"
#include "Drawer.h"

NeuronGroup::NeuronGroup(GroupOptions &group_opts, Drawer &drawer)
{
    group_type = group_opts.group_type;
    neurons_type = group_opts.neurons_type;
    neurons.reserve(group_opts.neurons_count);
    int level;
    switch (group_type)
    {
    case INPUT:
        level = 1;
        break;
    case HIDDEN:
        level = 2;
        break;
    case OUTPUT:
        level = 3;
        break;
    default:
        break;
    }
    for (int idx = 0; idx < group_opts.neurons_count; idx++) {
        neurons.emplace_back(Neuron(idx, neurons_type, 1));
        drawer.add_node(level, group_opts.name + std::to_string(idx));
    }
    name = group_opts.name;
    count = group_opts.neurons_count;
}

void NeuronGroup::debug_info()
{
    spdlog::info("Neuron Group: {0}", name);
    spdlog::info("Neuron Group Count: {0}", neurons.size());
}