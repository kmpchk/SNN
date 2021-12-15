// ThirdParty
#include "spdlog/spdlog.h"

// SNN
#include "NeuronGroup.h"

NeuronGroup::NeuronGroup(GroupOptions &group_opts)
{
    group_type = group_opts.group_type;
    neurons_type = group_opts.neurons_type;
    neurons.reserve(group_opts.neurons_count);
    for (int idx = 0; idx < group_opts.neurons_count; idx++) {
        neurons.emplace_back(Neuron(idx, neurons_type, 1));
    }
    name = group_opts.name;
    count = group_opts.neurons_count;
}

void NeuronGroup::debug_info()
{
    spdlog::info("Neuron Group: {0}", name);
    spdlog::info("Neuron Group Count: {0}", neurons.size());
}