// ThirdParty
#include "spdlog/spdlog.h"

// SNN
#include "NeuronGroup.h"

NeuronGroup::NeuronGroup(GroupOptions &group_opts)
{
    group_type = group_opts.group_type;
    neurons.resize(group_opts.neurons_count);
    name = group_opts.name;
    count = neurons.size();
}

void NeuronGroup::debug_info()
{
    spdlog::info("Neuron Group: {0}", name);
    spdlog::info("Neuron Group Count: {0}", neurons.size());
}