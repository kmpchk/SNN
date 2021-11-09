// ThirdParty
#include "spdlog/spdlog.h"

// SNN
#include "NeuronGroup.h"

NeuronGroup::NeuronGroup(GroupOptions &group_opts)
{
    group_type = group_opts.group_type;
    neurons.resize(group_opts.neurons_count);
    for (int idx = 0; idx < neurons.size(); idx++) {
        neurons[idx].id = idx;
    }
    // by default, activator group is fired
    /*if (group_type == GROUP_TYPE::ACTIVATOR) {
        for (Neuron& neuron: neurons)
            neuron.set_fired(true);
    }*/
    name = group_opts.name;
    count = neurons.size();
}

void NeuronGroup::debug_info()
{
    spdlog::info("Neuron Group: {0}", name);
    spdlog::info("Neuron Group Count: {0}", neurons.size());
}