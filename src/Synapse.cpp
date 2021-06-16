// C++11
#include <random>
#include <memory>

// Project headers
#include "Synapse.h"

void Synapse::generateWeight(SYNAPSE_TYPE synapse_type)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    if (synapse_type == SYNAPSE_TYPE::Excitatory) {
        std::uniform_int_distribution<int> excitatory_distrib(0, 127);
        // Generate synapse weight
        this->weight = excitatory_distrib(gen);
    }
    else {
        std::uniform_int_distribution<int> inhibitory_distrib(-128, 0);
        // Generate synapse weight
        this->weight = inhibitory_distrib(gen);
    }
}

Synapse::Synapse(Neuron *input_neuron, SYNAPSE_TYPE synapse_type)
{
    this->input_neuron = input_neuron;
    this->synapse_type = synapse_type;
    generateWeight(synapse_type);
}

void Synapse::connect(Neuron *output_neuron)
{
    this->output_neuron = output_neuron;
}

void Synapse::STDP()
{
    auto time_delta = input_neuron->getSpikeTime() - output_neuron->getSpikeTime();
}
