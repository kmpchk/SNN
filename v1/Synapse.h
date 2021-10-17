#ifndef SNN_SYNAPSE_H
#define SNN_SYNAPSE_H

#include <random>
#include <memory>

#include "Neuron.h"
#include "structs.h"

class Neuron;

class Synapse
{
private:
    Neuron *input_neuron, *output_neuron;
    uint8_t weight;
    SYNAPSE_TYPE synapse_type;
    //static std::mt19937 gen(std::random_device());

    void generateWeight(SYNAPSE_TYPE synapse_type);

public:
    Synapse(Neuron *input_neuron, SYNAPSE_TYPE synapse_type);

    void connect(Neuron *output_neuron);

    void STDP();
};

#endif //SNN_SYNAPSE_H