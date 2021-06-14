#ifndef SNN_SYNAPSE_H
#define SNN_SYNAPSE_H

#include <random>
#include <memory>

// Forward declaration
class Neuron;

enum SYNAPSE_TYPE
{
    Excitatory = 1,
    Inhibitory = -1
};

class Synapse
{
private:
    Neuron *input_neuron, *output_neuron;
    uint8_t weight;
    SYNAPSE_TYPE synapse_type;
    //static std::mt19937 gen(std::random_device());

    void generateWeight(SYNAPSE_TYPE synapse_type)
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

public:
    Synapse(Neuron *input_neuron, SYNAPSE_TYPE synapse_type)
    {
        this->input_neuron = input_neuron;
        this->synapse_type = synapse_type;
        generateWeight(synapse_type);
    }

    void connect(Neuron *output_neuron)
    {
        this->output_neuron = output_neuron;
    }
};

#endif //SNN_SYNAPSE_H