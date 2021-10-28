#ifndef SNN_NEURONGROUP_H
#define SNN_NEURONGROUP_H

// C++
#include <vector>
#include <cstdint>

// SNN
#include "Neuron.h"

enum GROUP_TYPE {
    ACTIVATOR,
    USUAL_NEURON
};

typedef struct _GroupOptions
{
    int neurons_count;
    std::string name;
    GROUP_TYPE group_type;
} GroupOptions;

class NeuronGroup {
public:
    NeuronGroup(GroupOptions &group_opts);

    // Debug info
    void debug_info();

    std::vector<Neuron> neurons;
    std::string name;
    int count;
    GROUP_TYPE group_type;
};


#endif //SNN_NEURONGROUP_H
