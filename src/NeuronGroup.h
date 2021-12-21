#ifndef SNN_NEURONGROUP_H
#define SNN_NEURONGROUP_H

// C++
#include <vector>
#include <cstdint>

// SNN
#include "Type.h"
#include "Neuron.h"
#include "Drawer.h"

typedef struct _GroupOptions
{
    int neurons_count;
    std::string name;
    GroupType group_type;
    NeuronType neurons_type;
} GroupOptions;

class NeuronGroup {
public:
    NeuronGroup(GroupOptions &group_opts, Drawer &drawer);

    // Debug info
    void debug_info();

    std::vector<Neuron> neurons;
    std::string name;
    int count;
    GroupType group_type;
    NeuronType neurons_type;
};


#endif //SNN_NEURONGROUP_H
