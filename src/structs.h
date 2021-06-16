#ifndef SNN_STRUCTS_H
#define SNN_STRUCTS_H

// Synapse type
enum SYNAPSE_TYPE
{
    Excitatory = 1,
    Inhibitory = -1
};

// Neuron type
typedef SYNAPSE_TYPE NEURON_TYPE;

// Neuron group type
typedef NEURON_TYPE GROUP_TYPE;

#endif //SNN_STRUCTS_H
