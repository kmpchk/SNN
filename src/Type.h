#ifndef SNN_TYPE_H
#define SNN_TYPE_H

enum GroupType {
    INPUT,
    HIDDEN,
    OUTPUT
};

enum NeuronType {
    Excitatory = 1,
    Inhibitory = -1
};

#endif //SNN_TYPE_H