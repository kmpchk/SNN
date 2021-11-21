#ifndef SNN_CONNECTION_H
#define SNN_CONNECTION_H

#include <vector>
#include <string>
#include "Neuron.h"

enum CONNECTION_TYPE {
	GENERATOR_TO_NEURON,
	NEURON_TO_NEURON
};

enum WEIGHT_BOUNDARIES {
    LOW = 15,
    HIGH = 35
};

class Neuron;

class Connection {
public:
    Connection();

	int weight;
	int spike_transmit_time = 5;
	std::vector<int> timers;
    std::string ID;
	CONNECTION_TYPE conn_type;
    Neuron *pre_neuron = nullptr, *post_neuron = nullptr;
};


#endif //SNN_CONNECTION_H
