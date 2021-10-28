#ifndef SNN_CONNECTION_H
#define SNN_CONNECTION_H

#include <vector>
#include "Neuron.h"

enum CONNECTION_TYPE {
	GENERATOR_TO_NEURON,
	NEURON_TO_NEURON
};

class Neuron;

class Connection {
public:
    Connection();

	int weight;
	int spike_transmit_time = 5;
	std::vector<int> timers;
	CONNECTION_TYPE conn_type;
};


#endif //SNN_CONNECTION_H
