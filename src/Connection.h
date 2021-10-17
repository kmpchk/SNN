#ifndef SNN_CONNECTION_H
#define SNN_CONNECTION_H

#include <vector>
#include "Neuron.h"

enum CONNECTION_TYPE {
	GENERATOR_TO_NEURON,
	NEURON_TO_NEURON
};

class Connection {
public:
	Connection(Neuron *post, Neuron *pre);

	void transmitSignal();

	bool isSignalsTransmitted();

	void update();

private:
	int weight;
	int spike_transmit_time = 5;
	std::vector<int> timers;
	CONNECTION_TYPE conn_type;
	Neuron *pre_neuron;
	Neuron *post_neuron;
};


#endif //SNN_CONNECTION_H
