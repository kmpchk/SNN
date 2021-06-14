#ifndef SNN_NEURON_H
#define SNN_NEURON_H

#include <cstdint>
#include <algorithm>
#include <chrono>
#include <vector>
#include "Synapse.h"

using namespace std;

using milliseconds = chrono::duration<uint64_t, ratio<1, 1000>>;
using timestamp = chrono::steady_clock::time_point;

typedef SYNAPSE_TYPE NEURON_TYPE;

class Neuron {
private:
	static const int8_t ABSOLUTE_REFRACTORY_PERIOD = 2;
	static const int8_t RELATIVE_REFRACTORY_PERIOD = 3;
	int8_t current_potential;
	uint64_t spike_time;
	vector<Synapse*> synapses;

public:
	Neuron(NEURON_TYPE neuron_type) {
		this->current_potential = 1;
		this->spike_time = 0;
		// synapses[0] = Axon
		synapses.emplace_back(new Synapse(this, neuron_type));
	}

	~Neuron() = default;

	void connect(Neuron *neuron)
    {
        synapses[0]->connect(neuron);
        neuron->addSynapse(synapses[0]);
    }

    void addSynapse(Synapse* s)
    {
        synapses.push_back(s);
    }

	void spike()
	{

	}
};

#endif //SNN_NEURON_H