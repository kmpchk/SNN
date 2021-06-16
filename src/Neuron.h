#ifndef SNN_NEURON_H
#define SNN_NEURON_H

#include <cstdint>
#include <algorithm>
#include <chrono>
#include <vector>
#include "Synapse.h"
#include "structs.h"

using namespace std;

//using milliseconds = chrono::duration<uint64_t, ratio<1, 1000>>;
//using timestamp = chrono::steady_clock::time_point;

class Synapse;

class Neuron {
private:
	static const int8_t ABSOLUTE_REFRACTORY_PERIOD = 2;
	static const int8_t RELATIVE_REFRACTORY_PERIOD = 3;
	int8_t current_potential;
	uint64_t spike_time;
	vector<Synapse*> synapses;
    int8_t leak = 0;
    int8_t noise = 0;
    bool fired = false;
    bool potential_enough = false;

    void genLeak();
    void genNoise();

public:
	Neuron(NEURON_TYPE neuron_type);

	~Neuron() = default;

	void connect(Neuron *neuron);

    void addSynapse(Synapse* s);

	void spike();

	void calculatePotential();

    uint64_t getSpikeTime();

    bool isFired();

    bool isPotentialEnough();
};

#endif //SNN_NEURON_H