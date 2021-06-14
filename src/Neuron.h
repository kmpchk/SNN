#ifndef SNN_NEURON_H
#define SNN_NEURON_H

#include <algorithm>
#include <chrono>
#include <vector>
#include "Axon.h"
#include "Dendrite.h"

using namespace std;

using milliseconds = chrono::duration<uint64_t, ratio<1, 1000>>;
using timestamp = chrono::steady_clock::time_point;

class Neuron {
private:
	static const int8_t MAX_POTENTIAL = 40;
	static const int8_t MIN_POTENTIAL = -90;
	static const int8_t RESTING_POTENTIAL = -70;
	static const int8_t THRESHOLD_POTENTIAL = -55;
	static const int8_t ABSOLUTE_REFRACTORY_PERIOD = 2;
	static const int8_t RELATIVE_REFRACTORY_PERIOD = 3;
	int8_t current_potential;
	uint64_t spike_time;
	vector<Dendrite> dendrites;
	Axon axon;

	/* If vector contains single dendrite and it is not connected - return it
	 * Else - create new Dendrite object on the heap and return it */
	Dendrite create_dendrite() {
		if (this->dendrites.size() == 1 && !this->dendrites[0].is_connected()) {
			return this->dendrites[0];
		} else {
			auto *d = new Dendrite();
			this->dendrites.push_back(*d);
			return *d;
		}
	}

	/* If vector contains single dendrite and it is not connected - do nothing
	 * Else - find dendrite, remove from vector and call its destructor */
	void delete_dendrite(Dendrite &d) {
		if (this->dendrites.size() != 1 || this->dendrites[0].is_connected()) {
			this->dendrites.erase(remove(
					this->dendrites.begin(),
					this->dendrites.end(), d
			));
		}
	}

public:
	Neuron() {
		this->current_potential = RESTING_POTENTIAL;
		this->spike_time = 0;
		this->axon = Axon();
		this->dendrites.emplace_back();
	}

	~Neuron() = default;

	void connect(Neuron &neuron) {
		auto d = neuron.create_dendrite();
		this->axon.connect(d);
	}

	void disconnect(Neuron &neuron) {
		auto d = this->axon.disconnect();
		neuron.delete_dendrite(d);
	}

	void spike() {

	}
};

#endif //SNN_NEURON_H