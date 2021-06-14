#ifndef SNN_AXON_H
#define SNN_AXON_H

#include "Dendrite.h"

class Axon {
private:
	Dendrite dendrite;

public:

	Axon() = default;

	void connect(const Dendrite &d) {
		this->dendrite = d;
		this->dendrite.connect();
	}

	Dendrite disconnect() {
		this->dendrite.disconnect();
		return this->dendrite;
	}
};

#endif //SNN_AXON_H