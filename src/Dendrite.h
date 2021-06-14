#ifndef SNN_DENDRITE_H
#define SNN_DENDRITE_H

class Dendrite {
private:
	bool connected;

public:
	Dendrite() {
		this->connected = false;
	}

	void connect() {
		this->connected = true;
	}

	void disconnect() {
		this->connected = false;
	}

	bool is_connected() {
		return this->connected;
	}
};

#endif //SNN_DENDRITE_H