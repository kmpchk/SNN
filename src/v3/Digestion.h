#ifndef SNN_DIGESTION_H
#define SNN_DIGESTION_H

#include <iostream>

enum Coin {
	Genuine = 1,
	Fake = -1
};

class Acceptor {
public:
	Acceptor() {

	}

	uint8_t accept() {

	}
};

class Digestion {
private:
	Acceptor acceptor;

public:
	Digestion() {
		acceptor = Acceptor();
	}

	bool digest(Coin coin) {
		return coin.fake();
	}
};

#endif //SNN_DIGESTION_H