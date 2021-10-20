#include "Connection.h"

Connection::Connection() : weight(35)
{
	//pre_neuron = pre;
	//post_neuron = post;
}

void Connection::transmitSignal()
{
	timers.push_back(spike_transmit_time);
}

bool Connection::isSignalsTransmitted()
{

}

void Connection::update()
{
	if (!timers.empty()) {
		for (auto timer = timers.begin(); timer != timers.end(); timer++) {
			if (*timer)
				(*timer)--;
			else
			{
				double new_V = post_neuron->getV();
				new_V += weight;
				post_neuron->setV(new_V);
				timers.erase(timer--);
			}
		}
	}
}