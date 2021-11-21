#include "Neuron.h"

Neuron::Neuron()
{
    //connection = Connection();
}

void Neuron::setV(double new_V)
{
	V = new_V;
}

double Neuron::getV() const
{
	return V;
}

void Neuron::update()
{

}

bool Neuron::fired() const
{
    return is_fired;
}

bool Neuron::spiked() const
{
    return V > 60;
}

void Neuron::set_fired(bool is_fired)
{
    this->is_fired = is_fired;
}