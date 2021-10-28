#include "Neuron.h"

Neuron::Neuron()
{
    connection = Connection();
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