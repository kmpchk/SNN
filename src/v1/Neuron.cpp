#include <functional>
#include "Neuron.h"

Neuron::Neuron(NEURON_TYPE neuron_type)
{
    this->current_potential = 1;
    this->spike_time = 0;
    // synapses[0] = Axon
    synapses.emplace_back(new Synapse(this, neuron_type));
}

void Neuron::connect(Neuron *neuron)
{
    synapses[0]->connect(neuron);
    neuron->addSynapse(synapses[0]);
}

void Neuron::addSynapse(Synapse* s)
{
    synapses.push_back(s);
}

void Neuron::spike()
{

}

uint64_t Neuron::getSpikeTime()
{
    return this->spike_time;
}

void Neuron::genLeak()
{
    this->leak = 2;
}

void Neuron::genNoise()
{
    const double mean = 0.0;
    const double stddev = 0.1;
    auto dist = std::bind(std::normal_distribution<double>{mean, stddev},
                          std::mt19937(std::random_device{}()));
    this->noise = dist();
}

bool Neuron::isFired()
{
    return this->fired;
}

void Neuron::calculatePotential()
{
    //this->current_potential =
}

bool Neuron::isPotentialEnough()
{
    return this->potential_enough;
}
