#ifndef SNN_NEURON_H
#define SNN_NEURON_H

#include "Connection.h"
#include <vector>
#include <cstdint>
#include <functional>

namespace voltages {
	inline constexpr auto V_REST = -70;
	inline constexpr auto V_PEAK = 30;
	inline constexpr auto V_RESET = -80;
}

namespace ref_periods {
	inline constexpr auto ABSOLUTE = 2;
	inline constexpr auto RELATIVE = 3;
}

class Connection;

class Neuron {
public:
	Neuron();

	void setV(double new_V);

	double getV() const;

	void update();

    bool fired() const;

    void set_fired(bool is_fired);

    bool spiked() const;

    std::vector<Connection*> pre_conns;
    //std::vector<std::reference_wrapper<Connection>> pre_conns;
    std::vector<Connection> post_conns;
    std::vector<Neuron*> pre_neurons;
    //std::vector<std::reference_wrapper<Neuron>> pre_neurons;
    std::vector<Neuron*> post_neurons;
    //std::vector<std::reference_wrapper<Neuron>> post_neurons;

    int id;

    int V = voltages::V_REST; // Membrane Potential Voltage
    bool is_fired = false;
};


#endif //SNN_NEURON_H
