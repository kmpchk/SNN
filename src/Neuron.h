#ifndef SNN_NEURON_H
#define SNN_NEURON_H

namespace voltages {
	inline constexpr auto V_REST = -70;
	inline constexpr auto V_PEAK = 30;
	inline constexpr auto V_RESET = -80;
}

namespace ref_periods {
	inline constexpr auto ABSOLUTE = 2;
	inline constexpr auto RELATIVE = 3;
}

class Neuron {
public:
	Neuron();

	void setV(double new_V);

	double getV() const;

	void update();



private:
	double V = voltages::V_REST; // Membrane Potential Voltage
};


#endif //SNN_NEURON_H
