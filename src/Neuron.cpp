#include "Neuron.h"

Neuron::Neuron(IdType id, NeuronType neuron, NoiseType noise) :
        ID(id), NEURON(neuron), NOISE(noise), in(), out() {}

vector<pair<Neuron *, WeightType>> Neuron::STDP()
{
    /*
         * Asymmetric STDP of the Hebbian learning rule.
         * Approximated with ∆w = 3 / ∆t function along with discrete points.
         * Every possible state combination and delta (this.state - in.state = ∆t):
         * 4 - 3 = 1 | 3 - 3 = 0 | 2 - 3 = -1 | 1 - 3 = -2 | 0 - 3 = -3
         * 4 - 2 = 2 | 3 - 2 = 1 | 2 - 2 =  0 | 1 - 2 = -1 | 0 - 2 = -2
         * 4 - 1 = 3 | 3 - 1 = 2 | 2 - 1 =  1 | 1 - 1 =  0 | 0 - 1 = -1
         * 4 - 0 = 4 | 3 - 0 = 3 | 2 - 0 =  2 | 1 - 0 =  1 | 0 - 0 =  0
         * To avoid float division, the ∆t = { 4, 2, 0, -2 } cases are handled separately.
         * To avoid integer overflow in unsigned byte/int8 type (0..255),
         * if min/max weight is reached - do not decrease/increase weight further.
         *
         * Returns current neuron input vector.
         */
    int dt, dw, w;
    for (pair<Neuron *, WeightType> &i: in) {
        dt = state - i.first->state;
        switch (dt) {
            case 4:
                dw = 0;
                break;
            case 2:
                dw = 2;
                break;
            case 0:
                dw = 0;
                break;
            case -2:
                dw = -2;
                break;
            default:
                dw = 3 / dt;
        }
        w = i.second + dw;
        if (w < MIN_WEIGHT) i.second = MIN_WEIGHT;
        else if (w > MAX_WEIGHT) i.second = MAX_WEIGHT;
        else i.second = w;
    }
    return in;
}

pair<Neuron *, WeightType> Neuron::connect(Neuron &neuron)
{
    const WeightType DEFAULT_WEIGHT = 128;
    threshold = (in.size() + 1) * DEFAULT_WEIGHT / 2;
    neuron.out.push_back(this);
    return in.emplace_back(&neuron, DEFAULT_WEIGHT);
}

StateType Neuron::check(const bool stdp)
{
    /*
         * Check & update neuron state.
         * If the neuron is in a relative refractory state, calculate the sum:
         * Sum = Σ (Type * (Noise + Fired? * Weight).
         * Compare the sum to the threshold with a multiplier representing the state, assuming that:
         * when the neuron is in RefractoryStart state
         * it is harder to reach the threshold, so the multiplier is 2,
         * when the neuron is in RefractoryPeak && RefractoryEnd states
         * it is easier to reach the threshold, so the multiplier is 1.
         *
         * Uses "stdp" boolean variable to control if the neuron weights are static or dynamic:
         * if stdp is true - STDP function is called and weights are updated, else - weights remain static.
         * Returns current neuron state.
         */
    if (State::RefractoryStart >= state && state >= State::RefractoryEnd) {
        StateType multiplier = 1;
        if (state == State::RefractoryStart) {
            multiplier = 2;
        }
        SumType s = 0;
        for (const pair<Neuron *, WeightType> &i: in) {
            s += i.first->NEURON * (i.first->NOISE + (i.first->state == State::ActionEnd) * i.second);
            if (s >= (threshold * multiplier)) {
                state = State::ActionStart;
                break;
            }
        }
        //printf("[sum] weight sum = %d\n", s);
    }
    if (stdp) STDP();
    //printf("[check] state = %d\n", state);
    if (state > State::RefractoryEnd) return state--;
    else return state;
}

StateType Neuron::spike()
{
    /*
         * Simulate a spike in first layer neurons.
         *
         * Returns current neuron state.
         */
    if (state <= RefractoryStart) {
        state = 4;
    }
    return state;
}
