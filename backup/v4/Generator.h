#ifndef SNN_GENERATOR_H
#define SNN_GENERATOR_H

#include <cstdint>
#include <random>

class Generator {
public:
    Generator() = default;
    Generator(std::uint32_t frequency, std::uint32_t sim_steps);
    void set_freq(std::uint32_t frequency);
    bool fired();

private:
    float safe_rand();

    float time_resolution;
    std::uint32_t frequency;
};


#endif //SNN_GENERATOR_H
