#include "Generator.h"

Generator::Generator(std::uint32_t frequency, std::uint32_t sim_steps) : frequency(frequency)
{
    time_resolution = 1.f / sim_steps;
}

void Generator::set_freq(std::uint32_t frequency)
{
    this->frequency = frequency;
}

bool Generator::fired()
{
    return safe_rand() < static_cast<float>(frequency) * time_resolution;
}

// Thread safe random generator
float Generator::safe_rand()
{
    thread_local static std::mt19937 gen(std::random_device{}());
    thread_local std::uniform_real_distribution<float> poisson(0.0, 1.0);
    return poisson(gen);
}