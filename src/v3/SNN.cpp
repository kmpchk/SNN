#include <chrono>
#include <thread>

#include "spdlog/spdlog.h"
#include "Digestion.h"

int main() {
	spdlog::info("SNN");



	uint32_t sim_steps = 10;
	for (int i = 0; i < sim_steps; ++i) {

		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	return 0;
}