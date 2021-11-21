#ifndef SNN_UTILS_H
#define SNN_UTILS_H

// Spdlog
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>

inline constexpr auto MAX_LOG_FILE_SIZE_BYTES = 50 * 1024 * 1024;

// Timer
#include <chrono>
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;
using std::chrono::seconds;

void init_logger(spdlog::level::level_enum terminal_log_level,
                 spdlog::level::level_enum file_log_level);

int normalize_number(double value, double min, double max, double new_min, double new_max);

#endif //SNN_UTILS_H
