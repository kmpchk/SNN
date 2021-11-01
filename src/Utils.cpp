// C++
#include <unistd.h>
#include <memory>
#include <chrono>
#include <ctime>
// SNN
#include "Utils.h"

void init_logger(spdlog::level::level_enum terminal_log_level,
                 spdlog::level::level_enum file_log_level) {
    std::time_t end_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    const auto root_log_path = "logs/snn_" + std::string(std::ctime(&end_time)) + ".log";

    // Showing logs in terminal
    auto terminal_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    terminal_sink->set_level(terminal_log_level);

    // Saving logs to file
    auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(root_log_path, MAX_LOG_FILE_SIZE_BYTES, 5);
    auto file_sink_loglevel = spdlog::level::trace;
    file_sink->set_level(file_log_level);

    auto sinks = std::vector<spdlog::sink_ptr>({file_sink, terminal_sink});

    auto root_logger = std::make_shared<spdlog::logger>("root", std::begin(sinks), std::end(sinks));
    root_logger->set_level(spdlog::level::trace);

    spdlog::set_default_logger(root_logger);

    spdlog::info("Process has started with PID = {0}", ::getpid());
    spdlog::info("Project log dir: {0}", root_log_path);
}
