if (NOT EXISTS "${PROJECT_SOURCE_DIR}/contrib/spdlog/CMakeLists.txt")
    message(FATAL_ERROR "submodule contrib/spdlog is missing.
         To fix try run: \n git submodule update --init --recursive")
else ()
    add_subdirectory(${PROJECT_SOURCE_DIR}/contrib/spdlog ${CMAKE_CURRENT_BINARY_DIR}/spdlog)
endif ()
