cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

project(SNN)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependencies
add_subdirectory(contrib)

# Source
file(GLOB SOURCES src/*.cpp)
file(GLOB HEADERS src/*.h)

add_executable(Neuron ${SOURCES} ${HEADERS})
# logging
target_link_libraries(Neuron PUBLIC spdlog::spdlog)