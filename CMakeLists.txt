cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

project(SNN)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Boost REQUIRED
        COMPONENTS
        system
        )

add_subdirectory(contrib)

file(GLOB SOURCES src/*.cpp)
file(GLOB HEADERS src/*.h)

add_executable(Neuron ${SOURCES} ${HEADERS})

target_link_libraries(Neuron PUBLIC spdlog::spdlog csv h5pp::h5pp)
target_include_directories(Neuron PRIVATE ${Boost_INCLUDE_DIRS} ${CSV_INCLUDE_DIR})