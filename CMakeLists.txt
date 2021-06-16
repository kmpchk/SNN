project(SNN)

set(CMAKE_CXX_STANDARD 17)

add_executable(
        Neuron
        src/main.cpp
        src/Network.h
        src/Neuron.h
        src/Neuron.cpp
        src/Synapse.h
        src/Synapse.cpp
)