project(SNN)

set(CMAKE_CXX_STANDARD 17)

add_executable(
        Neuron
        src/Network.cpp
		src/Neuron.h
		src/Axon.h
		src/Dendrite.h
)