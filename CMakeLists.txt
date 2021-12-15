cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

project(SNN)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Release
set(CMAKE_BUILD_TYPE Release)
# Show all errors
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")
# High optimization level
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        )

find_package(Boost REQUIRED
        COMPONENTS
        system
        )

add_subdirectory(contrib)

set(SOURCE_FILES src/main.cpp src/Neuron.h src/NeuronGroup.cpp src/Utils.cpp)
set(HEADER_FILES src/Network.h  src/Type.h src/NeuronGroup.h src/Utils.h src/structs.h)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES test
        CATKIN_DEPENDS roscpp rospy std_msgs
        # DEPENDS spdlog h5pp Boost
)

add_executable(SNN ${SOURCE_FILES} ${HEADER_FILES})

target_link_libraries(SNN PUBLIC spdlog::spdlog h5pp::h5pp ${catkin_LIBRARIES})
target_include_directories(SNN PRIVATE ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})