cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

project(SNN)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


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

file(GLOB SOURCES src/*.cpp)
file(GLOB HEADERS src/*.h)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES test
        CATKIN_DEPENDS roscpp rospy std_msgs
        # DEPENDS spdlog h5pp Boost
)

add_executable(Neuron ${SOURCES} ${HEADERS})

target_link_libraries(SNN PUBLIC spdlog::spdlog h5pp::h5pp ${catkin_LIBRARIES})
target_include_directories(SNN PRIVATE ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})