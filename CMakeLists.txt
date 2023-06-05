cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(tracking-physics)
find_package(PCL 1.10 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS}
                    ${PROJECT_SOURCE_DIR}/include
                    ${PROJECT_SOURCE_DIR}/src)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Executable for tracking
add_executable(tracking src/tracker/pcl_tracking.hpp src/tracker/pcl_tracking.cpp
                 src/main.cpp)
target_link_libraries(tracking ${PCL_LIBRARIES})

# Executable for timing the tracking
add_executable(timing src/tracker/pcl_tracking.hpp src/tracker/pcl_tracking.cpp
        src/experiments/timing.cpp)
target_link_libraries(timing ${PCL_LIBRARIES})

# Executable for setting up the network
add_executable(network src/network/communication.hpp src/network/communication.cpp
        src/tracker/pcl_tracking.hpp src/tracker/pcl_tracking.cpp
        src/network/run_network.cpp)
target_link_libraries(network ${PCL_LIBRARIES})

# Dummy applications that "acts" like Unity engine (Testing purposes)
add_executable(dummy_unity_pos src/network/dummy_unity_pos.cpp)
add_executable(dummy_unity_pcd src/network/dummy_unity_pcd.cpp)
