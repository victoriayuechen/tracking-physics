cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(tracking-physics)
find_package(PCL 1.10 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS}
                    ${PROJECT_SOURCE_DIR}/include
                    ${PROJECT_SOURCE_DIR}/src)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Executable for tracking
add_executable(tracking src/imu.hpp src/imu.cpp
                 src/pcl_tracking.hpp src/pcl_tracking.cpp
                 src/main.cpp)
target_link_libraries(tracking ${PCL_LIBRARIES})

# Executable for tutorial of tracking 
add_executable(example examples/tracking_example.cpp examples/tracking_example.cpp)
target_link_libraries(example ${PCL_LIBRARIES})

# add_executable(load_cloud examples/load_cloud.cpp)                
# target_link_libraries(load_cloud ${PCL_LIBRARIES})