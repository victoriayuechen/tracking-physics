cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(tracking-physics)
find_package(PCL 1.3 REQUIRED COMPONENTS common io)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(load_cloud load_cloud.cpp)
target_link_libraries(load_cloud ${PCL_LIBRARIES})