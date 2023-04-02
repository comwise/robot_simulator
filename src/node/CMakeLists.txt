cmake_minimum_required(VERSION 3.2)

include_directories(node)

file(GLOB_RECURSE SOURCE_FILES "*.c*")

add_library(node STATIC ${SOURCE_FILES})
target_link_libraries(node ros_node srv motion pthread rt) 

add_executable(sim_node main.cpp)
target_link_libraries(sim_node node)