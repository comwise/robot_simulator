cmake_minimum_required(VERSION 3.2)

include_directories(srv)

file(GLOB_RECURSE SOURCE_FILES "*.c*")

add_library(srv STATIC ${SOURCE_FILES})
target_link_libraries(srv motion pthread rt) 

add_executable(srv_node robot_test.cpp)
target_link_libraries(srv_node srv) 