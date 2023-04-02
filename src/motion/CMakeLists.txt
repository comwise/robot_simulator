cmake_minimum_required(VERSION 3.2)

include_directories(${PROJECT_SOURCE_DIR}/include/robot_sim)

file(GLOB_RECURSE SOURCE_FILES "*.c*")

add_library(motion STATIC ${SOURCE_FILES})
target_link_libraries(motion pthread rt)

add_executable(motion_test motion_test.cpp)
target_link_libraries(motion_test motion) 