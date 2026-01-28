cmake_minimum_required(VERSION 4.2)
project(ballisticsolver)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(core)
add_subdirectory(python)

add_executable(test_ballistics test.cpp)
target_link_libraries(test_ballistics PRIVATE ballisticsolver_core)