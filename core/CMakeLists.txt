cmake_minimum_required(VERSION 4.2)
project(
    ballistic_solve_core
    VERSION 0.0.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

set(BALLISTIC_SOLVE_CORE_HEADERS
    include/ballistic-solve/environment.hpp
    include/ballistic-solve/platform.hpp
    include/ballistic-solve/projectile.hpp
    include/ballistic-solve/simulation.hpp
    include/ballistic-solve/trajectory.hpp
    include/ballistic-solve/targeting.hpp
    include/ballistic-solve/tools.hpp
)

set(BALLISTIC_SOLVE_CORE_SOURCES
    src/environment.cpp
    src/platform.cpp
    src/projectile.cpp
    src/simulation.cpp
    src/trajectory.cpp
    src/targeting.cpp
    src/tools.cpp
)

set(PRIVATE_REQUIRED_EXTERNAL_LIBRARIES
    Boost::boost
)

set(PUBLIC_REQUIRED_EXTERNAL_LIBRARIES
    Eigen3::Eigen
)

add_library(ballistic_solve_core STATIC)

target_sources(ballistic_solve_core
    PUBLIC
        ${BALLISTIC_SOLVE_CORE_HEADERS}
    PRIVATE
        ${BALLISTIC_SOLVE_CORE_SOURCES}
)

target_include_directories(ballistic_solve_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(ballistic_solve_core
    PRIVATE
        ${PRIVATE_REQUIRED_EXTERNAL_LIBRARIES}
    PUBLIC
        ${PUBLIC_REQUIRED_EXTERNAL_LIBRARIES}
)

set_target_properties(ballistic_solve_core PROPERTIES POSITION_INDEPENDENT_CODE ON)