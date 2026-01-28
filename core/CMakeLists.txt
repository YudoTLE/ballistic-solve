cmake_minimum_required(VERSION 4.2)
project(
    ballisticsolver_core
    VERSION 0.0.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

set(BALLISTICSOLVER_CORE_HEADERS
    include/ballisticsolver/environment.hpp
    include/ballisticsolver/platform.hpp
    include/ballisticsolver/projectile.hpp
    include/ballisticsolver/simulation.hpp
    include/ballisticsolver/trajectory.hpp
    include/ballisticsolver/targeting.hpp
    include/ballisticsolver/tools.hpp
)

set(BALLISTICSOLVER_CORE_SOURCES
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

add_library(ballisticsolver_core STATIC)

target_sources(ballisticsolver_core
    PUBLIC
        ${BALLISTICSOLVER_CORE_HEADERS}
    PRIVATE
        ${BALLISTICSOLVER_CORE_SOURCES}
)

target_include_directories(ballisticsolver_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(ballisticsolver_core
    PRIVATE
        ${PRIVATE_REQUIRED_EXTERNAL_LIBRARIES}
    PUBLIC
        ${PUBLIC_REQUIRED_EXTERNAL_LIBRARIES}
)

set_target_properties(ballisticsolver_core PROPERTIES POSITION_INDEPENDENT_CODE ON)