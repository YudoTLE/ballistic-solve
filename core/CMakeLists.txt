cmake_minimum_required(VERSION 4.2)
project(
    ballistic_solve_core
    VERSION 0.1.1
    LANGUAGES CXX
)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(ballistic_solve_core STATIC
    src/ballistic.cpp
    src/environment.cpp
    src/projectile.cpp
    src/utility.cpp
)

target_sources(ballistic_solve_core
    PUBLIC
        FILE_SET HEADERS
        BASE_DIRS include
        FILES
            include/ballistic_solve/ballistic.hpp
            include/ballistic_solve/environment.hpp
            include/ballistic_solve/projectile.hpp
            include/ballistic_solve/tools.hpp
            include/ballistic_solve/tools.inl
            include/ballistic_solve/utility.hpp
)

target_include_directories(ballistic_solve_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(ballistic_solve_core
    PUBLIC
        Boost::boost
        Eigen3::Eigen
)

target_compile_features(ballistic_solve_core PUBLIC cxx_std_20)

set_target_properties(ballistic_solve_core PROPERTIES 
    POSITION_INDEPENDENT_CODE ON
)