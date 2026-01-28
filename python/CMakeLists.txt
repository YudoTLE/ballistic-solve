cmake_minimum_required(VERSION 4.2)

project(ballistic_solve_python LANGUAGES CXX)

find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

execute_process(
  COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
  OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE nanobind_ROOT
)
find_package(nanobind CONFIG REQUIRED)

nanobind_add_module(_core
    STABLE_ABI
    NB_STATIC
    src/bindings.cpp
)

target_link_libraries(_core
    PRIVATE
        ballistic_solve_core
)

install(TARGETS _core
  LIBRARY DESTINATION ballistic_solve
)

nanobind_add_stub(
    _core_stub
    MODULE _core
    OUTPUT _core.pyi
    PYTHON_PATH $<TARGET_FILE_DIR:_core>
    DEPENDS _core
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/_core.pyi
    DESTINATION ballistic_solve
)