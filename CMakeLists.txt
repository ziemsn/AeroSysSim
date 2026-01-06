# Specify the minimum CMake version
cmake_minimum_required(VERSION 3.10)

# Set the project name
project(AeroSysSim)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(src)
add_subdirectory(apps)

enable_testing()
add_subdirectory(tests)

