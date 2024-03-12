cmake_minimum_required(VERSION 3.10)
project(GreedyIterativeCoveragePathPlanner)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include the directory where the header files are located
include_directories(include)

# Collect all source files except main.cpp into a variable
file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_SOURCE_DIR} "src/*.cpp")
list(FILTER SOURCES EXCLUDE REGEX "main\\.cpp$")

# Create a static library from the collected source files
add_library(GreedyIterativeCoveragePathPlanner STATIC ${SOURCES})
