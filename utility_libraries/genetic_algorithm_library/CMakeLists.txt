cmake_minimum_required(VERSION 3.10)
project(GeneticAlgorithm)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Get googletest
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
FetchContent_MakeAvailable(googletest)

find_package(GeographicLib REQUIRED)

# Include the directory where the header files are located
include_directories(
  include/GeneticAlgorithmLibrary 
  include/GeneticAlgorithmLibrary/EncodingMechanism
  include/GeneticAlgorithmLibrary/FitnessFunction
  include/GeneticAlgorithmLibrary/ReproductionMechanism
  include/GeneticAlgorithmLibrary/TerminationCriterion
  include/GeneticAlgorithmLibrary/tomlpp)

# Collect all source files except main.cpp into a variable
file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_SOURCE_DIR} "src/*.cpp")
list(FILTER SOURCES EXCLUDE REGEX "main\\.cpp$")

# Create a static library from the collected source files
add_library(GeneticAlgorithm STATIC ${SOURCES})

add_executable(GeneticAlgorithmExecutable src/main.cpp src/ProblemLogUtility.cpp ${SOURCES})

# Attempt to find the GeographicLib library manually
find_library(GEOGRAPHICLIB_LIB NAMES GeographicLib libGeographicLib)
target_link_libraries(GeneticAlgorithmExecutable PRIVATE stdc++fs)
target_link_libraries(GeneticAlgorithmExecutable PRIVATE ${GEOGRAPHICLIB_LIB})

## Create google test
#enable_testing()
#add_executable(
#    GeneticAlgorithmTest
#    test/DistanceFitnessFunction.cc
#)
#target_link_libraries(
#    GeneticAlgorithmTest
#    GeneticAlgorithm
#    GTest::gtest_main
#)
#include(GoogleTest)
##gtest_discover_tests(GeneticAlgorithmTest)
