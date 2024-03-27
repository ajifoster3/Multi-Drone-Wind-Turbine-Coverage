cmake_minimum_required(VERSION 3.8)
project(central_ground_coverage_controller)


# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package (GeographicLib REQUIRED)

# Define the static library
add_library(GreedyIterativeCoveragePathPlanner STATIC IMPORTED)
# Specify the location of the static library
set_property(TARGET GreedyIterativeCoveragePathPlanner PROPERTY IMPORTED_LOCATION "${CMAKE_SOURCE_DIR}/lib/libGreedyIterativeCoveragePathPlanner.a")

include_directories(
  ${CMAKE_SOURCE_DIR}/include/CentralGroundCoverageController 
  ${CMAKE_SOURCE_DIR}/include/GreedyIterativeCoveragePathPlanner 
  ${CMAKE_SOURCE_DIR}/include/tomlpp
  ${GeographicLib_INCLUDE_DIRS})

add_executable(
  CentralCoverageController 
  src/CentralCoverageController.cpp 
  src/CentralCoverageControllerNode.cpp 
  src/TimedCoveragePath.cpp 
  src/TimedCoverageViewpoint.cpp 
  src/CoverageLogger.cpp
  include/CentralGroundCoverageController/CentralCoverageControllerNode.h )
target_link_libraries(CentralCoverageController GreedyIterativeCoveragePathPlanner ${GeographicLib_LIBRARIES})
ament_target_dependencies(CentralCoverageController rclcpp geometry_msgs mavros_msgs)

install(TARGETS
CentralCoverageController
  DESTINATION lib/${PROJECT_NAME})

ament_package()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test 
  test/TimedCoveragePathTest.cc
  src/TimedCoveragePath.cpp
  src/TimedCoverageViewpoint.cpp
  src/CentralCoverageControllerNode.cpp
  src/CoverageLogger.cpp
  )
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}_test
    std_msgs
    rclcpp 
    geometry_msgs 
    mavros_msgs
  )
  target_link_libraries(${PROJECT_NAME}_test
    GreedyIterativeCoveragePathPlanner
    ${GeographicLib_LIBRARIES}
  )
endif()

