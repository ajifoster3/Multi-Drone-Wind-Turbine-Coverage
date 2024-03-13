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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Define the static library
add_library(GreedyIterativeCoveragePathPlanner STATIC IMPORTED)
# Specify the location of the static library
set_property(TARGET GreedyIterativeCoveragePathPlanner PROPERTY IMPORTED_LOCATION "${CMAKE_SOURCE_DIR}/lib/libGreedyIterativeCoveragePathPlanner.a")

include_directories(${CMAKE_SOURCE_DIR}/include/GreedyIterativeCoveragePathPlanner include ${GeographicLib_INCLUDE_DIRS})



add_executable(
  central_coverage_controller 
  src/CentralCoverageController.cpp 
  src/CentralCoverageControllerNode.cpp 
  include/CentralCoverageControllerNode.h )
target_link_libraries(central_coverage_controller GreedyIterativeCoveragePathPlanner ${GeographicLib_LIBRARIES})
ament_target_dependencies(central_coverage_controller rclcpp geometry_msgs mavros_msgs)

install(TARGETS
  central_coverage_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()
