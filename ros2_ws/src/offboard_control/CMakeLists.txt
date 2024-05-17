cmake_minimum_required(VERSION 3.8)
project(offboard_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(offboard_control_interfaces REQUIRED)
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

include_directories(
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/include/nlohmann
  ${GeographicLib_INCLUDE_DIRS}
)


add_executable(takeoff src/Takeoff.cpp src/OffboardNode.cpp src/HaversineDistance.cpp src/CoverageViewpointLoader.cpp)

target_link_libraries(takeoff ${GeographicLib_LIBRARIES})

ament_target_dependencies(takeoff rclcpp geometry_msgs mavros_msgs offboard_control_interfaces)

install(TARGETS
   takeoff
   DESTINATION lib/${PROJECT_NAME})

ament_package()
