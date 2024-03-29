cmake_minimum_required(VERSION 3.5)
project(quadruped_takahashi)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(kondo_b3m_ros2 REQUIRED)

set(srv_files
  "srv/Mode.srv"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
)
ament_export_dependencies(rosidl_default_runtime)

add_executable(quadruped_takahashi_odometry_node
  src/quadruped_takahashi_odometry.cpp
)
ament_target_dependencies(quadruped_takahashi_odometry_node
  rclcpp
  tf2_ros
  tf2_msgs
  tf2_geometry_msgs
  geometry_msgs
  sensor_msgs
)
target_include_directories(quadruped_takahashi_odometry_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(quadruped_takahashi_control_node
  src/quadruped_takahashi_control.cpp
)
ament_target_dependencies(quadruped_takahashi_control_node
  rclcpp
  tf2_ros
  tf2_msgs
  tf2_geometry_msgs
  geometry_msgs
  kondo_b3m_ros2
)
target_include_directories(quadruped_takahashi_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
rosidl_target_interfaces(quadruped_takahashi_control_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

install(TARGETS
quadruped_takahashi_odometry_node
quadruped_takahashi_control_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
