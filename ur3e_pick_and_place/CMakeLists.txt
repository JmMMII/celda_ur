cmake_minimum_required(VERSION 3.8)
project(ur3e_pick_and_place)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(ur_client_library REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(srdfdom REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# add_executable(celda_pick_and_place src/celda_pick_and_place.cpp)
# ament_target_dependencies(celda_pick_and_place rclcpp moveit_task_constructor_core moveit_ros_planning_interface)

add_executable(celda_pick_and_place src/celda_pick_and_place.cpp)
ament_target_dependencies(celda_pick_and_place
  rclcpp
  std_msgs
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  shape_msgs
  ur_client_library
  moveit_ros_planning
  moveit_ros_move_group
  srdfdom
)


install(TARGETS celda_pick_and_place
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

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

ament_package()
