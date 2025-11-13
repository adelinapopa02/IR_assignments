cmake_minimum_required(VERSION 3.8)
project(group10_ex4)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# This command generates the C++ header for RefillBurrow.srv
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RefillBurrow.srv"
)

# --- Executable Definitions ---

# TurtleBot Server Node (Uses rclcpp, sensor_msgs, geometry_msgs, and custom service)
add_executable(turtlebot_server_node src/turtlebot_server_node.cpp)
ament_target_dependencies(turtlebot_server_node 
  rclcpp 
  sensor_msgs 
  geometry_msgs
)
install(TARGETS turtlebot_server_node DESTINATION lib/${PROJECT_NAME})


# Burrow Client Node (Uses rclcpp and custom service)
add_executable(burrow_client_node src/burrow_client_node.cpp)
ament_target_dependencies(burrow_client_node 
  rclcpp
)
install(TARGETS burrow_client_node DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# This is the command that makes the generated service header available
ament_package()