cmake_minimum_required(VERSION 3.16)
project(group10_ex4)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# --- 1. FIND ALL DEPENDENCIES ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# --- 2. GENERATE CUSTOM SERVICE INTERFACE ---
# Defines the targets used for linking later
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CheckResources.srv"
  DEPENDENCIES geometry_msgs sensor_msgs
)

# --- 3. DEFINE EXECUTABLES AND LINK ---

# --- Turtlebot Server ---
add_executable(turtlebot_server src/turtlebot_server.cpp)

# Use standard ROS 2 dependency linking
ament_target_dependencies(turtlebot_server 
  rclcpp 
  sensor_msgs 
  geometry_msgs
)

# CRITICAL FIX: Explicitly add the include paths and link libraries 
# for the generated service code to satisfy the compiler/linker.
target_include_directories(turtlebot_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)
target_link_libraries(turtlebot_server 
  ${PROJECT_NAME}__rosidl_typesupport_cpp
  ${PROJECT_NAME}__rosidl_generator_cpp
)

# Explicitly ensure the build system builds the headers before compiling the executable
add_dependencies(turtlebot_server ${PROJECT_NAME}__rosidl_generator_c)
install(TARGETS
  turtlebot_server
  DESTINATION lib/${PROJECT_NAME}
)


# --- Burrow Client ---
add_executable(burrow_client src/burrow_client.cpp)

# Use standard ROS 2 dependency linking
ament_target_dependencies(burrow_client 
  rclcpp
)

# CRITICAL FIX: Explicitly add the include paths and link libraries 
# for the generated service code to satisfy the compiler/linker.
target_include_directories(burrow_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)
target_link_libraries(burrow_client 
  ${PROJECT_NAME}__rosidl_typesupport_cpp
  ${PROJECT_NAME}__rosidl_generator_cpp
)

# Explicitly ensure the build system builds the headers before compiling the executable
add_dependencies(burrow_client ${PROJECT_NAME}__rosidl_generator_c)
install(TARGETS
  burrow_client
  DESTINATION lib/${PROJECT_NAME}
)


# --- 4. INSTALL INTERFACE FILES ---
install(DIRECTORY srv
  DESTINATION share/${PROJECT_NAME}
)

# --- 5. EXPORT THE PACKAGE ---
# Export the generated typesupport headers
ament_export_dependencies(rclcpp sensor_msgs geometry_msgs ${PROJECT_NAME}__rosidl_typesupport_cpp)

# Include test support at the end
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()