cmake_minimum_required(VERSION 3.8)
project(ros2_eval_task)

# ---------------- Dependencies ----------------
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ---------------- Install Python Modules ----------------
ament_python_install_package(${PROJECT_NAME})

# ---------------- C++ Library ----------------
# Include directories
include_directories(include)

# Create the gazebo_utils library
add_library(gazebo_utils SHARED
  src/gazebo_utils_client.cpp
)

# Link dependencies to the library
ament_target_dependencies(gazebo_utils
  rclcpp
  geometry_msgs
  gazebo_msgs
)

# Install the library
install(TARGETS gazebo_utils
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

# Install header files
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

# Export the library for other packages to use
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp geometry_msgs gazebo_msgs)

# ---------------- C++ Executables ----------------
# Model spawner node executable
add_executable(model_spawner_node src/model_spawner_node.cpp)

# Link dependencies to the executable
ament_target_dependencies(model_spawner_node
  rclcpp
  geometry_msgs
  ament_index_cpp
)

# Link against our gazebo_utils library
target_link_libraries(model_spawner_node gazebo_utils)

# Install the executable
install(TARGETS model_spawner_node
  DESTINATION lib/${PROJECT_NAME})

# ---------------- Install Launch, RViz, and Worlds ----------------
install(DIRECTORY
  launch
  rviz
  worlds
  models
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  models/
  DESTINATION $ENV{HOME}/.gazebo/models
)

# ---------------- Install Package.xml ----------------
install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# ---------------- Export ----------------
ament_package()

