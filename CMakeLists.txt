cmake_minimum_required(VERSION 3.8)
project(primitive_actions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate action interfaces from within this package
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CartesianPathFollow.action"
  "action/Pick.action"
  "action/Place.action"
  DEPENDENCIES geometry_msgs
)

# Install Python nodes so `ros2 run primitive_actions <node>` works
install(PROGRAMS primitive_actions/cartesian_path_action_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cartesian_path_action_server
)
install(PROGRAMS primitive_actions/primitive_action_planner.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME primitive_action_planner
)
install(PROGRAMS primitive_actions/test_cartesian_circle.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME test_cartesian_circle
)
install(PROGRAMS primitive_actions/test_primitive_actions.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME test_primitive_actions
)
install(PROGRAMS primitive_actions/franka_move_to_start.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME franka_move_to_start
)
install(PROGRAMS primitive_actions/franka_error_recovery.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME franka_error_recovery
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install URDF files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
