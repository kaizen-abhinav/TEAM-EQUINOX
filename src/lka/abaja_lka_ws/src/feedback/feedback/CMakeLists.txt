cmake_minimum_required(VERSION 3.8)
project(feedback)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Add message files
rosidl_generate_interfaces(${PROJECT_NAME}
"msg/Velocity.msg"
DEPENDENCIES builtin_interfaces
)
# Export dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_package()
