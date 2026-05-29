cmake_minimum_required(VERSION 3.8)
project(endurance)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Control.msg"
  "msg/Velocity.msg"
  "msg/Pose.msg"
  "msg/RadarTrack.msg"
  "msg/RadarTrackList.msg"
  "msg/LaneDetection.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# Install python module (CarMaker library)
ament_python_install_package(endurance_core)

# Install executables (Python scripts)
install(PROGRAMS
  scripts/carmakercamera.py
  scripts/rsds_camera_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
