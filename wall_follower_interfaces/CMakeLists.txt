cmake_minimum_required(VERSION 3.8)
project(wall_follower_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED) # Crucial for Action definitions

# Declare the interface files
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/FindWall.srv"
  "action/OdomRecord.action"
  DEPENDENCIES geometry_msgs action_msgs
)

ament_package()