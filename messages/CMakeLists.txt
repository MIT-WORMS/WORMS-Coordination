cmake_minimum_required(VERSION 3.8)
project(messages)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}_srv 
  "msg/Personal.msg" 
  "msg/System.msg" 
  "msg/Readiness.msg" 
  "msg/Configuration.msg"
  "msg/State.msg"
  "msg/Command.msg"
  "msg/StateCommunication.msg"
  LIBRARY_NAME ${PROJECT_NAME} DEPENDENCIES sensor_msgs)

ament_package()

