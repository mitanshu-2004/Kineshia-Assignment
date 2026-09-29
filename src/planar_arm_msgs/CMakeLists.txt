cmake_minimum_required(VERSION 3.8)
project(planar_arm_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveToTarget.srv"
  "srv/PickPlace.srv"
  "msg/ArmStatus.msg"
)

ament_package()
