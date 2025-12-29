cmake_minimum_required(VERSION 3.8)
project(mirobot_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)  # xacro 사용 시

install(
  DIRECTORY
    launch
    meshes
    rviz
    urdf
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
