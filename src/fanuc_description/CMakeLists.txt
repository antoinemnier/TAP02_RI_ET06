cmake_minimum_required(VERSION 3.10.2)
project(moveit_resources_fanuc_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    meshes
    urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
