cmake_minimum_required(VERSION 3.5)
project(gz_camera_macros)

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
