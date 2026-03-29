cmake_minimum_required(VERSION 3.8)
project(zed2i_isaac_vslam)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/vio_watcher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
