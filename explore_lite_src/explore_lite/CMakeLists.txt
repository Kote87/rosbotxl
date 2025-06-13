cmake_minimum_required(VERSION 3.5)
project(explore_lite)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav2_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
