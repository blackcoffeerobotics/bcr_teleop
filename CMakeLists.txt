cmake_minimum_required(VERSION 3.16)
project(bcr_teleop)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)

install(PROGRAMS
    nodes/bcr_teleop_node.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()