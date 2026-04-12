cmake_minimum_required(VERSION 3.8)
project(agx_arm_moveit)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/publish_table_scene.py DESTINATION lib/${PROJECT_NAME})
