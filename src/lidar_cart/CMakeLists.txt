cmake_minimum_required(VERSION 3.8)
project(lidar_cart)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(drive_node src/drive_node.cpp)
ament_target_dependencies(drive_node rclcpp geometry_msgs)

install(TARGETS drive_node
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/mcl_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mcl_node
)

install(PROGRAMS
  scripts/global_planner.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME global_planner
)

install(PROGRAMS
  scripts/local_planner.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME local_planner
)

install(DIRECTORY
  launch
  urdf
  worlds
  maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
