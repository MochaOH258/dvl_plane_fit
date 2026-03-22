cmake_minimum_required(VERSION 3.16)

project(dvl_plane_fit LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(CORE_SOURCES
  src/dvl_plane.cpp
  src/pid.cpp
  src/sub.cpp
)

add_library(dvl_core STATIC
    ${CORE_SOURCES}
)

target_include_directories(dvl_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen
)

add_executable(dvl_ros_node
  src/dvl_ros_node.cpp
)

target_link_libraries(dvl_ros_node
  dvl_core
)

ament_target_dependencies(dvl_ros_node
  rclcpp
  geometry_msgs
  sensor_msgs
)


add_executable(dvl_plane_fit
    src/main.cpp
)

target_link_libraries(dvl_plane_fit PRIVATE
    dvl_core
)

install(TARGETS
  dvl_ros_node
  dvl_plane_fit
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
