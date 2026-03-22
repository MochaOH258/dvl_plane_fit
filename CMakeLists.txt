cmake_minimum_required(VERSION 3.16)

project(dvl_plane_fit LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(dvl_core STATIC
    src/dvl_plane.cpp
    src/pid.cpp
)

target_include_directories(dvl_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen
)


add_executable(dvl_plane_fit
    src/main.cpp
)

target_link_libraries(dvl_plane_fit PRIVATE
    dvl_core
)

find_package(gz-transport REQUIRED)
find_package(gz-msgs REQUIRED)

add_executable(dvl_gz_node
    src/gz_node_main.cpp
)

target_link_libraries(dvl_gz_node PRIVATE
    dvl_core
    gz-transport
    gz-msgs
)
