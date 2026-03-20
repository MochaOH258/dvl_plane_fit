cmake_minimum_required(VERSION 3.16)

project(dvl_plane_fit LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(dvl_plane_fit
    src/main.cpp
    src/dvl_plane.cpp
    src/pid.cpp
)

target_include_directories(dvl_plane_fit PRIVATE
    include
    third_party/eigen
)