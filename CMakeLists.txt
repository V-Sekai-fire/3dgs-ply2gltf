cmake_minimum_required(VERSION 3.11)

project(ply2gltf)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(${CMAKE_SOURCE_DIR})

add_executable(ply2gltf io.cpp dump.cpp main.cpp)
