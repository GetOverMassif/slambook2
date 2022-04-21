cmake_minimum_required(VERSION 2.8)
project(useEigen)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS "-std=c++03")

# 添加Eigen头文件
include_directories("/use/include/eigen3")
add_executable(coordinateTransform coordinateTransform.cpp)