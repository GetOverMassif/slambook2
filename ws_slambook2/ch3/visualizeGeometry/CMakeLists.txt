cmake_minimum_required( VERSION 2.8 )
project( visualizeGeometry )

set(CMAKE_CXX_FLAGS "-std=c++14")

# 添加 Eigen 头文件
include_directories("/use/include/eigen3")

# 添加 Pangolin 依赖
find_package( Pangolin )
include_directories( ${Pangolin_INCLUDE_DIRS} )

add_executable(visualizeGeometry visualizeGeometry.cpp)