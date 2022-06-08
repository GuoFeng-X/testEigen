cmake_minimum_required(VERSION 2.8.3)
project(test_Eigen)

######################
### Cmake flags
######################
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(Eigen3)
include_directories(
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(computerT computerT.cpp)

add_executable(MatrixComputer MatrixComputer.cpp)
