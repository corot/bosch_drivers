cmake_minimum_required(VERSION 2.8.3)
project(uniserial)

catkin_package(
  INCLUDE_DIRS include/uniserial
  LIBRARIES uniserial
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(uniserial src/uniserial.cpp)
