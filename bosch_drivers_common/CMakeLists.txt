cmake_minimum_required(VERSION 2.8.3)
project(bosch_drivers_common)

catkin_package(
  INCLUDE_DIRS include/bosch_drivers_common
)

include_directories(include ${catkin_INCLUDE_DIRS})
