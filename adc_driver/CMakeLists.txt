cmake_minimum_required(VERSION 2.8.3)
project(adc_driver)

find_package(catkin REQUIRED bosch_drivers_common)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES adc_driver
  CATKIN_DEPENDS bosch_drivers_common
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(adc_driver src/adc_driver.cpp)
