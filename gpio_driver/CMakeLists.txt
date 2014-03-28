cmake_minimum_required(VERSION 2.4.6)
project(gpio_driver)

find_package(catkin REQUIRED roscpp bosch_drivers_common)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gpio_driver
  CATKIN_DEPENDS roscpp bosch_drivers_common
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(gpio_driver src/gpio_driver.cpp)
