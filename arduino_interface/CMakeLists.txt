cmake_minimum_required(VERSION 2.8.3)
project(arduino_interface)

find_package(catkin REQUIRED roscpp uniserial bosch_drivers_common)

catkin_package(
  INCLUDE_DIRS include/arduino_interface
  LIBRARIES arduino_interface
  CATKIN_DEPENDS roscpp uniserial bosch_drivers_common
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(arduino_interface src/arduino_interface.cpp)
target_link_libraries(arduino_interface ${catkin_LIBRARIES})


# Build the firmware
# add_subdirectory(arduino_firmware)
# execute_process(COMMAND make -f Makefile)
