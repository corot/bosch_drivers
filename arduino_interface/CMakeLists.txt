cmake_minimum_required(VERSION 2.8.3)
project(arduino_interface)

find_package(catkin REQUIRED uniserial bosch_drivers_common)

catkin_package(
  INCLUDE_DIRS include/arduino_interface
  LIBRARIES arduino_interface
  CATKIN_DEPENDS uniserial bosch_drivers_common
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(arduino_interface src/arduino_interface.cpp)
target_link_libraries(arduino_interface uniserial)


#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)



#add_subdirectory(arduino_firmware)
#execute_process(COMMAND make -f Makefile)
