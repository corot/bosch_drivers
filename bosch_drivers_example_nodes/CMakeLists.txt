cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_add_executable( example_bma180_i2c src/example_bma180_i2c.cpp)
target_link_libraries(example_bma180_i2c arduino_interface bma180 bma180_parameters )


rosbuild_add_executable( example_bma180_spi src/example_bma180_spi.cpp)
target_link_libraries(example_bma180_i2c arduino_interface bma180 bma180_parameters )

rosbuild_add_executable( example_bma180_i2c_sub20 src/example_bma180_i2c_sub20.cpp)
target_link_libraries(example_bma180_i2c_sub20   sub20_interface bma180 bma180_parameters)

rosbuild_add_executable(example_bmc050 src/example_bmc050.cpp)
target_link_libraries(example_bmc050 arduino_interface bmc050 bmc050_parameters)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
