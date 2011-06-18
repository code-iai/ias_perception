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
#include_directories(${PROJECT_SOURCE_DIR}/src/sift/include)

#LINK_LIBRARIES(${PROJECT_SOURCE_DIR}/src/sift/lib/libfeat64.a)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
#target_link_libraries(${PROJECT_NAME} cv)
# add_dependencies(test bspline.o)
#rosbuild_add_executable(test src/test.cpp src/bspline.cpp)
#rosbuild_add_executable(snake src/snake.cpp src/bspline.cpp)
#rosbuild_add_executable(ros_to_openCV src/bspline.cpp src/ros_to_openCv.cpp)
#rosbuild_add_executable(ccd src/bspline.cpp src/ccd.cpp)
#rosbuild_add_executable(test_ccd src/bspline.cpp src/ccd.cpp src/test.cpp)
#rosbuild_add_executable(opencv_to_ros_node src/openCV_to_ros.cpp)
rosbuild_add_executable(ccd_node src/ccd.cpp src/bspline.cpp src/auto_init.cpp src/ccd_node.cpp)
rosbuild_add_executable(ccd_example src/ccd.cpp src/bspline.cpp src/auto_init.cpp src/example.cpp)
#rosbuild_add_executable(sift src/sift.cpp)
