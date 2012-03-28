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
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
include_directories(/usr/include/ImageMagick/)
#link_directories();
rosbuild_add_executable(zbar_barcode_reader src/zbar_barcode_reader.c)
target_link_libraries(zbar_barcode_reader  MagickWand MagickCore)

rosbuild_add_executable(zbar_barcode_reader_node src/zbar_barcode_reader_node.cpp)
target_link_libraries(zbar_barcode_reader_node Magick++ curl)

# rosbuild_add_executable(test_tinyxml src/tinyxml.cpp)
# target_link_libraries(test_tinyxml tinyxml)

# rosbuild_add_executable(curl_opencv src/curl_opencv.cpp)
# target_link_libraries(curl_opencv curl)
