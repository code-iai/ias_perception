cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
#INCLUDE_DIRECTORIES($ENV{HALCONROOT}/include)
#LINK_DIRECTORIES($ENV{HALCONROOT}/lib/$ENV{HALCONARCH})
#LINK_LIBRARIES(halconcpp)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
rosbuild_add_library(SwissrangerRemoteSensor
src/swissRangerRemoteSensor.cpp
#src/ClusterDetector.cpp
src/SegmentPrototype.cpp
src/RangeSensor.cpp
)
