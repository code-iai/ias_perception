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


#common commands for building c++ executables and libraries
rosbuild_add_executable(cop_collision_srv 
   src/CollisionInterface.cpp
   src/main.cpp
 )

set(DEBUG TRUE CACHE BOOL TRUE)

if (DEBUG )
  ADD_DEFINITIONS(-g -D_DEBUG -DBOOST_THREAD -DLOGFILE)
else (DEBUG)
  ADD_DEFINITIONS(-o3 -DBOOST_THREAD -DLOGFILE -DBOOST_1_35)
endif (DEBUG)
