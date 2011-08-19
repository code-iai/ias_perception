PROJECT(cop)

cmake_minimum_required(VERSION 2.6)
INCLUDE("${CMAKE_MODULE_PATH}/MakeDocu.cmake" OPTIONAL)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

find_package(PkgConfig)

pkg_check_modules(LIBXML2 REQUIRED libxml-2.0)
include_directories(${LIBXML2_INCLUDE_DIR})
link_directories( ${LIBXML2_LIBRARY_DIRS} )
add_definitions( ${LIBXML2_CFLAGS} )

LINK_LIBRARIES(m lo cop)


rosbuild_add_executable(cop_srv ./src/main.cpp)

ROSBUILD_ADD_LIBRARY(cop
./src/AlgorithmEval.cpp      
./src/RelPose.cpp		 
./src/AttentionManager.cpp   
./src/AttentionAlgorithm.cpp   
./src/Signature.cpp
./src/Class.cpp
./src/SignatureDB.cpp
./src/RelPoseFactory.cpp
./src/Descriptor.cpp         
./src/Statistics.cpp
./src/Elem.cpp               
./src/TrackAlgorithm.cpp
./src/VisFinder.cpp
./src/ImageInputSystem.cpp   
./src/VisLearner.cpp
./src/XMLTag.cpp
./src/LocateAlgorithm.cpp  
./src/Object.cpp       
./src/RefineAlgorithm.cpp	
./src/ProveAlgorithm.cpp
./src/LogFile.cpp
./src/ROSjloComm.cpp
./src/ROSComm.cpp             
./src/File.cpp     
./src/Reading.cpp
./src/Sensor.cpp
./src/PerceptionPrimitive.cpp
)

set(DEBUG TRUE CACHE BOOL TRUE)

if (DEBUG )
  ADD_DEFINITIONS(-g -D_DEBUG -DBOOST_THREAD -DLOGFILE)
else (DEBUG)
  ADD_DEFINITIONS(-o3 -DBOOST_THREAD -DLOGFILE -DBOOST_1_35)
endif (DEBUG)

