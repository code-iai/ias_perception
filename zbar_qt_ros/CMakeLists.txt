cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#PROJECT(zbar_qt_ros)
FIND_PACKAGE(Qt4 REQUIRED)
SET(zbar_qt_ros_SOURCES src/main.cpp src/zbargui.cpp)
SET(zbar_qt_ros_HEADERS src/zbargui.h)
QT4_WRAP_CPP(zbar_qt_ros_HEADERS_MOC ${zbar_qt_ros_HEADERS})
QT4_WRAP_UI(zbar_qt_ros_FORMS_HEADERS ${zbar_qt_ros_FORMS})
QT4_ADD_RESOURCES(zbar_qt_ros_RESOURCES_RCC ${zbar_qt_ros_RESOURCES})
INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS}) 

#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)

#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
include_directories(/usr/include/Qt4 /usr/include/opencv-2.3.1/opencv/)
#rosbuild_add_executable(Zbar_QT src/main.cpp)
rosbuild_add_executable(zbar_qt_ros src/CQTImageConvertor.cpp ${zbar_qt_ros_SOURCES} ${zbar_qt_ros_HEADERS_MOC} ${zbar_qt_ros_FORMS_HEADERS})
target_link_libraries(zbar_qt_ros ${QT_LIBRARIES})
#target_link_libraries(example ${PROJECT_NAME})




