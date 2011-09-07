PROJECT(LocatedObjects)

set(IN_ROS_SERVICE TRUE CACHE BOOL "Disable ROS")

IF (IN_ROS_SERVICE )
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
ENDIF (IN_ROS_SERVICE )
set(CREATE_YARP_SERVICE FALSE CACHE BOOL "Do you want to create a the yarp service")

# Generate pkg-config file
configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/liblo.pc.in ${CMAKE_CURRENT_SOURCE_DIR}/liblo.pc )
install( FILES ${CMAKE_CURRENT_SOURCE_DIR}/liblo.pc DESTINATION lib/pkgconfig )

INCLUDE_DIRECTORIES("include")

IF (WIN32)
#LIBXML2
  INCLUDE_DIRECTORIES("../3rdParty/libxml2-2.6.30.win32/include"
                      "../3rdParty/iconv-1.9.2.win32/include")
  LINK_DIRECTORIES( "../../3rdParty/libxml2-2.6.30.win32/lib")
  SET (LIBXML2_LIBRARIES libxml2.lib)

IF ( CREATE_YARP_SERVICE )
  #YARP
  INCLUDE_DIRECTORIES(../3rdParty/yarp/yarp-2.2.1/src/libYARP_OS/include)
  LINK_DIRECTORIES(../../3rdParty/yarp/yarp-2.2.1/build/lib/debug)
  SET (YARP_LIBRARIES YARP_OSd.lib ACEd.lib Winmm.lib)
ENDIF ( CREATE_YARP_SERVICE )

  #STATIC LIB UNDER WINDOWS
  ADD_LIBRARY(lo STATIC LocatedObject.cpp NewMatExhaustive.cpp ServiceInterface.cpp  ObjectContainer.cpp)

ELSE(WIN32)
find_package( PkgConfig )

  #LIBXML2
pkg_check_modules(LIBXML2 REQUIRED libxml-2.0)
include_directories(${LIBXML2_INCLUDE_DIR})
link_directories( ${LIBXML2_LIBRARY_DIRS} )
add_definitions( ${LIBXML2_CFLAGS} )

IF ( CREATE_YARP_SERVICE )
  #YARP
  SET (YARP_LIBRARIES YARP_OS)
ENDIF ( CREATE_YARP_SERVICE )
  #SHARED LIB FOR LINUX
  ADD_LIBRARY(lo SHARED LocatedObject.cpp NewMatExhaustive.cpp ServiceInterface.cpp  ObjectContainer.cpp)
ENDIF(WIN32)

TARGET_LINK_LIBRARIES(lo ${LIBXML2_LIBRARIES})

LINK_LIBRARIES(lo)



IF ( CREATE_YARP_SERVICE )

	ADD_EXECUTABLE(lo-service YarpService.cpp )
	TARGET_LINK_LIBRARIES(lo-service ${YARP_LIBRARIES})

ENDIF ( CREATE_YARP_SERVICE )


ADD_EXECUTABLE(test test.cpp UT_Timer.cpp)

INSTALL( TARGETS lo
         LIBRARY DESTINATION lib
         ARCHIVE DESTINATION lib )
IF ( CREATE_YARP_SERVICE )
INSTALL( TARGETS lo-service
         RUNTIME DESTINATION bin )
ENDIF ( CREATE_YARP_SERVICE )
INSTALL(
    FILES
        include/lo/YarpService.h
        include/lo/ServiceInterface.h
        include/lo/ServiceLocatedObject.h
        include/lo/ObjectContainer.h
        include/lo/lo.h
        include/lo/LocatedObject.h
        include/lo/NewMatExhaustive.h
        include/lo/UT_Timer.h
    DESTINATION include/lo )
INSTALL(
    FILES
        include/lo/NewMat/Array.h
        include/lo/NewMat/BandMatrix.h
        include/lo/NewMat/BaseMatrix.h
        include/lo/NewMat/CroutMatrix.h
        include/lo/NewMat/DiagonalMatrix.h
        include/lo/NewMat/Functions.h
        include/lo/NewMat/GeneralMatrix.h
        include/lo/NewMat/GenericMatrix.h
        include/lo/NewMat/IdentityMatrix.h
        include/lo/NewMat/LinearEquationSolver.h
        include/lo/NewMat/Matrix.h
        include/lo/NewMat/MatrixExceptions.h
        include/lo/NewMat/MatrixInput.h
        include/lo/NewMat/RectMatrix.h
        include/lo/NewMat/SquareMatrix.h
        include/lo/NewMat/SubMatrix.h
        include/lo/NewMat/SymmetricMatrix.h
        include/lo/NewMat/Temporary.h
        include/lo/NewMat/TriangularMatrix.h
        include/lo/NewMat/Vector.h
        include/lo/NewMat/general.h
        include/lo/NewMat/include.h
        include/lo/NewMat/myexcept.h
        include/lo/NewMat/nricMatrix.h
        include/lo/NewMat/utils.h
    DESTINATION include/lo/NewMat )

