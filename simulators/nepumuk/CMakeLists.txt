### Simplified CMake build file for ROSified nepumuk.

# depends on:
# - GL, GLU, GLUT
# - libsunflower ros-pkg
# - boost ros-pkg
#
# later (optional?) code will depend on:
# - estar ros-pkg
# - asl-mplan future ros-pkg
# - asl-mcontrol future ros-pkg

cmake_minimum_required(VERSION 2.6)
include(rosbuild)

rospack(nepumuk)

SET (ROS_BUILD_TYPE Debug)
SET (ROS_BUILD_STATIC_EXES false)
SET (ROS_BUILD_SHARED_LIBS true)
SET (ROS_BUILD_STATIC_LIBS false)

IF (WIN32)
  MESSAGE (STATUS "Detected Microsoft Windows")
  SET (ROS_COMPILE_FLAGS "-DWIN32 -Wall -DNPM_HAVE_ROS")
ELSE (WIN32)
  IF (APPLE)
    MESSAGE (STATUS "Detected Mac OS X")
    SET (ROS_COMPILE_FLAGS "-DOSX -Wall -DNPM_HAVE_ROS")
  ENDIF (APPLE)
  
  IF (CMAKE_SYSTEM_NAME MATCHES Linux)
    MESSAGE (STATUS "Detected Linux")
    SET (ROS_COMPILE_FLAGS "-DLINUX -Wall -DNPM_HAVE_ROS")
  ENDIF (CMAKE_SYSTEM_NAME MATCHES Linux)
  
  IF (CMAKE_SYSTEM_NAME MATCHES OpenBSD)
    MESSAGE (STATUS "Detected OpenBSD")
    SET (ROS_COMPILE_FLAGS "-DOPENBSD -Wall -DNPM_HAVE_ROS")
  ENDIF (CMAKE_SYSTEM_NAME MATCHES OpenBSD)
ENDIF(WIN32)

SET (ROS_LINK_FLAGS "")

rospack_add_executable(nepumuk src/simul/nepumuk.cpp)
rospack_add_executable(simpletest src/ros/test.cpp)



SET (CMAKE_VERBOSE_MAKEFILE ON)

INCLUDE (FindOpenGL)
IF (OPENGL_FOUND)
  MESSAGE (STATUS "Found OpenGL")
ELSE (OPENGL_FOUND)
  MESSAGE (FATAL_ERROR "OpenGL not found")
ENDIF (OPENGL_FOUND)
IF (OPENGL_GLU_FOUND)
  MESSAGE (STATUS "Found GLU")
ELSE (OPENGL_GLU_FOUND)
  MESSAGE (FATAL_ERROR "GLU not found")
ENDIF (OPENGL_GLU_FOUND)
INCLUDE_DIRECTORIES (${OPENGL_INCLUDE_DIR})

INCLUDE (FindGLUT)
IF (GLUT_FOUND)
  MESSAGE (STATUS "Found GLUT")
ELSE (GLUT_FOUND)
  MESSAGE (FATAL_ERROR "GLUT not found")
ENDIF (GLUT_FOUND)
INCLUDE_DIRECTORIES (${GLUT_INCLUDE_DIR})

INCLUDE (FindThreads)
IF (CMAKE_USE_PTHREADS_INIT)
  MESSAGE (STATUS "Found pthreads")
ELSE (CMAKE_USE_PTHREADS_INIT)
  MESSAGE (FATAL_ERROR "Sorry for the moment libsunflower still seems to need pthreads...")
ENDIF (CMAKE_USE_PTHREADS_INIT)


rospack_add_library (npm
	     src/common/BBox.cpp
	     src/common/BicycleDrive.cpp
	     src/common/BicycleDriveDrawing.cpp
	     src/common/Camera.cpp
	     src/common/CheatSheet.cpp
	     src/common/DiffDrive.cpp
	     src/common/DiffDriveDrawing.cpp
	     src/common/Drawing.cpp
	     src/common/Drive.cpp
	     src/common/Globals.cpp
	     src/common/GoalInstanceDrawing.cpp
	     src/common/HAL.cpp
	     src/common/HoloDrive.cpp
	     src/common/HoloDriveDrawing.cpp
	     src/common/Lidar.cpp
	     src/common/Manageable.cpp
	     src/common/Manager.cpp
	     src/common/MapperRefDrawing.cpp
	     src/common/MapperUpdateDrawing.cpp
	     src/common/NoiseModel.cpp
	     src/common/Object.cpp
	     src/common/OdometryDrawing.cpp
	     src/common/RobotClient.cpp
	     src/common/RobotDescriptor.cpp
	     src/common/RobotDrawing.cpp
	     src/common/RobotServer.cpp
	     src/common/RobotZoomCamera.cpp
	     src/common/ScannerDrawing.cpp
	     src/common/Sensor.cpp
	     src/common/Sharp.cpp
	     src/common/SharpDrawing.cpp
	     src/common/SimpleImage.cpp
	     src/common/StillCamera.cpp
	     src/common/TrajectoryDrawing.cpp
	     src/common/TraversabilityDrawing.cpp
	     src/common/View.cpp
	     src/common/World.cpp
	     src/common/WorldCamera.cpp
	     src/common/WorldDrawing.cpp
	     src/common/trfct.cpp
	     src/common/wrap_glu.cpp
	     src/robox/BBDrawing.cpp
	     src/robox/BLDrawing.cpp
	     src/robox/DODrawing.cpp
	     src/robox/DWDrawing.cpp
	     src/robox/GridLayerCamera.cpp
	     src/robox/GridLayerDrawing.cpp
	     src/robox/MPDrawing.cpp
	     src/robox/OCamera.cpp
	     src/robox/ODrawing.cpp
	     src/robox/RHDrawing.cpp
	     src/robox/Robox.cpp
	     src/visitor/Visitor.cpp
	     src/ros/ROSbot.cpp
	     src/simul/Interlock.cpp
	     src/simul/RobotFactory.cpp
	     src/simul/Simulator.cpp)

INCLUDE_DIRECTORIES (.)

TARGET_LINK_LIBRARIES (nepumuk npm sunflower
		       ${GLUT_LIBRARIES}
		       ${OPENGL_glu_LIBRARY}
		       ${OPENGL_gl_LIBRARY}
		       pthread)
