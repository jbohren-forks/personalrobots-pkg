cmake_minimum_required (VERSION 2.6)

### If you're trying to debug include directives or so, this here
### might help:
set (CMAKE_VERBOSE_MAKEFILE ON)

set (ROS_BUILD_TYPE Release)
include (rosbuild)
add_definitions (-Wall)
rospack (mp_benchmarks)

##################################################

INCLUDE (FindOpenGL)
IF (OPENGL_FOUND)
  MESSAGE (STATUS "Found OpenGL in ${OPENGL_INCLUDE_DIR}")
  MESSAGE (STATUS "  OPENGL_LIBRARIES   ${OPENGL_LIBRARIES}")
  MESSAGE (STATUS "  OPENGL_gl_LIBRARY  ${OPENGL_gl_LIBRARY}")
  MESSAGE (STATUS "  OPENGL_glu_LIBRARY ${OPENGL_glu_LIBRARY}")
  INCLUDE_DIRECTORIES (${OPENGL_INCLUDE_DIR})
ELSE (OPENGL_FOUND)
  MESSAGE (FATAL_ERROR "OpenGL not found")
ENDIF (OPENGL_FOUND)

INCLUDE (FindGLUT)
IF (GLUT_FOUND)
  MESSAGE (STATUS "Found GLUT in ${GLUT_INCLUDE_DIR}")
  MESSAGE (STATUS "  GLUT_LIBRARIES    ${GLUT_LIBRARIES}")
  MESSAGE (STATUS "  GLUT_glut_LIBRARY ${GLUT_glut_LIBRARY}")
  MESSAGE (STATUS "  GLUT_Xmu_LIBRARY  ${GLUT_Xmu_LIBRARY}")
  MESSAGE (STATUS "  GLUT_Xi_LIBRARY   ${GLUT_Xi_LIBRARY}")
  INCLUDE_DIRECTORIES (${GLUT_INCLUDE_DIR})
ELSE (GLUT_FOUND)
  MESSAGE (FATAL_ERROR "GLUT not found")
ENDIF (GLUT_FOUND)

IF (WIN32)
  MESSAGE (STATUS "Detected Microsoft Windows")
  ADD_DEFINITIONS (-DWIN32)
ELSE (WIN32)
  IF (APPLE)
    MESSAGE (STATUS "Detected Mac OS X")
    ADD_DEFINITIONS (-DOSX)
    SET (EXTRA_LIBRARIES "-lobjc")
    MESSAGE (STATUS "  EXTRA_LIBRARIES: ${EXTRA_LIBRARIES}")
  ENDIF (APPLE)
 
  IF (CMAKE_SYSTEM_NAME MATCHES Linux)
    MESSAGE (STATUS "Detected Linux")
    ADD_DEFINITIONS (-DLINUX)
  ENDIF (CMAKE_SYSTEM_NAME MATCHES Linux)
 
  IF (CMAKE_SYSTEM_NAME MATCHES OpenBSD)
    MESSAGE (STATUS "Detected OpenBSD")
    ADD_DEFINITIONS (-DOPENBSD)
  ENDIF (CMAKE_SYSTEM_NAME MATCHES OpenBSD)
ENDIF(WIN32)

##################################################

rospack_add_library (mp_benchmarks src/setup.cpp)

rospack_add_executable (mpbench src/main.cpp)
target_link_libraries (mpbench mp_benchmarks costmap_2d sunflower sbpl_util nepumuk
                               ${GLUT_glut_LIBRARY} ${GLUT_Xmu_LIBRARY} ${GLUT_Xi_LIBRARY}
			       ${OPENGL_glu_LIBRARY} ${OPENGL_gl_LIBRARY})
