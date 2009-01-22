cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
include(AddFileDependencies)
rospack(ijg_libjpeg)

add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/lib/libjpeg.a
                          ${PROJECT_SOURCE_DIR}/include/ijg_libjpeg/jpeglib.h
                          ${PROJECT_SOURCE_DIR}/include/ijg_libjpeg/jmorecfg.h
                          ${PROJECT_SOURCE_DIR}/include/ijg_libjpeg/jerror.h
                          ${PROJECT_SOURCE_DIR}/include/ijg_libjpeg/jpegint.h
                          ${PROJECT_SOURCE_DIR}/include/ijg_libjpeg/jconfig.h
                   COMMAND mkdir -p ${PROJECT_SOURCE_DIR}/lib
                   COMMAND cd ${PROJECT_SOURCE_DIR}/build && tar xzf ../jpeg-6b.tar.gz
	           COMMAND cd ${PROJECT_SOURCE_DIR}/build/jpeg-6b && CFLAGS=-fPIC ./configure
	           COMMAND cd ${PROJECT_SOURCE_DIR}/build/jpeg-6b && make $ENV{PARALLEL_JOBS}
	           COMMAND cp ${PROJECT_SOURCE_DIR}/build/jpeg-6b/libjpeg.a ${PROJECT_SOURCE_DIR}/lib/libjpeg.a
	           COMMAND cp ${PROJECT_SOURCE_DIR}/build/jpeg-6b/jpeglib.h ${PROJECT_SOURCE_DIR}/include/ijg_libjpeg/jpeglib.h
	           COMMAND cp ${PROJECT_SOURCE_DIR}/build/jpeg-6b/jmorecfg.h ${PROJECT_SOURCE_DIR}/include/ijg_libjpeg/jmorecfg.h
	           COMMAND cp ${PROJECT_SOURCE_DIR}/build/jpeg-6b/jerror.h ${PROJECT_SOURCE_DIR}/include/ijg_libjpeg/jerror.h
	           COMMAND cp ${PROJECT_SOURCE_DIR}/build/jpeg-6b/jpegint.h ${PROJECT_SOURCE_DIR}/include/ijg_libjpeg/jpegint.h
	           COMMAND sed -e '/HAVE_STDLIB_H/d' -e '/HAVE_STDDEF_H/d' <${PROJECT_SOURCE_DIR}/build/jpeg-6b/jconfig.h >${PROJECT_SOURCE_DIR}/include/ijg_libjpeg/jconfig.h
                   )

rospack_add_library(ros_jpeg_mutex src/ros_jpeg_mutex.cpp)
add_file_dependencies(src/ros_jpeg_mutex.cpp ${PROJECT_SOURCE_DIR}/lib/libjpeg.a)

