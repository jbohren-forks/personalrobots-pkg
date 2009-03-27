cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
include(FindPkgConfig)
rospack(stage)


add_custom_target(build_stage ALL
                  COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.stage)
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES stage-svn)

rospack_add_boost_directories()

pkg_check_modules(STAGE REQUIRED stage)
include_directories(${STAGE_INCLUDE_DIRS})
link_directories(${STAGE_LIBRARY_DIRS})
rospack_add_executable(bin/stageros src/stageros.cpp)
rospack_link_boost(bin/stageros thread)
add_dependencies(bin/stageros build_stage)

set(ENV{PKG_CONFIG_PATH} "${PROJECT_SOURCE_DIR}/stage/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}")
rospack_add_compile_flags(bin/stageros ${STAGE_CFLAGS_OTHERS})
target_link_libraries(bin/stageros ${STAGE_LIBRARIES})
rospack_add_link_flags(bin/stageros ${STAGE_LDFLAGS_OTHERS})


rospack_add_rostest(test/hztest.xml)
