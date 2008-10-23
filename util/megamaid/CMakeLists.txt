cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(megamaid)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_executable(playback src/playback/playback.cpp)
rospack_add_executable(stepback src/stepback/stepback.cpp)
rospack_add_executable(vacuum src/vacuum/vacuum.cpp)
rospack_add_executable(dustbuster src/dustbuster/dustbuster.cpp)
rospack_add_executable(central_vac src/central_vac/central_vac.cpp)


