cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(amcl_player)
set(new_code FALSE)

if(NOT new_code)

rospack_add_executable(amcl_player amcl_player.cc)
target_link_libraries(amcl_player playerdrivers)
rospack_add_executable(cli cli.cpp)

else(NOT new_code)

rospack_add_library(amcl_pf 
                    src/pf/pf.c 
                    src/pf/pf_kdtree.c 
                    src/pf/pf_pdf.c 
                    src/pf/pf_vector.c
                    src/pf/eig3.c
                    src/pf/pf_draw.c)

rospack_add_library(amcl_map
                    src/map/map.c
                    src/map/map_range.c
                    src/map/map_store.c
                    src/map/map_draw.c)

rospack_add_library(amcl_sensors
                    src/sensors/amcl_sensor.cpp
                    src/sensors/amcl_odom.cpp
                    src/sensors/amcl_laser.cpp)
target_link_libraries(amcl_sensors amcl_map amcl_pf)

rospack_add_executable(bin/amcl
                       src/amcl_node.cpp)
target_link_libraries(bin/amcl amcl_sensors amcl_map amcl_pf)

endif(NOT new_code)
