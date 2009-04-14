cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#set(ROS_BUILD_TYPE RelWithDebInfo)
rospack(amcl_player)

# Old amcl_player node, to be removed
rospack_add_executable(amcl_player amcl_player.cc)
target_link_libraries(amcl_player playerdrivers)
rospack_add_executable(cli cli.cpp)

# Library build won't work until we stop linking aginst Player, because
# libplayerdrivers defines the same symbols.  Temporary workaround is to
# build everything into one executable.

#rospack_add_library(amcl_pf 
#                    src/pf/pf.c 
#                    src/pf/pf_kdtree.c 
#                    src/pf/pf_pdf.c 
#                    src/pf/pf_vector.c
#                    src/pf/eig3.c
#                    src/pf/pf_draw.c)
#
#rospack_add_library(amcl_map
#                    src/map/map.c
#                    src/map/map_range.c
#                    src/map/map_store.c
#                    src/map/map_draw.c)
#
#rospack_add_library(amcl_sensors
#                    src/sensors/amcl_sensor.cpp
#                    src/sensors/amcl_odom.cpp
#                    src/sensors/amcl_laser.cpp)
#target_link_libraries(amcl_sensors amcl_map amcl_pf)

rospack_add_executable(bin/amcl
                       src/amcl_node.cpp
                    src/pf/pf.c 
                    src/pf/pf_kdtree.c 
                    src/pf/pf_pdf.c 
                    src/pf/pf_vector.c
                    src/pf/eig3.c
                    src/pf/pf_draw.c
                    src/map/map.c
                    src/map/map_range.c
                    src/map/map_store.c
                    src/map/map_draw.c
                    src/sensors/amcl_sensor.cpp
                    src/sensors/amcl_odom.cpp
                    src/sensors/amcl_laser.cpp)

#target_link_libraries(bin/amcl amcl_sensors amcl_map amcl_pf)

rospack_download_test_data(http://pr.willowgarage.com/data/amcl/basic_localization_stage.bag test/basic_localization_stage.bag)
rospack_download_test_data(http://pr.willowgarage.com/data/amcl/willow-full.pgm test/willow-full.pgm)

rospack_add_rostest(test/basic_localization_stage.xml)
