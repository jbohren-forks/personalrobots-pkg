cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(filter_coefficient_server)
genmsg()
gensrv()
add_subdirectory(test)

rospack_add_library(filter_coefficient_server
                    src/filter_coeff_client.cpp)

rospack_add_executable(filter_coeff_client
                       src/filter_coeff_client.cpp)

                      

