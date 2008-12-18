cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(filter_coefficient_server)
genmsg()
gensrv()

rospack_add_library(filter_coefficient_server
                    src/generate_filter_coeffs_client.cpp)

rospack_add_executable(generate_filter_coeffs_client src/generate_filter_coeffs_client.cpp)
target_link_libraries(generate_filter_coeffs_client filter_coefficient_server)
