rospack_add_executable(power_node power_node.cpp)

rospack_add_boost_directories()
rospack_link_boost(power_node thread)
