FIND_PACKAGE(Boost REQUIRED thread)



rospack_add_executable(power_node power_node.cpp)
target_link_libraries(power_node ${Boost_LIBRARIES} )
