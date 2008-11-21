rospack_add_executable(avoid_monster avoid_monster.cpp)
set_target_properties(avoid_monster PROPERTIES
                      RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
rospack_add_gtest_build_flags(avoid_monster)

rospack_add_rostest_future(${CMAKE_CURRENT_SOURCE_DIR}/avoid_monster.xml)
