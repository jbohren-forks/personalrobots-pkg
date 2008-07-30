cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(sbpl)
include_directories(${PROJECT_SOURCE_DIR}/code)
rospack_add_library(sbpl 
                         src/discrete_space_information/nav2d/environment_nav2D.cpp
                         src/discrete_space_information/template/environment_XXX.cpp
                         src/planners/ARAStar/araplanner.cpp
                         src/planners/VI/viplanner.cpp
                         src/utils/heap.cpp
                         src/utils/mdp.cpp
                         src/utils/utils.cpp
			 )
rospack_add_executable(bin/test_2dnav src/test/main.cpp)
target_link_libraries(bin/test_2dnav sbpl)

