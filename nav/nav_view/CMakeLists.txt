cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(nav_view)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(wxWidgets REQUIRED)
include(${wxWidgets_USE_FILE})
include_directories( ${wxWidgets_INCLUDE_DIRS} )
include_directories( src/nav_view )

rospack_add_library(${PROJECT_NAME} src/nav_view/nav_view_panel.cpp
                                    src/nav_view/tools.cpp
                                    src/nav_view/nav_view_panel_generated.cpp)
                                    
                                    
target_link_libraries(${PROJECT_NAME} ${wxWidgets_LIBRARIES})
# swig python needs a shared library named _<modulename>.[so|dll|...]
# this renames the output file to conform to that by prepending an underscore and removing the "lib" prefix
set_target_properties(${PROJECT_NAME}
                      PROPERTIES OUTPUT_NAME _${PROJECT_NAME}
                      PREFIX "")
                                    
rospack_add_executable(nav_view_exe src/test/nav_view_test.cpp)
target_link_libraries(nav_view_exe ${PROJECT_NAME} ${wxWidgets_LIBRARIES})
set_target_properties(nav_view_exe
                      PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
