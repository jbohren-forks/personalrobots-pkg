set (sources cmvision.cc conversions.c CMVisionBF.cpp main.cpp)

rospack_add_executable(cmvision ${sources})
rospack_add_executable(colorgui color_gui_main.cpp color_gui.cpp cmvision.cc conversions.c)
target_link_libraries(colorgui fltk)
