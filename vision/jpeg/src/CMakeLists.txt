set (encoder_sources jpeg.c encoder.cpp)
set (decoder_sources jpeg.c decoder.cpp)

rospack_add_executable(encoder ${encoder_sources})
rospack_add_executable(decoder ${decoder_sources})

target_link_libraries(encoder jpeg)
target_link_libraries(decoder jpeg)
