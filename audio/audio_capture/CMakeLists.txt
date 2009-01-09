cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(audio_capture)

rospack_add_executable(audio_capture audio_capture.cpp)
target_link_libraries(audio_capture portaudio)

rospack_add_executable(audio_clip_writer audio_clip_writer.cpp)
target_link_libraries(audio_clip_writer sndfile)

