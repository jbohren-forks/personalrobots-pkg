set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/example)

rospack_add_executable(testrecv  testrecv.cpp)
target_link_libraries(testrecv messageseq)
rospack_add_executable(testgen   testgen.cpp)
rospack_add_executable(testdelay testdelay.cpp)
