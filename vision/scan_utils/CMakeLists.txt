cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(scan_utils)
add_definitions(-Wall)
genmsg()

# Where's a better place to get this list of libraries?
set(vtk_libs vtkRendering vtkGraphics vtkImaging vtkIO vtkFiltering vtkCommon vtksys pthread dl m vtkDICOMParser vtkftgl vtkHybrid vtkFiltering vtkGraphics)

rospack_add_library(scanutils src/smartScan.cpp src/dataTypes.cpp src/intersection_triangle.cpp src/intersection_obb.cpp src/intersection_sphere.cpp)
target_link_libraries(scanutils ${vtk_libs})
rospack_add_library(listennode src/listen_node/scanListenNode.cpp src/listen_node/rosSystemCalls.cpp)
target_link_libraries(listennode ${vtk_libs} scanutils)

rospack_add_executable(cloudToOctree src/cloudToOctree/cloudToOctree.cpp)
target_link_libraries(cloudToOctree scanutils ${vtk_libs})

rospack_add_gtest(testOctree test/testOctree.cpp)
target_link_libraries(testOctree scanutils ${vtk_libs})
set_target_properties(testOctree PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
