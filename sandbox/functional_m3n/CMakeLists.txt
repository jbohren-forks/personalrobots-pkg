cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rospack(functional_m3n)
rospack_add_boost_directories()

#rospack_add_executable (bin/test_fm3n examples/test_fm3n.cpp)
#target_link_libraries (bin/test_fm3n fm3n)

rospack_add_executable (bin/create_pt_cloud examples/create_pt_cloud.cpp)
target_link_libraries (bin/create_pt_cloud fm3n)

#rospack_add_link_flags(bin/create_pt_cloud "-lprofiler")

# ---[ Functional Max-Margin Markov Network library
rospack_add_library (fm3n
			src/submodular_energy_min.cpp
			src/random_field.cpp
			src/m3n_model.cpp
			src/m3n_model_inference.cpp
			src/m3n_model_learning.cpp
			src/regressors/regression_tree_wrapper.cpp
			src/m3n_params.cpp
                      )

# check for OpenMP
include(CheckIncludeFile)
include(CheckCXXCompilerFlag)
include(CheckLibraryExists)

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  # check if compilers supports -fopenmp
  check_cxx_compiler_flag (-fopenmp HAVE_OPENMP)
  check_library_exists (gomp omp_get_num_threads "" HAS_GOMP_LIB)

  if(HAVE_OPENMP AND HAS_GOMP_LIB)
    add_definitions ("-fopenmp")
#    target_link_libraries (bin/normal_estimation_node gomp)
#    target_link_libraries (bin/planar_fit_node gomp)
    set (OPENMP_LFLAGS "-lgomp")
  endif(HAVE_OPENMP AND HAS_GOMP_LIB)
endif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
