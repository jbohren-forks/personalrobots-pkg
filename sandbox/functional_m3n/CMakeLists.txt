cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rospack(functional_m3n)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rospack_add_boost_directories()

rospack_add_executable (classify_m3n
                        examples/classify_m3n.cpp)
target_link_libraries (classify_m3n ${PROJECT_NAME})

rospack_add_executable (train_m3n
                        examples/train_m3n.cpp)
target_link_libraries (train_m3n ${PROJECT_NAME})

#rospack_add_link_flags(create_pt_cloud "-lprofiler")

# ---[ Functional Max-Margin Markov Network library
rospack_add_library (${PROJECT_NAME}
                        src/inference_tools/vk_submodular_energy_min.cpp
			src/inference_tools/bgl_submodular_energy_min.cpp
			src/random_field.cpp
			src/m3n_model.cpp
			src/m3n_model_inference.cpp
			src/m3n_model_learning.cpp
			src/regressors/regression_tree_wrapper.cpp
			src/m3n_params.cpp
			src/example/pt_cloud_rf_creator.cpp
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
