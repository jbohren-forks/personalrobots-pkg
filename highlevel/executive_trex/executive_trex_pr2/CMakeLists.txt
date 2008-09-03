cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(executive_trex_pr2)


execute_process(
    COMMAND rospack find trex
    ERROR_VARIABLE __rospack_err_ignore
    OUTPUT_VARIABLE trex_dir
    OUTPUT_STRIP_TRAILING_WHITESPACE)

if(__rospack_err_ignore)
  message(FATAL_ERROR, "Failed to invoke rospack to find trex")
endif(__rospack_err_ignore)


add_custom_target(jam_g ALL
                  COMMAND PLASMA_HOME=${trex_dir}/PLASMA/ jam -q exec_inputs
                  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})


set(TREX_FILES src/Executive.cc 
	       src/ROSAdapter.cc 
	       src/ROSNode.cc 
	       src/Logger.cc 
               src/LogClock.cc 
	       src/CalcAngleDiffConstraint.cc 
	       src/CalcDistanceConstraint.cc 
	       src/ExecDefs.cc 
	       src/Components.cc
	       src/BaseStateAdapter.cc
	       src/BaseControllerAdapter.cc)

rospack_add_executable(executive_trex_pr2_g_rt ${TREX_FILES})
target_link_libraries(executive_trex_pr2_g_rt Utils_g TREX_g)


rospack_add_executable(executive_trex_pr2_o_rt ${TREX_FILES})
target_link_libraries(executive_trex_pr2_o_rt Utils_o TREX_o)
rospack_add_compile_flags(executive_trex_pr2_o_rt "-O3 -DEUROPA_FAST")

