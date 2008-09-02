cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(exec_trex)


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


set(TREX_FILES Executive.cc 
	       ROSAdapter.cc 
	       ROSNode.cc 
	       Logger.cc 
               LogClock.cc 
	       CalcCommandConstraintPlayback.cc
	       CalcGlobalPathConstraintPlayback.cc 
	       CalcCommandConstraint.cc
	       Playback.cc
	       CalcGlobalPathConstraint.cc  
	       CalcArmInverseKinematicsConstraint.cc 
	       CalcInterpolatedEndEffectorPosConstraint.cc  
	       CalcGraspPositionConstraint.cc 
	       CalcAngleDiffConstraint.cc 
	       CalcDistanceConstraint.cc 
	       WavefrontPlanner.cc 
	       map_helpers.cc 
	       ExecDefs.cc 
	       Components.cc)

rospack_add_executable(exec_trex_g_rt ${TREX_FILES})
target_link_libraries(exec_trex_g_rt Utils_g TREX_g)


rospack_add_executable(exec_trex_o_rt ${TREX_FILES})
target_link_libraries(exec_trex_o_rt Utils_o TREX_o)
rospack_add_compile_flags(exec_trex_o_rt "-O3 -DEUROPA_FAST")

