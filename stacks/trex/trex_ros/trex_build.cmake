cmake_minimum_required(VERSION 2.4.6)

# To build the executive's config.
macro(create_nddl_config)
    if(NOT ROSPACK_NDDL_PATH)
        _rospack_invoke(${PROJECT_NAME} ${_prefix} ROSPACK_NDDL_PATH export --lang=nddl --attrib=iflags)
        set(ROSPACK_NDDL_PATH ${${_prefix}_ROSPACK_NDDL_PATH})
    endif(NOT ROSPACK_NDDL_PATH)
    string(REPLACE "-I" ";" _nddl_file_path ${ROSPACK_NDDL_PATH})
    file(WRITE ${CMAKE_CURRENT_SOURCE_DIR}/temp_nddl_gen.cfg 
    	       "<configuration>\n"
	       " <binding nddl=\"Object\" cpp=\"Object\"/>\n"
	       " <binding nddl=\"Timeline\" cpp=\"Timeline\"/>\n"
  	       " <include path=\"${_nddl_file_path}\"/>\n"
	       "</configuration>\n")
endmacro(create_nddl_config)

# Set _TREX_LINK_LIBS to be the result of using rospack and splice the ends with "ender"
macro(_get_trex_link_libs ender)
    _rospack_invoke(${PROJECT_NAME} ${_prefix} ROSPACK_TREX_LIBS export --lang=trex_libs --attrib=libs)
    set(_TREX_LINK_LIBS_LIST ${${_prefix}_ROSPACK_TREX_LIBS})
    separate_arguments(_TREX_LINK_LIBS_LIST)

    set(_TREX_LINK_LIBS "")
    foreach(_TREX_LIB ${_TREX_LINK_LIBS_LIST})
        set(_TREX_LINK_LIBS "${_TREX_LINK_LIBS} ${_TREX_LIB}${ender}")
    endforeach(_TREX_LIB)

    separate_arguments(_TREX_LINK_LIBS)
    message(${_TREX_LINK_LIBS})
endmacro(_get_trex_link_libs)


# Declares the referenced executable as debug
macro(trex_declare_debug target)
  rospack_remove_compile_flags(${target} "-O3 -DEUROPA_FAST")
  _get_trex_link_libs("_g")
  message("Link debug target " ${target} " with " ${_TREX_LINK_LIBS})
  target_link_libraries(${target} ${_TREX_LINK_LIBS})
endmacro(trex_declare_debug)


# Declares the referenced executable as fast
macro(trex_declare_fast target)
  _get_trex_link_libs("_o")
  message("Link fast target " ${target} " with " ${_TREX_LINK_LIBS})
  target_link_libraries(${target} ${_TREX_LINK_LIBS})
endmacro(trex_declare_fast)

# Create a TREX library using files as args. The name of the targets created are {target}_o and {target}_g
macro(create_trex_lib target files)
  if($ENV{ROS_TREX_DEBUG} MATCHES 1)
    rospack_add_library(${target}_g ${${files}})
    trex_declare_debug(${target}_g)
  endif($ENV{ROS_TREX_DEBUG} MATCHES 1)
  rospack_add_library(${target}_o ${${files}})
  trex_declare_fast(${target}_o)
endmacro(create_trex_lib)





# A wrapper around add_executable(), using info from the rospack
# invocation to set up compiling and linking.
macro(rospack_add_executable_trex_mod exe)
  add_executable(${ARGV})

  # Add explicit dependency of each file on our manifest.xml and those of
  # our dependencies.
  # The SOURCES property seems to be available only since 2.6.  Yar.
  #get_target_property(_srclist ${exe} SOURCES) 
  set(_srclist ${ARGN})
  foreach(_src ${_srclist}) 
    # Handle the case where the second argument is EXCLUDE_FROM_ALL, not a
    # source file.  Only have to do this because we can't get the SOURCES
    # property.
    if(NOT _src STREQUAL EXCLUDE_FROM_ALL)
      find_file(file_name ${_src} ${CMAKE_CURRENT_SOURCE_DIR} /)
      add_file_dependencies(${file_name} ${ROS_MANIFEST_LIST}) 
    endif(NOT _src STREQUAL EXCLUDE_FROM_ALL)
  endforeach(_src)

  rospack_add_compile_flags(${exe} ${${PROJECT_NAME}_CFLAGS_OTHER})
  rospack_add_link_flags(${exe} ${${PROJECT_NAME}_LDFLAGS_OTHER})

  if(ROS_BUILD_STATIC_EXES AND ${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
    # This will probably only work on Linux.  The LINK_SEARCH_END_STATIC
    # property should be sufficient, but it doesn't appear to work
    # properly.
    rospack_add_link_flags(${exe} -static-libgcc -Wl,-Bstatic)
  endif(ROS_BUILD_STATIC_EXES AND ${CMAKE_SYSTEM_NAME} STREQUAL "Linux")

  target_link_libraries(${exe} ${${PROJECT_NAME}_LIBRARIES})

  # Add ROS-wide compile and link flags (usually things like -Wall).  These
  # are set in rosconfig.cmake.
  rospack_add_compile_flags(${exe} ${ROS_COMPILE_FLAGS})
  rospack_add_link_flags(${exe} ${ROS_LINK_FLAGS})

  # Make sure that any messages get generated prior to building this target
  add_dependencies(${exe} rospack_genmsg)
  add_dependencies(${exe} rospack_gensrv)

  # If we're linking boost statically, we have to force allow multiple definitions because
  # rospack does not remove duplicates
  if ("$ENV{ROS_BOOST_LINK}" STREQUAL "static")
    rospack_add_link_flags(${exe} "-Wl,--allow-multiple-definition")
  endif("$ENV{ROS_BOOST_LINK}" STREQUAL "static")

endmacro(rospack_add_executable_trex_mod)



# Create trex executables
macro(create_trex_executables fast debug)
  find_ros_package(trex_ros)
  set(file ${trex_ros_PACKAGE_PATH}/src/executable_main.cpp)

  if($ENV{ROS_TREX_DEBUG} MATCHES 1)
    message("BUILDING TREX DEBUG")
    # trexdebug builds with a large number of run-time error checking running which is expensive
    # but gives good feedback in discovering problems.
    rospack_add_executable_trex_mod(${debug} ${file})
    rospack_link_boost(${debug} thread)
    rospack_add_gtest_build_flags(${debug})
    trex_declare_debug(${debug})
  else ($ENV{ROS_TREX_DEBUG} MATCHES 1)
    message("NO TREX DEBUG")
  endif($ENV{ROS_TREX_DEBUG} MATCHES 1)




  # trexfast is about an order of magnitude faster than trexdebug
  rospack_add_executable_trex_mod(${fast} ${file})
  rospack_link_boost(${fast} thread)
  rospack_add_gtest_build_flags(${fast})
  trex_declare_fast(${fast})

  # rospack_add_gtest_build_flags excludes the target from all, on the
  # assumption that it's only used as a unit test.  In this case that's not
  # true
  set_target_properties(${fast} PROPERTIES EXCLUDE_FROM_ALL false)
endmacro(create_trex_executables)
