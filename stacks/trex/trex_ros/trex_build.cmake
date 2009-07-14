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
  rospack_add_library(${target}_g ${TREX_FILES})
  trex_declare_debug(${target}_g)
  rospack_add_library(${target}_o ${TREX_FILES})
  trex_declare_fast(${target}_o)
endmacro(create_trex_lib)

