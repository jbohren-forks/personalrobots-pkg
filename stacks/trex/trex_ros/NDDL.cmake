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


