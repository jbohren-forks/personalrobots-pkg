#CMake NDDL Support. THIS IS STILL NOT A FULLY FUNCTIONAL REPLACEMENT FOR JAM.


#Set the prefix
if(NOT ROSPACK_NDDL_PATH)
    _rospack_invoke(${PROJECT_NAME} ${_prefix} ROSPACK_NDDL_PATH export --lang=nddl --attrib=iflags)
    set(ROSPACK_NDDL_PATH ${${_prefix}_ROSPACK_NDDL_PATH})
endif(NOT ROSPACK_NDDL_PATH)

#Adds an nddl directory. You should probably look at using the manifest if you are doing this.
macro(rospack_add_nddl_directory dir)
    if("${filein}" MATCHES "[ ]^/")
        set(ROSPACK_NDDL_PATH -I${dir} " " ${ROSPACK_NDDL_PATH})
    else("${filein}" MATCHES "[ ]^/")
        set(ROSPACK_NDDL_PATH -I${CMAKE_CURRENT_SOURCE_DIR}/${dir} " " ${ROSPACK_NDDL_PATH})
    endif("${filein}" MATCHES "[ ]^/")
endmacro(rospack_add_nddl_directory)


#Get the dependencies of a nddl file.
macro(nddl_depends file)
    find_ros_package(executive_trex_pr2)

    #Execute the script to get dependencies.
    execute_process(
        COMMAND ${executive_trex_pr2_PACKAGE_PATH}/bin/nddl_depends.py "-S${CMAKE_CURRENT_SOURCE_DIR}" " " ${file} " " ${ROSPACK_NDDL_PATH}
	ERROR_VARIABLE nddl_depends_error
        RESULT_VARIABLE nddl_depends_failed
	OUTPUT_VARIABLE nddl_files
	OUTPUT_STRIP_TRAILING_WHITESPACE)

    #If there is and error, complain to the user.
    if(nddl_depends_failed)
       message("ERROR in nddl_depends:")
       message("STDOUT:\n" ${nddl_files})
       message("STDERR:\n" ${nddl_depends_error})
       message(ARGS: ${executive_trex_pr2_PACKAGE_PATH}/bin/nddl_depends.py " " ${file} " " ${ROSPACK_NDDL_PATH})
       message("end")
       message(FATAL_ERROR "Failed to get NDDL deps for ${file}")
    endif(nddl_depends_failed)
    #message(${file} "\n" ${nddl_files} "\n")
endmacro(nddl_depends)


if(NOT NDDL_FILES_CREATED)
    set(NDDL_FILES_CREATED "YES")
    add_custom_target(NDDL_FILES ALL)
endif(NOT NDDL_FILES_CREATED)



#Add an nddl file to the build
macro(rospack_add_nddl file)

    #Get the dependecies of the NDDL
    nddl_depends(${file})

    #Get the NDDL compilier's package.
    find_ros_package(trex)
    
    #Write nddl cfg.
    string(REPLACE "-I" ";" nddl_file_path ${ROSPACK_NDDL_PATH})
    file(WRITE ${CMAKE_CURRENT_SOURCE_DIR}/temp_nddl_gen.cfg 
    	       "<configuration>\n"
	       " <binding nddl=\"Object\" cpp=\"Object\"/>\n"
	       " <binding nddl=\"Timeline\" cpp=\"Timeline\"/>\n"
  	       " <include path=\"${nddl_file_path}\"/>\n"
	       "</configuration>\n")


    #Convert to a list for cmake
    string(REPLACE "\n" ";" nddl_deps ${nddl_files})

    #Name the XML
    string(REPLACE ".nddl" ".xml" nddl_dependency_xml ${file})


    #Run the parser.
    add_custom_command(OUTPUT ${nddl_dependency_xml}
                       COMMAND java -jar ${trex_PACKAGE_PATH}/PLASMA/build/lib/nddl.jar --NddlParser -C "${CMAKE_CURRENT_SOURCE_DIR}" --config "temp_nddl_gen.cfg" ${file}
	               WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                       DEPENDS ${nddl_deps})




    #Create the target.
    string(REPLACE "\n" "" nddl_dependency_targ ${file})
    string(REPLACE "/" "_" nddl_dependency_targ ${nddl_dependency_targ})
    add_custom_target(${nddl_dependency_targ}
                      DEPENDS ${nddl_dependency_xml})
    add_dependencies(NDDL_FILES ${nddl_dependency_targ})
endmacro(rospack_add_nddl)

