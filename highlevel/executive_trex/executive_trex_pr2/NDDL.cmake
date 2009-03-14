#CMake NDDL Support. THIS IS STILL NOT A FULLY FUNCTIONAL REPLACEMENT FOR JAM.


#Set the prefix
_rospack_invoke(${PROJECT_NAME} ${_prefix} ROSPACK_NDDL_PATH export --lang=nddl --attrib=iflags)
set(ROSPACK_NDDL_PATH ${${_prefix}_ROSPACK_NDDL_PATH})

macro(rospack_add_nddl_directory dir)
    if("${filein}" MATCHES "^/")
        set(ROSPACK_NDDL_PATH -I${dir} " " ${ROSPACK_NDDL_PATH})
    else("${filein}" MATCHES "^/")
        set(ROSPACK_NDDL_PATH -I${CMAKE_CURRENT_SOURCE_DIR}/${dir} " " ${ROSPACK_NDDL_PATH})
    endif("${filein}" MATCHES "^/")
endmacro(rospack_add_nddl_directory)


#Get the dependencies of a nddl file.
macro(nddl_depends filein)
    find_ros_package(executive_trex_pr2)

    #If it is absolute, do nothing. Otherwise, add the current source dir.
    if("${filein}" MATCHES "^/")
        set(file ${filein})
    else("${filein}" MATCHES "^/")
        set(file ${CMAKE_CURRENT_SOURCE_DIR}/${filein})
    endif("${filein}" MATCHES "^/")
    
    #Execute the script to get dependencies.
    execute_process(
        COMMAND ${executive_trex_pr2_PACKAGE_PATH}/bin/nddl_depends.py " " ${file} " " ${ROSPACK_NDDL_PATH}
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
       message(FATAL_ERROR "Failed to get NDDL deps")
    endif(nddl_depends_failed)
endmacro(nddl_depends)

#Add an nddl file to the build
macro(rospack_add_nddl file)
    nddl_depends(${file})
    find_ros_package(trex)
    
    #Write nddl cfg.
    string(REPLACE "-I" ";" nddl_file_path ${ROSPACK_NDDL_PATH})
    file(WRITE ${CMAKE_CURRENT_SOURCE_DIR}/temp_nddl_gen.cfg 
    	       "<configuration>\n"
	       " <binding nddl=\"Object\" cpp=\"Object\"/>\n"
	       " <binding nddl=\"Timeline\" cpp=\"Timeline\"/>\n"
  	       " <include path=\"${nddl_file_path}\"/>\n"
	       "</configuration>\n")



    string(REPLACE "\n" ";" nddl_deps ${nddl_files})

    #Iterate over the dependencies of this file, adding each.
    foreach(sfile ${nddl_deps})
        #message(${sfile})
	
	#Get the dependencies of this file. Note that there is no way to have a dependency here that is not in nddl_deps
        nddl_depends(${sfile})

	#Replace the newlines with ; for cmake, and add "xml" to the end of every file name.
	set(nddl_files_pure ${nddl_files})

	if(nddl_files)
	    string(REPLACE "${sfile}\n" "" nddl_files ${nddl_files})
	endif(nddl_files)
	if(nddl_files)
	    string(REPLACE "${sfile}" "" nddl_files ${nddl_files})
	endif(nddl_files)
	if(nddl_files)
            string(REPLACE "\n" ".xml;" nddl_files ${nddl_files})
	endif(nddl_files)
	if(nddl_files)
            set(nddl_files "${nddl_files}.xml")
	endif(nddl_files)

	#foreach(dfile ${nddl_files})
	#    message("\t"${dfile})
	#endforeach(dfile)


        string(REPLACE "\n" ";" nddl_files_pure ${nddl_files_pure})
        set(nddl_files "${nddl_files};${nddl_files_pure}")

	#Run the NDDL parse.
        add_custom_command(OUTPUT ${sfile}.xml
		          COMMAND pwd
			  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                          COMMAND java -jar ${trex_PACKAGE_PATH}/PLASMA/build/lib/nddl.jar --NddlParser -C "${CMAKE_CURRENT_SOURCE_DIR}" --config "temp_nddl_gen.cfg" ${sfile}
			  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                          DEPENDS ${nddl_files})

	#Create the target. Need to remove and \n or / in its name.
	string(REPLACE "\n" "" sfiletarg ${sfile})
	string(REPLACE "/" "_" sfiletarg ${sfiletarg})
  	add_custom_target(${sfiletarg} ALL
        		 DEPENDS ${sfile}.xml ${sfile})
        #message(${sfile} "   ->   " ${sfiletarg})
    endforeach(sfile)
endmacro(rospack_add_nddl)




