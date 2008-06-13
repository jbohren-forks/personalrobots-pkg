cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

###############################################################################
# Macro to turn a list into a string (why doesn't CMake have this
# built-in?)
MACRO (LIST_TO_STRING _string _list)
  SET (${_string})
  FOREACH (_item ${_list})
    SET (${_string} "${${_string}} ${_item}")
  ENDFOREACH (_item)
ENDMACRO (LIST_TO_STRING)


include(FindPkgConfig)
# pkg_check_modules is defined in the FindPkgConfig module
pkg_check_modules(GDKPIXBUF REQUIRED gdk-pixbuf-2.0)

set(cflags ${GDKPIXBUF_CFLAGS})
set(lflags ${GDKPIXBUF_LDFLAGS})

set(pkg map_server)
set(exe map_server)
set(srcs map_server.cc)
set(rospackexe rospack)

execute_process(COMMAND ${rospackexe} export/cpp/cflags ${pkg}
                OUTPUT_VARIABLE rospackCflags
                RESULT_VARIABLE rospack_failed
                OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND ${rospackexe} export/cpp/lflags ${pkg}
                OUTPUT_VARIABLE rospackLflags
                RESULT_VARIABLE rospack_failed
                OUTPUT_STRIP_TRAILING_WHITESPACE)
if(${rospack_failed})
 message(SEND_ERROR "rospack failed")
endif(${rospack_failed})

set(cflags "${cflags} ${rospackCflags}")
set(lflags "${lflags} ${rospackLflags}")

LIST_TO_STRING(cflags "${cflags}")
LIST_TO_STRING(lflags "${lflags}")


add_executable(${exe} ${srcs})
set_target_properties(${exe} 
                      PROPERTIES COMPILE_FLAGS "${cflags}")
set_target_properties(${exe} 
                      PROPERTIES LINK_FLAGS "${lflags}")
