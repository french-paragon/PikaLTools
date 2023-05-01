
set(PIKALTOOLS_LIB_PREFIX PikaLTools)

macro(add_pikaltools_libdirectory directory)
    add_subdirectory(${directory})
    set(${lib_var_name} ${lib_name} PARENT_SCOPE)
endmacro()

macro(define_pikaltools_lib varname name)
    set(lib_var_name ${varname}_LIB PARENT_SCOPE)
    set(lib_name ${PIKALTOOLS_LIB_PREFIX}${name} PARENT_SCOPE)
    set(${varname}_LIB ${PIKALTOOLS_LIB_PREFIX}${name} PARENT_SCOPE)
    set(${varname}_LIB ${PIKALTOOLS_LIB_PREFIX}${name})
endmacro()

add_pikaltools_libdirectory(io)
add_pikaltools_libdirectory(geo)
add_pikaltools_libdirectory(gui)
