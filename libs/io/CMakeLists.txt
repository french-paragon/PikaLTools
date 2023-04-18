define_pikaltools_lib(IO io)

set(LIB_SRC io_globals.h
    read_envi_bil.h
    read_envi_bil.cpp)

add_library(
    ${IO_LIB}
    ${LIB_SRC}
)
