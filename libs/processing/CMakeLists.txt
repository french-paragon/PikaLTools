define_pikaltools_lib(PROCESSING processing)

set(LIB_SRC scanlinecleaner.h
    texturegeneration.h
    texturegeneration.cpp
    pushbroomprojections.h
    pushbroomprojections.cpp
    darkframestools.h
    darkframestools.cpp)

add_library(
    ${PROCESSING_LIB}
    ${LIB_SRC}
)
