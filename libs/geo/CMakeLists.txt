define_pikaltools_lib(GEO geo)

set(LIB_SRC coordinate_conversions.h
    coordinate_conversions.cpp)

add_library(
    ${GEO_LIB}
    ${LIB_SRC}
)
