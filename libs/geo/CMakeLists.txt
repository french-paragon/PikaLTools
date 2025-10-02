define_pikaltools_lib(GEO geo)

set(LIB_SRC coordinate_conversions.h
    coordinate_conversions.cpp
    georasterrectification.h)

add_library(
    ${GEO_LIB}
    ${LIB_SRC}
)

target_link_libraries(${GEO_LIB} SteviappSdk::StereoVisionAppGeo MultidimArrays::MultidimArrays)
