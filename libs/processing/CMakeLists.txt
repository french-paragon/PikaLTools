define_pikaltools_lib(PROCESSING processing)

set(LIB_SRC scanlinecleaner.h
    texturegeneration.h
    texturegeneration.cpp
    pushbroomprojections.h
    pushbroomprojections.cpp
    darkframestools.h
    darkframestools.cpp
    relativeoffsetsestimator.h
    relativeoffsetsestimator.cpp)

add_library(
    ${PROCESSING_LIB}
    ${LIB_SRC}
)

target_link_libraries(${PROCESSING_LIB} StereoVision::stevi MultidimArrays::MultidimArrays)
