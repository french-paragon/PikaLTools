define_pikaltools_lib(GUI gui)

set(LIB_SRC nextandpreviousframeeventfilter.h
    nextandpreviousframeeventfilter.cpp
    hyperspectralslicedisplayadapter.h
    hyperspectralsimplepseudocolordisplayadapter.h)

add_library(
    ${GUI_LIB}
    ${LIB_SRC}
)

target_link_libraries(${GUI_LIB} Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Widgets MultidimArrays::MultidimArrays)
