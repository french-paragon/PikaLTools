set(EXEC_NAME bil_rectifier)

set(PROG_SRC main.cpp)

add_executable(
    ${EXEC_NAME}
    ${PROG_SRC}
)

#ensure the path to the submodule libraries is set

target_link_libraries(${EXEC_NAME}
    StereoVision::stevi
    SteviappSdk::StereoVisionAppGeo
    SteviappSdk::StereoVisionAppVision
    SteviappSdk::StereoVisionAppData
    ${IO_LIB} ${GEO_LIB}
    PROJ::proj
    JPEG::JPEG
    TIFF::TIFF
    PNG::PNG
    Qt${QT_VERSION_MAJOR}::Core
    Qt${QT_VERSION_MAJOR}::Widgets)
