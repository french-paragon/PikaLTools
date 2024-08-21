set(EXEC_NAME bil_preview_generator)

set(PROG_SRC main.cpp)

add_executable(
    ${EXEC_NAME}
    ${PROG_SRC}
)

#ensure the path to the submodule libraries is set

target_link_libraries(${EXEC_NAME}
    StereoVision::stevi ${IO_LIB}
    JPEG::JPEG
    TIFF::TIFF
    PNG::PNG
    Qt${QT_VERSION_MAJOR}::Core )
