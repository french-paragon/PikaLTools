set(EXEC_NAME aviris4_file_converter)

set(PROG_SRC main.cpp)

add_executable(
    ${EXEC_NAME}
    ${PROG_SRC}
)

#ensure the path to the submodule libraries is set

target_link_libraries(${EXEC_NAME} StereoVision::stevi ${IO_LIB} ${GUI_LIB} Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Widgets MultidimArrays::MultidimArrays)
