set(EXEC_NAME bil_dataset_stats)

set(PROG_SRC main.cpp)

add_executable(
    ${EXEC_NAME}
    ${PROG_SRC}
)

#ensure the path to the submodule libraries is set

target_link_libraries(${EXEC_NAME} StereoVision::stevi ${IO_LIB} Qt${QT_VERSION_MAJOR}::Core)
