set(EXEC_NAME impulse_detector)

set(PROG_SRC main.cpp)

add_executable(
    ${EXEC_NAME}
    ${PROG_SRC}
)

#ensure the path to the submodule libraries is set
target_link_directories(${EXEC_NAME} PRIVATE ${CMAKE_BINARY_DIR}/submodules/LibStevi/)
target_link_directories(${EXEC_NAME} PRIVATE ${CMAKE_BINARY_DIR}/submodules/LibStevi/submodules/qImageDisplayWidget/)

target_link_libraries(${EXEC_NAME}
    StereoVision::stevi
    StereoVision::qImageWidget
    ${IO_LIB}
    MultidimArrays::MultidimArrays
    Qt${QT_VERSION_MAJOR}::Core
    Qt${QT_VERSION_MAJOR}::Widgets
    ${QCUSTOMPLOT_LIBRARY})
