set(EXEC_NAME exported_ortho_viewer)

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
    Qt${QT_VERSION_MAJOR}::Core
    Qt${QT_VERSION_MAJOR}::Widgets
    ${QCUSTOMPLOT_LIBRARY})
