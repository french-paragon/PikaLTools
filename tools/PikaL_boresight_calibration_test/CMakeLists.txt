set(EXEC_NAME pika_l_boresight_calibration_test)

set(PROG_SRC main.cpp)

add_executable(
    ${EXEC_NAME}
    ${PROG_SRC}
)

target_link_libraries(${EXEC_NAME}
    StereoVision::stevi
    StereoVision::qImageWidget
    ${IO_LIB}
    ${GEO_LIB}
    MultidimArrays::MultidimArrays
    Qt${QT_VERSION_MAJOR}::Core
    )
