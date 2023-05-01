set(EXEC_NAME patch_extractor)

set(PROG_SRC main.cpp)

add_executable(
    ${EXEC_NAME}
    ${PROG_SRC}
)

target_link_libraries(${EXEC_NAME}
    ${STEREOVISION_LIB}
    ${QIMAGEDISPLAYWIDGET_LIB}
    ${IO_LIB}
    Qt${QT_VERSION_MAJOR}::Core
    Qt${QT_VERSION_MAJOR}::Widgets)
