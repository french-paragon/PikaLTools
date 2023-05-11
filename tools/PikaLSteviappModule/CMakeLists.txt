set(MODULE_NAME pikal_steviapp_module)

set(MODULE_SRC
    pikalsteviappmodule.h
    pikalsteviappmodule.cpp
    datablocks/bilacquisitiondata.h
    datablocks/bilacquisitiondata.cpp
    actions/bilsequenceactionmanager.h
    actions/bilsequenceactionmanager.cpp
    actions/bilsequenceactions.h
    actions/bilsequenceactions.cpp)


add_library(
    ${MODULE_NAME} MODULE
    ${MODULE_SRC}
)

set_target_properties(${MODULE_NAME} PROPERTIES PREFIX "")

target_link_libraries(${MODULE_NAME}
    SteviappSdk::StereoVisionAppControl
    SteviappSdk::StereoVisionAppData
    SteviappSdk::stevi #stereovision lib from the Steviapp SDK
    SteviappSdk::qImageWidget #qImageWidget lib from the Steviapp SDK
    ${IO_LIB}
    Qt${QT_VERSION_MAJOR}::Core
    Qt${QT_VERSION_MAJOR}::Widgets)
