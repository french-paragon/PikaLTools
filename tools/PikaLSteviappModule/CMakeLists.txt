set(MODULE_NAME pikal_steviapp_module)

set(MODULE_SRC
    pikalsteviappmodule.h
    pikalsteviappmodule.cpp
    datablocks/bilacquisitiondata.h
    datablocks/bilacquisitiondata.cpp
    datablocks/inputdtm.h
    datablocks/inputdtm.cpp
    actions/bilsequenceactionmanager.h
    actions/bilsequenceactionmanager.cpp
    actions/bilsequenceactions.h
    actions/bilsequenceactions.cpp
    actions/inputdtmactionmanager.h
    actions/inputdtmactionmanager.cpp
    actions/inputdtmactions.h
    actions/inputdtmactions.cpp
    gui/opengldrawables/opengldrawabletrajectory.h
    gui/opengldrawables/opengldrawabletrajectory.cpp
    gui/opengldrawables/opengldrawabledtm.h
    gui/opengldrawables/opengldrawabledtm.cpp
    gui/trajectoryvieweditor.h
    gui/trajectoryvieweditor.cpp
    gui/bilcubevieweditor.h
    gui/bilcubevieweditor.cpp
    gui/imageOverlays/bilsequencelandmarksoverlay.h
    gui/imageOverlays/bilsequencelandmarksoverlay.cpp
    gui/shaders.qrc)


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
