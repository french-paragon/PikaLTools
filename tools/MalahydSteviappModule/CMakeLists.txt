set(MODULE_NAME pikal_steviapp_module)

set(MODULE_SRC
    pikalsteviappmodule.h
    pikalsteviappmodule.cpp
    datablocks/bilacquisitiondata.h
    datablocks/bilacquisitiondata.cpp
    datablocks/inputdtm.h
    datablocks/inputdtm.cpp
    datablocks/comparisontrajectory.h
    datablocks/comparisontrajectory.cpp
    actions/bilsequenceactionmanager.h
    actions/bilsequenceactionmanager.cpp
    actions/bilsequenceactions.h
    actions/bilsequenceactions.cpp
    actions/inputdtmactionmanager.h
    actions/inputdtmactionmanager.cpp
    actions/inputdtmactions.h
    actions/inputdtmactions.cpp
    actions/comparisonsequenceactionmanager.h
    actions/comparisonsequenceactionmanager.cpp
    actions/comparisonsequenceactions.h
    actions/comparisonsequenceactions.cpp
    processing/rectifybilseqtoorthosteppedprocess.h
    processing/rectifybilseqtoorthosteppedprocess.cpp
    gui/opengldrawables/opengldrawabletrajectory.h
    gui/opengldrawables/opengldrawabletrajectory.cpp
    gui/opengldrawables/opengldrawabledtm.h
    gui/opengldrawables/opengldrawabledtm.cpp
    gui/trajectoryvieweditor.h
    gui/trajectoryvieweditor.cpp
    gui/bilcubevieweditor.h
    gui/bilcubevieweditor.cpp
    gui/dtmrastervieweditor.h
    gui/dtmrastervieweditor.cpp
    gui/imageOverlays/bilsequencelandmarksoverlay.h
    gui/imageOverlays/bilsequencelandmarksoverlay.cpp
    gui/imageOverlays/inputdtmlandmarksoverlay.h
    gui/imageOverlays/inputdtmlandmarksoverlay.cpp
    gui/exportorthophotooptionsdialog.h
    gui/exportorthophotooptionsdialog.cpp
    gui/exportorthophotooptionsdialog.ui
    gui/shaders.qrc
    gui/datacolumnsselectionwidget.h
    gui/datacolumnsselectionwidget.cpp
    gui/pushbroomoptimizationconfigdialog.h
    gui/pushbroomoptimizationconfigdialog.cpp
    gui/simulatepushbroomtiepointsoptiondialog.h
    gui/simulatepushbroomtiepointsoptiondialog.cpp
    solving/bilsequencesbamodule.h
    solving/bilsequencesbamodule.cpp
    solving/dtmtiepointsmodule.h
    solving/dtmtiepointsmodule.cpp
    solving/cost_functors/pinholepushbroomuvprojector.h
    solving/cost_functors/pinholepushbroomuvprojector.cpp
    solving/cost_functors/parametrizedxyz2pushbroom.h
    solving/cost_functors/parametrizedxyz2pushbroom.cpp
    solving/cost_functors/orientationspeedconsistency.h
    solving/cost_functors/orientationspeedconsistency.cpp)


add_library(
    ${MODULE_NAME} MODULE
    ${MODULE_SRC}
    gui/prerectifiedbillvieweditor.h gui/prerectifiedbillvieweditor.cpp
)

#ensure the path to the submodule libraries is set
target_link_directories(${MODULE_NAME} PRIVATE ${CMAKE_BINARY_DIR}/submodules/LibStevi/)
target_link_directories(${MODULE_NAME} PRIVATE ${CMAKE_BINARY_DIR}/submodules/LibStevi/submodules/qImageDisplayWidget/)

set_target_properties(${MODULE_NAME} PROPERTIES PREFIX "")
set_target_properties(${MODULE_NAME} PROPERTIES DEBUG_POSTFIX "_d")

target_link_libraries(${MODULE_NAME}
    SteviappSdk::StereoVisionAppControl
    SteviappSdk::StereoVisionAppData
    StereoVision::stevi #stereovision lib from the Steviapp SDK
    StereoVision::qImageWidget #qImageWidget lib from the Steviapp SDK
    ${IO_LIB}
    ${PROCESSING_LIB}
    MultidimArrays::MultidimArrays
    Qt${QT_VERSION_MAJOR}::Core
    Qt${QT_VERSION_MAJOR}::Widgets)

if (BUILD_TESTS)

    add_executable(testMalahydModuleOpt tests/test_malahyd_module_opt.cpp

        solving/bilsequencesbamodule.h
        solving/bilsequencesbamodule.cpp
        solving/dtmtiepointsmodule.h
        solving/dtmtiepointsmodule.cpp
        solving/cost_functors/pinholepushbroomuvprojector.h
        solving/cost_functors/pinholepushbroomuvprojector.cpp
        solving/cost_functors/parametrizedxyz2pushbroom.h
        solving/cost_functors/parametrizedxyz2pushbroom.cpp
        solving/cost_functors/orientationspeedconsistency.h
        solving/cost_functors/orientationspeedconsistency.cpp

        datablocks/bilacquisitiondata.h
        datablocks/bilacquisitiondata.cpp
        datablocks/inputdtm.h
        datablocks/inputdtm.cpp
        datablocks/comparisontrajectory.h
        datablocks/comparisontrajectory.cpp
    )

    target_link_libraries(testMalahydModuleOpt
        SteviappSdk::StereoVisionAppControl
        SteviappSdk::StereoVisionAppData
        SteviappSdk::StereoVisionAppGui
        SteviappSdk::StereoVisionAppSolving
        SteviappSdk::StereoVisionAppProcessing
        SteviappSdk::StereoVisionAppTestUtils
        StereoVision::stevi #stereovision lib from the Steviapp SDK
        StereoVision::qImageWidget #qImageWidget lib from the Steviapp SDK
        ${IO_LIB}
        ${PROCESSING_LIB}
        MultidimArrays::MultidimArrays
        Ceres::ceres
        PROJ::proj
        Qt${QT_VERSION_MAJOR}::Core
        Qt${QT_VERSION_MAJOR}::Widgets
        Qt${QT_VERSION_MAJOR}::Test
    )

    add_test(TestMalahydModuleOpt testMalahydModuleOpt)

endif (BUILD_TESTS)
