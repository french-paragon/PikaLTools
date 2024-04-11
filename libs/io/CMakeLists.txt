define_pikaltools_lib(IO io)

set(LIB_SRC io_globals.h
    read_trajectory_data.h
    read_trajectory_data.cpp
    read_envi_bil.h
    read_envi_bil.cpp
    read_aviris4_bin.h
    read_aviris4_bin.cpp
    georasterreader.h
    georasterreader.cpp)

add_library(
    ${IO_LIB}
    ${LIB_SRC}
)

get_target_property(SteviAppGeoIncludeDirs SteviappSdk::StereoVisionAppGeo INTERFACE_INCLUDE_DIRECTORIES)
#steviapp link to proj directly, but it lead to a conflict with gdal on certain version of ubuntu.
#to solve this we need to encapsulate the io function, which use gdal, without linking to proj.
#But we still need the includes.
target_include_directories(${IO_LIB} PUBLIC ${SteviAppGeoIncludeDirs})

target_link_libraries(${IO_LIB} PRIVATE Qt5::Core StereoVision::stevi geotiff)
