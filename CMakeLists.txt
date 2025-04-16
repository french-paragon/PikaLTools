cmake_minimum_required(VERSION 3.5)

project(PikaLTools LANGUAGES CXX)

option(BUILD_TESTS "Build the test suite" ON)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DMULTIDIMARRAYS_STRONG_BOUND_CHECK")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -DMULTIDIMARRAYS_STRONG_BOUND_CHECK")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fopenmp -march=native -ftree-vectorize")

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libs)

find_package(QT NAMES Qt5 COMPONENTS Core Test Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Core Test Widgets REQUIRED)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_library(QCUSTOMPLOT_LIBRARY NAMES qcustomplot QCustomPlot)

if (NOT QCUSTOMPLOT_LIBRARY)
    message(SEND_ERROR "QCustomPlot not found")
endif()

find_package(JPEG REQUIRED)
find_package(PNG REQUIRED)
find_package(TIFF REQUIRED)

find_package(StereoVision REQUIRED)

find_package(PkgConfig REQUIRED)

pkg_check_modules(PROJ REQUIRED IMPORTED_TARGET proj)
add_library(PROJ::proj ALIAS PkgConfig::PROJ)


find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

find_package(Eigen3 3.3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(SteviappSdk)

if(${SteviappSdk_FOUND})
    message("Found steviapp sdk")
endif()

add_subdirectory(libs)

add_subdirectory(tools)

if (BUILD_TESTS)
        add_subdirectory(tests)
endif (BUILD_TESTS)
