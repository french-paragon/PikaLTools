cmake_minimum_required(VERSION 3.5)

project(PikaLTools LANGUAGES CXX)

include(FetchContent)

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

#find dependancies

find_package(SteviappSdk)

if(${SteviappSdk_FOUND})
    message("Found steviapp sdk")
else()
    message("Steviapp sdk not found!")
endif()

FetchContent_Declare(
  MultidimArrays
  GIT_REPOSITORY https://github.com/french-paragon/MultidimArrays.git
  GIT_TAG 78d674a3a3713feee4ced37a95fef3b53cc2832b
  FIND_PACKAGE_ARGS
)
FetchContent_MakeAvailable(MultidimArrays)

FetchContent_Declare(
  StereoVision
  GIT_REPOSITORY https://github.com/french-paragon/LibStevi.git
  GIT_TAG 28da8b530013fb49799a8cb270a8b0f558d2b4c3
  FIND_PACKAGE_ARGS
)
set(buildGui ON) #ensuire gui is built for stereo vision
FetchContent_MakeAvailable(StereoVision)

find_package(QT NAMES Qt5 COMPONENTS Core Test Widgets PrintSupport REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Core Test Widgets PrintSupport REQUIRED)

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

find_package(PkgConfig REQUIRED)

pkg_check_modules(PROJ REQUIRED IMPORTED_TARGET proj)
add_library(PROJ::proj ALIAS PkgConfig::PROJ)


find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

find_package(Eigen3 3.3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_subdirectory(libs)

add_subdirectory(tools)

if (BUILD_TESTS)
        add_subdirectory(tests)
endif (BUILD_TESTS)
