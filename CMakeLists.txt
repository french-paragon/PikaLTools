cmake_minimum_required(VERSION 3.5)

project(PikaLTools LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fopenmp -march=native -ftree-vectorize")

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libs)

find_package(QT NAMES Qt6 Qt5 COMPONENTS Core Test Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Core Test Widgets REQUIRED)

set(CMAKE_AUTOMOC ON)

find_library(QCUSTOMPLOT_LIBRARY NAMES qcustomplot)

if (NOT QCUSTOMPLOT_LIBRARY)
    message(SEND_ERROR "QCustomPlot not found")
endif()

find_package(Eigen3 3.3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_subdirectory(submodules)
include_directories(${MODULES_INCLUDE_DIRECTORIES})
link_directories(${MODULES_LINK_DIRECTORIES})

add_subdirectory(libs)

add_subdirectory(tools)
