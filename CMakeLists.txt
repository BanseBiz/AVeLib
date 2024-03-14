cmake_minimum_required(VERSION 3.14.0 FATAL_ERROR)
project(avelib VERSION 0.1.1 LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 23)
set(DEFAULT_BUILD_TYPE "Release")

find_package(GeographicLib REQUIRED)
include_directories (${GeographicLib_INCLUDE_DIRS})

add_library(avelib SHARED
    src/movement.cpp
    src/vehicle.cpp
)

set_target_properties(avelib PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    PUBLIC_HEADER include/vehicle.hpp
)

configure_file(avelib.pc.in avelib.pc @ONLY)

target_include_directories(avelib PRIVATE include)
target_include_directories(avelib PRIVATE src)

include(GNUInstallDirs)
install(TARGETS avelib
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(FILES ${CMAKE_BINARY_DIR}/avelib.pc
    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/pkgconfig
)