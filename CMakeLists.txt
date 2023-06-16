cmake_minimum_required(VERSION 3.23)

set(USE_FOLDERS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE STRING "Vcpkg toolchain file")

option(USE_UNITY_BUILDS "Enable unity build to improve build times" OFF)
option(UNIT_TESTS "Build unit tests" ON)

project(firmware)

add_subdirectory(source/common)

if (${UNIT_TESTS})
    enable_testing()
    add_subdirectory(source/tests)
endif()
