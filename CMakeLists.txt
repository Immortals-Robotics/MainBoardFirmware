cmake_minimum_required(VERSION 3.23)

set(USE_FOLDERS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(ENV{VCPKG_FORCE_SYSTEM_BINARIES} ON)

set(CMAKE_TOOLCHAIN_FILE "~/dev/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE STRING "Vcpkg toolchain file")

option(USE_UNITY_BUILDS "Enable unity build to improve build times" OFF)
option(UNIT_TESTS "Build unit tests" ON)

project(firmware)

add_subdirectory(source)

if (${UNIT_TESTS})
    enable_testing()
    add_subdirectory(tests)
endif()
