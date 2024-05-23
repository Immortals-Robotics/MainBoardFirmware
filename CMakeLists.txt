cmake_minimum_required(VERSION 3.23)

set(USE_FOLDERS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(ENV{VCPKG_FORCE_SYSTEM_BINARIES} ON)

option(USE_UNITY_BUILDS "Enable unity build to improve build times" OFF)

project(firmware)

add_subdirectory(source)
