cmake_minimum_required(VERSION 3.23)

set(ENV{VCPKG_FORCE_SYSTEM_BINARIES} ON)

set(USE_FOLDERS ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

option(USE_UNITY_BUILDS "Enable unity build to improve build times" OFF)
option(ENABLE_SANITIZERS "Enable address and undefined behavior sanitizers" OFF)

project(firmware)

# Enable ASAN
add_compile_options($<$<BOOL:${ENABLE_SANITIZERS}>:-fsanitize=address>)
add_link_options($<$<BOOL:${ENABLE_SANITIZERS}>:-fsanitize=address>)

# Set warning levels
add_compile_options(-Wall -Wextra -Wpedantic)
# And remove annoying warnings
add_compile_options(-Wno-error=sign-compare)
# And treat them as error
set(CMAKE_COMPILE_WARNING_AS_ERROR ON)

if (${TRACE_BUILD_TIME})
    add_compile_options(-ftime-trace)
endif ()

add_subdirectory(source/protos)
add_subdirectory(source/common)
add_subdirectory(source/daemon)
