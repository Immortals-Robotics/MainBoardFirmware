project(${CMAKE_PROJECT_NAME}-daemon)

find_package(immortals-common CONFIG REQUIRED)
find_package(tmc-api CONFIG REQUIRED)

option(USE_MOCK_GPIO "Use a mock version of pigpio. Useful when developing on platforms other than rpi." OFF)

set(HEADER_FILES
        pch.h
        command.h
        micro.h
        motor.h)

set(C_HEADER_FILES
        spi_hal.h)

set(SOURCE_FILES
        command.cpp
        micro.cpp
        motor.cpp
        main.cpp)

set(C_SOURCE_FILES
        spi_hal.c)

if (${USE_MOCK_GPIO})
    set(C_HEADER_FILES ${C_HEADER_FILES} mock_pigpio/pigpio.h)
    set(C_SOURCE_FILES ${C_SOURCE_FILES} mock_pigpio/pigpio.c)
endif ()

add_executable(${PROJECT_NAME}
        ${C_SOURCE_FILES} ${C_HEADER_FILES}
        ${SOURCE_FILES} ${HEADER_FILES})

if (${USE_MOCK_GPIO})
    target_include_directories(${PROJECT_NAME} PRIVATE mock_pigpio)
endif ()

target_link_libraries(${PROJECT_NAME}
        immortals::common
        analogdevicesinc::tmc-api)

if (NOT ${USE_MOCK_GPIO})
    target_link_libraries(${PROJECT_NAME} pigpio)
endif ()

set_target_properties(${PROJECT_NAME} PROPERTIES UNITY_BUILD ${USE_UNITY_BUILDS})

set_source_files_properties(${C_SOURCE_FILES} PROPERTIES
        SKIP_PRECOMPILE_HEADERS TRUE
        SKIP_UNITY_BUILD_INCLUSION TRUE)

target_precompile_headers(${PROJECT_NAME} PRIVATE pch.h)

install(TARGETS ${PROJECT_NAME})
