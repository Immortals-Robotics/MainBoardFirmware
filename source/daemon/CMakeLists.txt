project(${CMAKE_PROJECT_NAME}-daemon)

option(USE_MOCK_GPIO "Use a mock version of pigpio. Useful when developing on platforms other than rpi." OFF)

set(TMC_DIR "../tmc-api")

set(HEADER_FILES
        pch.h
        command.h
        micro.h
        motor.h)

set(C_HEADER_FILES
        spi_hal.h
        ${TMC_DIR}/tmc/helpers/API_Header.h
        ${TMC_DIR}/tmc/helpers/Bits.h
        ${TMC_DIR}/tmc/helpers/Config.h
        ${TMC_DIR}/tmc/helpers/Constants.h
        ${TMC_DIR}/tmc/helpers/CRC.h
        ${TMC_DIR}/tmc/helpers/Functions.h
        ${TMC_DIR}/tmc/helpers/Macros.h
        ${TMC_DIR}/tmc/helpers/RegisterAccess.h
        ${TMC_DIR}/tmc/helpers/Types.h
        ${TMC_DIR}/tmc/ic/TMC4671/TMC4671.h
        ${TMC_DIR}/tmc/ic/TMC4671/TMC4671_Constants.h
        ${TMC_DIR}/tmc/ic/TMC4671/TMC4671_Fields.h
        ${TMC_DIR}/tmc/ic/TMC4671/TMC4671_Register.h
        ${TMC_DIR}/tmc/ic/TMC4671/TMC4671_Variants.h
        ${TMC_DIR}/tmc/ic/TMC6200/TMC6200.h
        ${TMC_DIR}/tmc/ic/TMC6200/TMC6200_Constants.h
        ${TMC_DIR}/tmc/ic/TMC6200/TMC6200_Fields.h
        ${TMC_DIR}/tmc/ic/TMC6200/TMC6200_Register.h)

set(SOURCE_FILES
        command.cpp
        micro.cpp
        motor.cpp
        main.cpp)

set(C_SOURCE_FILES
        spi_hal.c
        ${TMC_DIR}/tmc/helpers/CRC.c
        ${TMC_DIR}/tmc/helpers/Functions.c
        ${TMC_DIR}/tmc/ic/TMC4671/TMC4671.c
        ${TMC_DIR}/tmc/ic/TMC6200/TMC6200.c)

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

target_include_directories(${PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/${TMC_DIR}")

target_link_libraries(${PROJECT_NAME} ${CMAKE_PROJECT_NAME}-common)

if (NOT ${USE_MOCK_GPIO})
    target_link_libraries(${PROJECT_NAME} pigpio)
endif ()

set_target_properties(${PROJECT_NAME} PROPERTIES UNITY_BUILD ${USE_UNITY_BUILDS})

set_source_files_properties(${C_SOURCE_FILES} PROPERTIES
        SKIP_PRECOMPILE_HEADERS TRUE
        SKIP_UNITY_BUILD_INCLUSION TRUE)

get_target_property(COMMON_PCHS ${CMAKE_PROJECT_NAME}-common INTERFACE_PRECOMPILE_HEADERS)

target_precompile_headers(${PROJECT_NAME}
        PUBLIC ${COMMON_PCHS}
        PUBLIC pch.h)

install(TARGETS ${PROJECT_NAME})
