project(${CMAKE_PROJECT_NAME}-bin)

find_package(asio CONFIG REQUIRED)
find_package(Protobuf CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(tomlplusplus REQUIRED IMPORTED_TARGET tomlplusplus)

set(PROTOS_DIR "protos")
set(TMC_DIR "tmc-api")

add_subdirectory(${PROTOS_DIR})

set(HEADER_FILES
        pch.h
        services.h
        setting.h
        config/config.h
        logging/logging.h
        logging/macros.h
        network/udp_client.h
        network/udp_server.h
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
        main.cpp
        config/config.cpp
        logging/logging.cpp
        network/udp_client.cpp
        network/udp_server.cpp)

set(C_SOURCE_FILES
        spi_hal.c
        ${TMC_DIR}/tmc/helpers/CRC.c
        ${TMC_DIR}/tmc/helpers/Functions.c
        ${TMC_DIR}/tmc/ic/TMC4671/TMC4671.c
        ${TMC_DIR}/tmc/ic/TMC6200/TMC6200.c)

add_executable(${PROJECT_NAME}
        ${C_SOURCE_FILES} ${C_HEADER_FILES}
        ${SOURCE_FILES} ${HEADER_FILES})

target_include_directories(${PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/${PROTOS_DIR}")
target_include_directories(${PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/${TMC_DIR}")

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SOURCE_FILES} ${C_SOURCE_FILES} ${HEADER_FILES} ${C_HEADER_FILES})

target_link_libraries(${PROJECT_NAME}
        ${CMAKE_PROJECT_NAME}-protos
        pigpiod_if2
        protobuf::libprotobuf
        asio::asio
        PkgConfig::tomlplusplus
        spdlog::spdlog)

option(TRACE_BUILD_TIME "Use -ftime-trace to generate build time trace" OFF)
if (${TRACE_BUILD_TIME})
    if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "[Cc]lang")
        target_compile_options(${PROJECT_NAME} PUBLIC -ftime-trace)
    endif ()
endif ()

target_compile_definitions(${PROJECT_NAME} PUBLIC DATA_DIR="${CMAKE_SOURCE_DIR}/data")
target_compile_definitions(${PROJECT_NAME} PUBLIC LOG_DIR="${CMAKE_SOURCE_DIR}/log")

set_source_files_properties(${PROTO_SRCS} PROPERTIES SKIP_UNITY_BUILD_INCLUSION TRUE)
set_target_properties(${PROJECT_NAME} PROPERTIES UNITY_BUILD ${USE_UNITY_BUILDS})

set_source_files_properties(${C_SOURCE_FILES} PROPERTIES
        SKIP_PRECOMPILE_HEADERS TRUE
        SKIP_UNITY_BUILD_INCLUSION TRUE)

get_target_property(PROTOS_PCHS ${CMAKE_PROJECT_NAME}-protos INTERFACE_PRECOMPILE_HEADERS)

target_precompile_headers(${PROJECT_NAME}
        PUBLIC ${PROTOS_PCHS}
        PUBLIC pch.h)
