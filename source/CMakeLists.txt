project(${CMAKE_PROJECT_NAME}-bin)

include("${CMAKE_SOURCE_DIR}/cmake/helpers.cmake")

set_project_paths()
enable_parallel_build()

find_package(asio CONFIG REQUIRED)
find_package(Protobuf CONFIG REQUIRED)
find_package(tomlplusplus CONFIG REQUIRED)
find_package(quill CONFIG REQUIRED)

set(PROTOS_DIR "protos")
set(TMC_DIR "tmc-api")

add_subdirectory(${PROTOS_DIR})
add_protobuf_generate_command(PROTO_FILES PROTO_SRCS PROTO_HDRS)

set(HEADER_FILES
    pch.h
    services.h
    setting.h
    config/config.h
    logging/logging.h
    network/udp_client.h
    network/udp_server.h
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
    main.cpp
    config/config.cpp
    logging/logging.cpp
    network/udp_client.cpp
    network/udp_server.cpp)

set(C_SOURCE_FILES
    ${TMC_DIR}/tmc/helpers/CRC.c
    ${TMC_DIR}/tmc/helpers/Functions.c
    ${TMC_DIR}/tmc/ic/TMC4671/TMC4671.c
    ${TMC_DIR}/tmc/ic/TMC6200/TMC6200.c)

add_executable(${PROJECT_NAME} 
    ${SOURCE_FILES} ${C_SOURCE_FILES} ${HEADER_FILES}
    ${PROTO_SRCS} ${PROTO_HDRS}
    ${PROTO_FILES})

target_include_directories(${PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/${PROTOS_DIR}")
target_include_directories(${PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/${TMC_DIR}")

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SOURCE_FILES} ${C_SOURCE_FILES} ${HEADER_FILES} ${PROTO_FILES})
source_group(TREE ${CMAKE_CURRENT_BINARY_DIR} PREFIX "generated" FILES ${PROTO_SRCS} ${PROTO_HDRS})

target_link_libraries(${PROJECT_NAME} 
    protobuf::libprotobuf
    asio::asio
    tomlplusplus::tomlplusplus
    quill::quill)

target_compile_definitions(${PROJECT_NAME} PRIVATE TOML_EXCEPTIONS=0)
target_compile_definitions(${PROJECT_NAME} PRIVATE QUILL_ROOT_LOGGER_ONLY)

option(TRACE_BUILD_TIME "Use -ftime-trace to generate build time trace" OFF)
if (${TRACE_BUILD_TIME})
if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "[Cc]lang")
    target_compile_options(${PROJECT_NAME} PUBLIC -ftime-trace)
endif()
endif()

target_compile_definitions(${PROJECT_NAME} PUBLIC DATA_DIR="${CMAKE_SOURCE_DIR}/data")
target_compile_definitions(${PROJECT_NAME} PUBLIC LOG_DIR="${CMAKE_SOURCE_DIR}/log")

set_source_files_properties(${PROTO_SRCS} PROPERTIES SKIP_UNITY_BUILD_INCLUSION TRUE)
set_target_properties(${PROJECT_NAME} PROPERTIES UNITY_BUILD ${USE_UNITY_BUILDS})

set_source_files_properties(${C_SOURCE_FILES} PROPERTIES 
    SKIP_PRECOMPILE_HEADERS TRUE
    SKIP_UNITY_BUILD_INCLUSION TRUE)
target_precompile_headers(${PROJECT_NAME} PUBLIC ${PROTO_HDRS} ${HEADER_FILES})
