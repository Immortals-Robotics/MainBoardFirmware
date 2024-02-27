# CMake helper macros
# Ali Salehi, Spring 2016

macro(enable_parallel_build)
  if (CMAKE_GENERATOR MATCHES "Visual Studio")
    add_compile_options("/MP")
  elseif(${CMAKE_GENERATOR} MATCHES "Unix Makefiles")
    message(STATUS ${CMAKE_BUILD_TOOL})
    include(ProcessorCount)
    ProcessorCount(CORES_COUNT)
    if(CORES_COUNT EQUAL 0)
      set(CORES_COUNT 4)
      message(STATUS "Failed to determine cpu cores, using 4")
    else()
      message(STATUS "Found ${CORES_COUNT} cpu cores")
    endif()
      set(CMAKE_MAKE_PROGRAM "${CMAKE_MAKE_PROGRAM} -j${CORES_COUNT}")
      message(STATUS "Added parallel build arguments to CMAKE_MAKE_PROGRAM")
  endif()
endmacro()

macro(set_project_paths)
  add_definitions(-DDATA_DIR="${CMAKE_SOURCE_DIR}/data")
endmacro()

macro(add_protobuf_generate_command PROTO_DEF PROTO_SRCS PROTO_HDRS)
  set(${PROTO_SRCS})
  set(${PROTO_HDRS})
  
  #TODO: don't hard-code .../protos
  set(DEF_BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/protos)
  set(GEN_BASE_DIR ${CMAKE_CURRENT_BINARY_DIR}/protos)

  foreach(file ${${PROTO_DEF}})
      get_filename_component(FILE_ABS ${file} ABSOLUTE)  # Full path to file
      get_filename_component(FILE_WE  ${file} NAME_WE)   # File name with neither the directory nor the longest extension
      get_filename_component(FILE_DIR ${file} DIRECTORY) # Directory without file name

      file(RELATIVE_PATH FILE_REL     ${DEF_BASE_DIR} ${FILE_ABS})
      file(RELATIVE_PATH FILE_DIR_REL ${DEF_BASE_DIR} ${FILE_DIR})

      set(GEN_DIR "${GEN_BASE_DIR}/${FILE_DIR_REL}")
      set(GEN_FILE_HEADER "${GEN_DIR}/${FILE_WE}.pb.h")
      set(GEN_FILE_SOURCE "${GEN_DIR}/${FILE_WE}.pb.cc")

      list(APPEND ${PROTO_SRCS} ${GEN_FILE_SOURCE})
      list(APPEND ${PROTO_HDRS} ${GEN_FILE_HEADER})

      add_custom_command(
        OUTPUT ${GEN_FILE_SOURCE}
               ${GEN_FILE_HEADER}
        COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
        ARGS --proto_path=${DEF_BASE_DIR} --cpp_out=${GEN_BASE_DIR} ${FILE_REL} 
        DEPENDS ${FILE_ABS}
        COMMENT "Running C++ protocol buffer compiler on ${FILE_REL}"
        VERBATIM )
  endforeach()
endmacro()
