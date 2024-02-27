project(${CMAKE_PROJECT_NAME}-tests)

include("${CMAKE_SOURCE_DIR}/cmake/helpers.cmake")
enable_parallel_build()

find_package(GTest CONFIG REQUIRED)
include(GoogleTest)

macro(add_test_file FILE_NAME)
	get_filename_component(FILE_WE  ${FILE_NAME} NAME_WE)
	message(STATUS "Adding ${FILE_WE} unit test")

	set(TEST_TARGET_NAME ${PROJECT_NAME}-${FILE_WE})

	add_executable(${TEST_TARGET_NAME} ${FILE_NAME})
	target_link_libraries(${TEST_TARGET_NAME} GTest::gtest_main ${CMAKE_PROJECT_NAME}-common)

	set_target_properties(${TEST_TARGET_NAME} PROPERTIES FOLDER tests)
	source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${FILE_NAME})

	target_precompile_headers(${TEST_TARGET_NAME} REUSE_FROM ${CMAKE_PROJECT_NAME}-common)

	gtest_discover_tests(${TEST_TARGET_NAME})
endmacro()
