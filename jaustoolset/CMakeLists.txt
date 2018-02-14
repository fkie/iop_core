cmake_minimum_required(VERSION 2.8.3)
project(jaustoolset)

if(NOT IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build/jaustoolset")
    message(STATUS "*********************************************************")
    message(STATUS "*                                                       *")
    message(STATUS "*  On the first run, a lot of stuff will be downloaded  *")
    message(STATUS "*  This may take a while...                             *")
    message(STATUS "*                                                       *")
    message(STATUS "*********************************************************")
endif()

message(STATUS "Fetching latest JAUSTOOLSET source code")
execute_process(
    COMMAND "build/get_jts_source"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    OUTPUT_FILE "${CMAKE_CURRENT_BINARY_DIR}/get_jts_source.stdout"
    ERROR_FILE "${CMAKE_CURRENT_BINARY_DIR}/get_jts_source.stderr"
    RESULT_VARIABLE get_jts_source_result
)
if(NOT ${get_jts_source_result} EQUAL 0)
    message(FATAL_ERROR "cannot update JAUSTOOLSET source code, see " "${CMAKE_CURRENT_BINARY_DIR}/get_jts_source.stderr")
endif()

add_custom_target(jaustoolset ALL
    COMMAND "build/get_jts_source"
    COMMAND "build/build_jts"
    COMMAND "build/update_cmake_config" "${CMAKE_CURRENT_SOURCE_DIR}/build/jaustoolset/GUI" "${CMAKE_CURRENT_BINARY_DIR}/output" "${CMAKE_INSTALL_PREFIX}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)

install(FILES "package.xml" DESTINATION "share/jaustoolset")
install(FILES "cfg/nm.cfg" DESTINATION "share/jaustoolset")
install(PROGRAMS "scripts/jaus_node_manager.sh" DESTINATION "share/jaustoolset")
install(
    FILES
        "${CMAKE_CURRENT_BINARY_DIR}/output/jaustoolsetConfig.cmake"
    DESTINATION "share/jaustoolset/cmake"
)
install(SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/output/make_install.cmake")

