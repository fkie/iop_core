cmake_minimum_required(VERSION 2.8.3)
project(jaustoolset)

add_custom_target(jaustoolset ALL
    COMMENT "Add custom target to get and build JTS sources"
    COMMAND ${CMAKE_COMMAND} -P cmake/jts_build.cmake
    COMMAND "cmake/update_cmake_config.py" "${CMAKE_CURRENT_SOURCE_DIR}/../../jaustoolset/GUI" "${CMAKE_CURRENT_BINARY_DIR}/output" "${CMAKE_INSTALL_PREFIX}"
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

