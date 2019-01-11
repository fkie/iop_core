include(CMakeParseArguments)

if ( NOT EXISTS ${CMAKE_INSTALL_PREFIX} )
    # working directory in develspace
    get_filename_component(jaustoolset_WORKING_DIR "${jaustoolset_INCLUDE_DIRS}" PATH)
else()
    # working directory in installspace
    get_filename_component(jaustoolset_INSTALL_PATH "${jaustoolset_INCLUDE_DIRS}" PATH)
    set(jaustoolset_WORKING_DIR ${jaustoolset_INSTALL_PATH}/share/jaustoolset)
endif()

get_filename_component(JTS_LIB_DIR "${jaustoolset_LIBRARIES}" PATH)
set(jaustoolset_JAR ${JTS_LIB_DIR}/iop_codegenerator.jar)

set(jaustoolset_INCLUDE_JARS
    ${jaustoolset_WORKING_DIR}/libs_external/jargs-1.0/jargs.jar:${jaustoolset_WORKING_DIR}/libs_external/runtime/commons-lang-2.5.jar:${jaustoolset_WORKING_DIR}/libs_external/smc/Smc.jar
)
set(INCLUDE_LOCAL_JARS
    libs_external/jargs-1.0/jargs.jar
    libs_external/runtime/commons-lang-2.5.jar
    libs_external/smc/Smc.jar
)
